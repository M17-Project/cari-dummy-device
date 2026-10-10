# cari-dummy-device

CARI 2.1 slave that emulates the planned **M17 SX1255 RRU** (Raspberry Pi + Semtech SX1255 with a 32 MHz crystal, I2S samples, SPI control). Register settings follow [libsx1255](https://github.com/M17-Project/libsx1255). Use it to develop and test CARI masters without hardware. It is built on [libcari](https://github.com/M17-Project/libcari) and supports [CARI 2.1](https://github.com/M17-Project/CARI/blob/main/docs/commands.md).

## What it presents

One subdevice (0), full duplex, I/Q only:

| | Value | Source |
|---|---|---|
| Subdevice capabilities | reset, I/Q, RX, TX, full duplex, CS16 | |
| RX / TX frequency | 400 000 000 .. 510 000 000 Hz | SX1255 DS Table 2-4 |
| Sample rate | 125 000, 250 000, 500 000 Hz (three equal-bound pairs) | 32 MHz / (8 · 2ⁿ), n = 5, 4, 3 |
| LNA gain | 0, 12, 24, 36, 42, 48 dB (six equal-bound pairs) | RxLnaGain G6..G1, dB above G6 as in libsx1255 |
| Output power | −34 .. +5 dBm, 1 dB steps, value read back quantized | TxDacGain × TXMixerGain, P1dB +5 dBm typ |
| Device capabilities | compression (FLAC), supervision | |
| Supervision | temperature, voltage, current, RL, incident and reflected power | |
| Frequency correction | ±100 ppm, applied to the crystal in the frequency words | |

Parameters not listed (channel width, AGC, AFC, DC correction, deviation) return status 2.

### SX1255 numbers checked against the datasheet (Rev. 3.1) and libsx1255

- **RF range** 400–510 MHz.
- **Frequency word** `Frf = f · 2^20 / FXOSC`. Section 3.3.3.3 prints 2^19, the register table 2^20; the reset value 0xC0E38E = 434.000 MHz at 36 MHz only works with 2^20. libsx1255 uses 2^20 as well.
- **LNA**: six steps (G1..G6 = 0, −6, −12, −24, −36, −48 dB relative to the highest gain). Expressed, like libsx1255, as dB above the lowest step: 48, 42, 36, 24, 12, 0. The datasheet gives no absolute gain. The PGA (codes 0..15, 2 dB steps; the datasheet calls the span both "30" and "32 dB") stays at a fixed code (`--pga`, default 15) because the RRU offers no AGC.
- **TX gain**: DAC 0/−3/−6/−9 dB × mixer −37.5 + 2·code dB (code 0..15) → 39 dB of control in 1 dB steps (1 and 38 dB below maximum are not reachable and round down). The datasheet gives no absolute power per code; the maximum is taken as the typical peak output P1dB = **+5 dBm** (saturated +7 dBm typ, minimum P1dB +2 dBm), so the range is **−34..+5 dBm**. CARI defines |x| = 1 as the set output power, i.e. peak, which matches a P1dB reference. With an external PA or after calibration, set `--pmax`.
- **Sample rates**: 32 MHz / (8 · 2ⁿ), n = 5, 4, 3 → 125/250/500 kHz. I2S mode B2 with **32-bit slots**, 16 significant bits MSB-aligned, CLK_OUT = XTAL / 4, 2, 1 (= 64 × rate) - the IISM/DIG_BRIDGE values libsx1255 writes.
- **Full duplex** requires identical RX/TX decimation/interpolation - one shared sample-rate parameter fits.
- **Static settings from libsx1255**: RXFE2 = 0x37 (ADC bandwidth code 1, ADC trim 5, analog pre-filter 500 kHz) for all rates, TXFE4 = 5 (64 FIR-DAC taps), SPI mode 0 at 500 kHz, RESET on GPIO 22 of gpiochip0. The datasheet's ADC-bandwidth guidance would pick different codes per rate; the RRU's values are used.

## Architecture

```
 CARI master ──CTRL──► libcari slave ──callbacks──► RRU core (rru.c)
                                                    │  SX1255 driver (sx1255.c, register level)
   UL PUSH ──► PULL ─► feeder thread ─► ring buffer ─► drain thread ─► hw->tx_write()
   DL SUB  ◄── PUB  ◄─ DL thread ◄──────────────────────────────────── hw->rx_read()
                                                    │
                                     rru_hw_ops_t backend (hw.h)
                                     ├─ hw_dummy.c  emulated chip + converters on the clock
                                     └─ hw_rpi.c    spidev + GPIO (I2S/ALSA: TODO)
```

- **DL**: `rx_read()` blocks until the samples would have been captured at the sample rate (absolute deadlines, no drift), then the DL thread publishes 10 ms per message (raw or one FLAC frame). `--period` sets the message length, which is also the DL latency.
- **UL**: `tx_write()` blocks for as long as the samples take to play out (the dummy's DAC clock starts with the first write, as ALSA playback does). The drain writes 1 ms at a time (`--ul-period`). The feeder reads the PULL socket only when the ring (60 ms) has room, so ZeroMQ flow control holds back a master that sends too fast; the slave's sample clock is the timing reference. The UL socket uses a 32 KiB receive buffer to keep the queue short.
- **Keying**: the PA is keyed once the prefill is buffered, with a 1 ms ramp-up. The prefill adapts to the master: largest UL message seen + drain period + ramp reserve + jitter margin (`--jitter`, default 5 ms), or a fixed `--prefill`. A master that is ahead of time keys within about a millisecond; a master sending in real time keys after the first message that completes the prefill.
- **Underrun**: a period is played only while one period plus one ramp length (1 ms) stays behind it. When less is left the buffer is about to run dry: all remaining samples are ramped down to zero and played, the PA is unkeyed and the trip is reported (`cari_slave_ul_underrun`). TX start with no samples arriving never keys and never trips.

UL key-up delay for a master sending in real time, with the defaults (measured with `test_latency` at the default 5 ms margin; the measurements match the model to within 0.3 ms):

| Master message | 1 ms | 2 ms | 5 ms | 10 ms | 20 ms |
|---|---|---|---|---|---|
| Prefill (msg + 1 + 1 + 5 ms) | 8 ms | 9 ms | 12 ms | 17 ms | 27 ms |
| Key-up after first sample | 7 ms | 8 ms | 10 ms | 10 ms | 20 ms |

The delay is set by the master's message size and the jitter margin, not by a constant: samples arrive one message at a time, so the buffer must bridge a whole message gap plus whatever jitter the network and the scheduler add. On a dedicated RRU with wired Ethernet 5 ms is comfortable; in a VM or over Wi-Fi raise `--jitter`. (The previous fixed 20 ms prefill with a 10 ms drain period tripped a real-time master sending 10 ms messages.)
- **End of transmission** (CARI 2.1): when the feeder reads the zero-byte marker it stops reading the socket. The drain plays every buffered sample, ramps the last 1 ms down, unkeys and reports the end; the subdevice returns to idle without a trip. A marker with no samples before it ends the transmission without keying.
- **TX stop**: aborts - ramps down over the next 1 ms of samples and unkeys; buffered samples are discarded.

The emulated chip has the datasheet reset values, latches frequencies on the LSB write, reports oscillator and PLL lock in STAT, scales the synthetic RX signal (a tone at +rate/16 plus noise) by the LNA/PGA registers, and derives the supervised incident power from what is actually transmitted and the TX gain registers.

### Using the core for the real RRU

Everything above `rru_hw_ops_t` is shared. A hardware backend implements: chip reset line, SPI register read/write, an optional PA/T-R line, blocking CS16 sample I/O paced by the converter clock (`rx_read`, `tx_write`), and sensors. `hw_rpi.c` has the SPI (spidev, SX1255 `wnr` bit 7, mode 0, 500 kHz) and GPIO (character device v2, RESET on GPIO 22) parts and marks where the ALSA code goes (S32_LE, 2 channels, upper 16 bits of each slot, slave clocking, short playback buffer - the core's ring is where ramping happens). libsx1255 itself is not linked: it talks to spidev directly with global state, so the dummy could not emulate the chip underneath it. Select it with `--hw rpi`.

## Build and run

```sh
git clone --recursive https://github.com/M17-Project/cari-dummy-device
cd cari-dummy-device               # in an existing clone: git submodule update --init
cmake -S . -B build
cmake --build build
./build/cari-dummy-device -v
```

libcari comes as a git submodule in `libcari/` and is linked statically, so the installed program needs nothing else from libcari. Other ways to provide it, in the order they are tried: `-DCARI_SOURCE_DIR=/path/to/libcari`, the submodule, a `libcari` checkout next to this repository, an installed libcari (`find_package(cari 2)`).

```
  -c, --ctrl=PORT|EP     CTRL REP port or ZMQ endpoint (default 17002)
  -b, --bind=HOST        data-plane bind address (default * = all interfaces)
  -i, --ident=STR        IDENT string
  -x, --xtal=HZ          SX1255 crystal (default 32e6)
  -P, --pmax=DBM         output power at full TX gain (default 5.0)
  -g, --pga=CODE         RX PGA gain code 0..15 (default 15 = +6 dB)
  -t, --period=MS        DL message period (default 10)
      --ul-period=US     UL drain period (default 1000)
  -u, --ul-buffer=MS     UL ring buffer (default 60)
  -p, --prefill=MS       UL samples buffered before keying (default 0 = automatic)
      --jitter=MS        jitter margin of the automatic prefill (default 5)
      --ul-rcvbuf=BYTES  UL socket receive buffer (default 32768)
  -f, --freq=HZ          default RX/TX frequency (default 433475000)
  -H, --hw=NAME          dummy (default) or rpi (skeleton)
  -v, --verbose          -v info, -vv debug, -vvv SPI trace
```

User registers: `0x02` backend (0 dummy, 1 rpi, read-only); `0x80..0x93` mirror SX1255 registers 0x00..0x13 (read-only) - e.g. `cari-ctrl reg 0x87` reads the chip version 0x11.

A subdevice reset (action 0x00) pulses the chip reset and reprograms the current settings. A device reset (write to register 0x00) restores the defaults: 433.475 MHz, 250 kHz, LNA 48 dB (highest), −10 dBm, compression off.

## Tests

```sh
ctest --test-dir build
```

- `test_sx1255` - datasheet values and encodings (frequency word, LNA/TX gain tables, rate/divider search, register writes).
- `test_ring` - UL ring buffer (wrap, blocking writes, wake-ups).
- `test_latency` - RRU core in-process: with the end marker, every sample sent is played (counted exactly) and no trip occurs, for real-time and ahead-of-time masters; with masters sending 1/2/5/10/20 ms messages in real time: key-up delay against the prefill model, no trip while the master keeps up (a session is retried once, because shared VMs stall threads for tens of milliseconds), and immediate keying for a master that is ahead.
- `test_e2e` - spawns the device and drives it with libcari: the advertised profile, parameter rules and quantization, the SX1255 registers behind each parameter, DL measured at 125/250/500 kHz (±3 %), FLAC DL, UL backpressure and **trip timing** (the trip arrives when the sent samples have been played at the sample rate, e.g. 1.0 s of samples → trip after 1.01 s), trip/clear/start-refused rules, FLAC UL, full duplex at 500 kHz with supervision (incident power = set power + 20·log10|x|), resets and clean shutdown.

Clean under ASan/UBSan and ThreadSanitizer.
