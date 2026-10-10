/*
 * Semtech SX1255 driver (register level)
 *
 * Numbers below are from the SX1255 datasheet, Rev. 3.1 (Semtech):
 *   - RF range 400..510 MHz (Table 2-4)
 *   - Frf = freq_rf * FXOSC / 2^20 (register table; the 2^19 in section
 *     3.3.3.3 contradicts the reset value 0xC0E38E = 434 MHz at 36 MHz)
 *   - LNA gain G1..G6 = 0, -6, -12, -24, -36, -48 dB (RXFE1 RxLnaGain);
 *     expressed here as 48, 42, 36, 24, 12, 0 dB above G6, like libsx1255
 *   - PGA gain = lowest + 2 dB * code, codes 0..15 (RXFE1 rx_pga_gain)
 *   - TX DAC gain 0/-3/-6/-9 dB (codes 3..0), mixer gain -37.5 + 2*code dB
 *     (TXFE1); peak output +5 dBm typ (TXP1dB), saturated +7 dBm typ
 *   - decimation/interpolation = MANT * 3^m * 2^n, MANT 8 or 9, m 0..1,
 *     n 0..6 (DIG_BRIDGE); CLK_OUT = XTAL / {1,2,4,8,12,16,24,32,48} (IISM)
 *   - full duplex requires equal RX/TX factors (one shared sample rate)
 */
#ifndef SX1255_H
#define SX1255_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#include "hw.h"

/* Registers */
#define SX1255_MODE         0x00
#define SX1255_FRFH_RX      0x01
#define SX1255_FRFM_RX      0x02
#define SX1255_FRFL_RX      0x03
#define SX1255_FRFH_TX      0x04
#define SX1255_FRFM_TX      0x05
#define SX1255_FRFL_TX      0x06
#define SX1255_VERSION      0x07
#define SX1255_TXFE1        0x08
#define SX1255_TXFE2        0x09
#define SX1255_TXFE3        0x0A
#define SX1255_TXFE4        0x0B
#define SX1255_RXFE1        0x0C
#define SX1255_RXFE2        0x0D
#define SX1255_RXFE3        0x0E
#define SX1255_IO_MAP       0x0F
#define SX1255_CK_SEL       0x10
#define SX1255_STAT         0x11
#define SX1255_IISM         0x12
#define SX1255_DIG_BRIDGE   0x13
#define SX1255_NREGS        0x14

/* MODE bits */
#define SX1255_MODE_REF     0x01
#define SX1255_MODE_RX      0x02
#define SX1255_MODE_TX      0x04
#define SX1255_MODE_PA      0x08    /* driver_enable */

/* STAT bits */
#define SX1255_STAT_LOCK_TX 0x01
#define SX1255_STAT_LOCK_RX 0x02
#define SX1255_STAT_XOSC    0x04
#define SX1255_STAT_EOL     0x08

#define SX1255_CHIP_VERSION 0x11    /* V1A */

#define SX1255_FREQ_MIN     400000000ull
#define SX1255_FREQ_MAX     510000000ull

/* ---- pure helpers (no I/O) ---- */

uint32_t sx1255_frf_word(uint64_t hz, double xtal_hz);
double   sx1255_frf_hz(uint32_t word, double xtal_hz);

/* LNA: gain in dB (one of the six steps) <-> RxLnaGain code 1..6. */
int      sx1255_lna_code(float db);                     /* -1 if not a step */
float    sx1255_lna_db(int code);
extern const float sx1255_lna_steps[6];                 /* 0 .. 48 */

/* TX: output power = pmax + DAC gain + (mixer gain + 7.5), i.e. pmax at
 * DAC code 3 and mixer code 15. Steps of 1 dB except 1 and 38 dB below max. */
typedef struct {
    uint8_t dac;        /* TxDacGain 0..3 */
    uint8_t mixer;      /* TXMixerGain 0..15 */
    float   dbm;        /* resulting output power */
} sx1255_txgain_t;
float sx1255_tx_dbm(uint8_t dac, uint8_t mixer, float pmax_dbm);
/* Nearest achievable setting (ties go to the lower power). */
void  sx1255_tx_gain_for(float dbm, float pmax_dbm, sx1255_txgain_t *out);
float sx1255_tx_min_dbm(float pmax_dbm);                /* pmax - 39 dB */

/* Sample rate: digital bridge factor + I2S clock divider (mode B2, one
 * I/Q pair per WS period, slot_bits bits per component). The RRU uses
 * 32-bit slots with the 16 significant bits MSB-aligned, as libsx1255
 * does: CLK_OUT = 64 x rate = XTAL / 4, 2, 1 for 125/250/500 kHz. */
#define SX1255_SLOT_BITS 32
typedef struct {
    uint8_t  mant9;     /* 0: MANT 8, 1: MANT 9 */
    uint8_t  m;         /* 0..1 */
    uint8_t  n;         /* 0..6 */
    uint8_t  div_code;  /* IISM iism_clk_div */
    uint32_t factor;    /* MANT * 3^m * 2^n */
    uint32_t clk_div;   /* XTAL / CLK_OUT */
} sx1255_rate_t;
int sx1255_rate_config(double xtal_hz, double rate, unsigned slot_bits, sx1255_rate_t *out);

/* ---- device ---- */

typedef struct {
    const rru_hw_ops_t *ops;
    rru_hw_t           *hw;
    double              xtal_hz;
    pthread_mutex_t     lock;       /* serialises SPI between threads */
    uint8_t             mode;       /* shadow of MODE */
    bool                comm_error; /* SPI failure or wrong chip version */
} sx1255_t;

void sx1255_attach(sx1255_t *d, const rru_hw_ops_t *ops, rru_hw_t *hw, double xtal_hz);
void sx1255_detach(sx1255_t *d);
int  sx1255_write(sx1255_t *d, uint8_t reg, uint8_t val);
int  sx1255_read(sx1255_t *d, uint8_t reg, uint8_t *val);

/* Reset pulse, version check, static configuration (I2S mode B2, MSB
 * aligned, CLK_OUT enabled, oscillator on, RXFE2/TXFE4 as libsx1255). */
int  sx1255_init(sx1255_t *d);
/* ppm: crystal correction applied to the frequency word. */
int  sx1255_set_freq(sx1255_t *d, bool tx, uint64_t hz, float ppm);
int  sx1255_set_rx_gain(sx1255_t *d, int lna_code, int pga_code);
int  sx1255_set_tx_gain(sx1255_t *d, uint8_t dac, uint8_t mixer);
int  sx1255_set_rate(sx1255_t *d, const sx1255_rate_t *r);
/* Update the MODE bits under mask. */
int  sx1255_set_mode(sx1255_t *d, uint8_t mask, uint8_t bits);
/* PLL lock of the enabled sections; true if nothing is enabled. */
bool sx1255_locked(sx1255_t *d, bool rx, bool tx);
/* Wait up to timeout_us for the PLL(s) to lock. */
bool sx1255_wait_lock(sx1255_t *d, bool rx, bool tx, int timeout_us);

#endif
