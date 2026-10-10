/*
 * Emulated RRU hardware: an SX1255 register file and converters that run
 * on the clock.
 *
 *  - SPI: register file with the datasheet reset values. STAT reports the
 *    oscillator and PLL lock (locked when a section is enabled and its
 *    latched frequency is inside 400..510 MHz).
 *  - RX: rx_read() blocks until the samples would have been captured at the
 *    sample rate (absolute-deadline pacing), then synthesises a tone at
 *    +rate/16 plus Gaussian noise, scaled by the LNA/PGA gain registers.
 *  - TX: tx_write() blocks for as long as the samples take to play out, so
 *    the RRU core consumes UL samples exactly at the sample rate. The
 *    average power of what is written (with the TX gain registers) feeds the
 *    supervision quantities while the driver is enabled.
 *  - Sensors: plausible temperature/voltage/current/RF readings.
 */
#include "hw.h"
#include "sx1255.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <cari/cari_proto.h>

#define DETECTOR_FLOOR_DBM  (-60.0)

struct rru_hw {
    rru_hw_cfg_t    cfg;
    pthread_mutex_t m;
    uint8_t         reg[SX1255_NREGS];
    uint32_t        frf_rx, frf_tx;         /* latched on the LSB write */
    bool            pa;

    uint32_t        rx_rate;
    struct timespec rx_next;
    double          rx_phase;
    unsigned        rx_seed;

    uint32_t        tx_rate;
    bool            tx_started;             /* the DAC clock runs from the first write */
    struct timespec tx_next;
    double          tx_avg_lin;             /* mW, averaged */
    bool            tx_seen;
    uint64_t        tx_late;

    double          temp, rl;
    struct timespec temp_t;
    unsigned        sens_seed;
};

static const uint8_t reset_regs[SX1255_NREGS] = {
    [SX1255_MODE] = 0x01,
    [SX1255_FRFH_RX] = 0xC0, [SX1255_FRFM_RX] = 0xE3, [SX1255_FRFL_RX] = 0x8E,
    [SX1255_FRFH_TX] = 0xC0, [SX1255_FRFM_TX] = 0xE3, [SX1255_FRFL_TX] = 0x8E,
    [SX1255_VERSION] = SX1255_CHIP_VERSION,
    [SX1255_TXFE1] = 0x2E, [SX1255_TXFE2] = 0x24, [SX1255_TXFE3] = 0x60, [SX1255_TXFE4] = 0x02,
    [SX1255_RXFE1] = 0x3F, [SX1255_RXFE2] = 0xF5, [SX1255_RXFE3] = 0x06,
    [SX1255_IO_MAP] = 0x00, [SX1255_CK_SEL] = 0x02, [SX1255_STAT] = 0x00,
    [SX1255_IISM] = 0x00, [SX1255_DIG_BRIDGE] = 0x00,
};

static double ts_sec(const struct timespec *t) { return (double)t->tv_sec + t->tv_nsec * 1e-9; }

static void ts_add(struct timespec *t, double sec)
{
    long ns = t->tv_nsec + (long)llround(sec * 1e9);
    t->tv_sec += ns / 1000000000L;
    t->tv_nsec = ns % 1000000000L;
}

static double randn(unsigned *seed)
{
    double u1 = (rand_r(seed) + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = (rand_r(seed) + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2 * M_PI * u2);
}

/* Sleep until the converter would have handled n more samples. */
static void pace(struct timespec *next, uint32_t rate, size_t n, uint64_t *late)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    /* resynchronise after a stall (e.g. the stream was idle) */
    if (ts_sec(next) < ts_sec(&now) - 0.050) {
        *next = now;
        if (late)
            (*late)++;
    }
    ts_add(next, (double)n / rate);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL))
        ;
}

static bool freq_ok(uint32_t w, double xtal)
{
    double f = sx1255_frf_hz(w, xtal);
    return f >= SX1255_FREQ_MIN - 1e3 && f <= SX1255_FREQ_MAX + 1e3;
}

/* ---------------------------------------------------------------- control */

static void do_reset(rru_hw_t *h)
{
    memcpy(h->reg, reset_regs, sizeof h->reg);
    h->frf_rx = h->frf_tx = 0xC0E38E;
    h->pa = false;
}

static rru_hw_t *d_open(const rru_hw_cfg_t *cfg)
{
    rru_hw_t *h = calloc(1, sizeof *h);
    if (!h)
        return NULL;
    h->cfg = *cfg;
    pthread_mutex_init(&h->m, NULL);
    do_reset(h);
    h->rx_seed = cfg->seed ? cfg->seed : 1;
    h->sens_seed = h->rx_seed * 7 + 3;
    h->temp = 31.0;
    h->rl = 18.5;
    clock_gettime(CLOCK_MONOTONIC, &h->temp_t);
    return h;
}

static void d_close(rru_hw_t *h)
{
    pthread_mutex_destroy(&h->m);
    free(h);
}

static int d_chip_reset(rru_hw_t *h)
{
    pthread_mutex_lock(&h->m);
    do_reset(h);
    pthread_mutex_unlock(&h->m);
    struct timespec ts = { 0, 5 * 1000000L };   /* datasheet: wait 5 ms */
    nanosleep(&ts, NULL);
    return 0;
}

static int d_spi_write(rru_hw_t *h, uint8_t a, uint8_t v)
{
    if (a >= SX1255_NREGS)
        return -1;
    pthread_mutex_lock(&h->m);
    if (a != SX1255_VERSION && a != SX1255_STAT) {
        h->reg[a] = v;
        if (a == SX1255_FRFL_RX)
            h->frf_rx = (uint32_t)h->reg[1] << 16 | (uint32_t)h->reg[2] << 8 | h->reg[3];
        if (a == SX1255_FRFL_TX)
            h->frf_tx = (uint32_t)h->reg[4] << 16 | (uint32_t)h->reg[5] << 8 | h->reg[6];
    }
    pthread_mutex_unlock(&h->m);
    if (h->cfg.verbose)
        fprintf(stderr, "  [sx1255] W %02X <- %02X\n", a, v);
    return 0;
}

static uint8_t stat_reg(rru_hw_t *h)
{
    uint8_t mode = h->reg[SX1255_MODE], st = 0;
    if (mode & SX1255_MODE_REF)
        st |= SX1255_STAT_XOSC;
    if ((mode & SX1255_MODE_REF) && (mode & SX1255_MODE_RX) && freq_ok(h->frf_rx, h->cfg.xtal_hz))
        st |= SX1255_STAT_LOCK_RX;
    if ((mode & SX1255_MODE_REF) && (mode & SX1255_MODE_TX) && freq_ok(h->frf_tx, h->cfg.xtal_hz))
        st |= SX1255_STAT_LOCK_TX;
    return st;
}

static int d_spi_read(rru_hw_t *h, uint8_t a, uint8_t *v)
{
    if (a >= SX1255_NREGS)
        return -1;
    pthread_mutex_lock(&h->m);
    *v = a == SX1255_STAT ? stat_reg(h) : h->reg[a];
    pthread_mutex_unlock(&h->m);
    return 0;
}

static int d_pa_enable(rru_hw_t *h, bool on)
{
    pthread_mutex_lock(&h->m);
    h->pa = on;
    pthread_mutex_unlock(&h->m);
    return 0;
}

/* ---------------------------------------------------------------- RX */

static int d_rx_open(rru_hw_t *h, uint32_t rate)
{
    h->rx_rate = rate;
    clock_gettime(CLOCK_MONOTONIC, &h->rx_next);
    return 0;
}

static long d_rx_read(rru_hw_t *h, int16_t *iq, size_t n)
{
    pace(&h->rx_next, h->rx_rate, n, NULL);

    pthread_mutex_lock(&h->m);
    uint8_t rxfe1 = h->reg[SX1255_RXFE1];
    bool on = (stat_reg(h) & SX1255_STAT_LOCK_RX) != 0;
    pthread_mutex_unlock(&h->m);

    float lna = sx1255_lna_db((rxfe1 >> 5) & 7);
    float pga = -24.0f + 2.0f * ((rxfe1 >> 1) & 0xF);   /* -24 .. +6 dB */
    if (isnan(lna))
        lna = -12.0f;                       /* unused code: very low gain */
    /* full gain: tone at about -10 dBFS; noise floor follows the PGA */
    double amp = on ? 0.3 * pow(10.0, (lna - 48.0 + pga - 6.0) / 20.0) : 0.0;
    double sigma = 0.002 * pow(10.0, (pga - 6.0) / 20.0) + 0.0002;
    double dphi = 2 * M_PI / 16.0;
    for (size_t i = 0; i < n; i++) {
        double I = amp * cos(h->rx_phase) + sigma * randn(&h->rx_seed);
        double Q = amp * sin(h->rx_phase) + sigma * randn(&h->rx_seed);
        h->rx_phase += dphi;
        if (h->rx_phase > 2 * M_PI)
            h->rx_phase -= 2 * M_PI;
        long vi = lrint(I * 32768.0), vq = lrint(Q * 32768.0);
        iq[2 * i] = (int16_t)(vi > 32767 ? 32767 : vi < -32768 ? -32768 : vi);
        iq[2 * i + 1] = (int16_t)(vq > 32767 ? 32767 : vq < -32768 ? -32768 : vq);
    }
    return (long)n;
}

static void d_rx_close(rru_hw_t *h) { (void)h; }

/* ---------------------------------------------------------------- TX */

static int d_tx_open(rru_hw_t *h, uint32_t rate)
{
    h->tx_rate = rate;
    h->tx_started = false;
    return 0;
}

static long d_tx_write(rru_hw_t *h, const int16_t *iq, size_t n)
{
    double acc = 0;
    for (size_t i = 0; i < 2 * n; i++)
        acc += (double)iq[i] * iq[i];
    double ms = n ? acc / n / (32768.0 * 32768.0) : 0;  /* mean |x|^2, |x|=1 full scale */

    pthread_mutex_lock(&h->m);
    uint8_t txfe1 = h->reg[SX1255_TXFE1];
    bool keyed = (h->reg[SX1255_MODE] & SX1255_MODE_PA) && (stat_reg(h) & SX1255_STAT_LOCK_TX);
    if (keyed) {
        float p = sx1255_tx_dbm((txfe1 >> 4) & 3, txfe1 & 0xF, h->cfg.pmax_dbm);
        double mw = pow(10.0, p / 10.0) * ms;
        h->tx_avg_lin = h->tx_seen ? 0.8 * h->tx_avg_lin + 0.2 * mw : mw;
        h->tx_seen = true;
    } else {
        h->tx_seen = false;
    }
    pthread_mutex_unlock(&h->m);

    if (!h->tx_started) {           /* like ALSA: playback starts with the first data */
        clock_gettime(CLOCK_MONOTONIC, &h->tx_next);
        h->tx_started = true;
    }
    pace(&h->tx_next, h->tx_rate, n, &h->tx_late);
    return (long)n;
}

static void d_tx_close(rru_hw_t *h)
{
    pthread_mutex_lock(&h->m);
    h->tx_seen = false;
    pthread_mutex_unlock(&h->m);
}

/* ---------------------------------------------------------------- sensors */

static int d_sensor(rru_hw_t *h, uint8_t q, float *v)
{
    pthread_mutex_lock(&h->m);
    uint8_t mode = h->reg[SX1255_MODE];
    bool rx = mode & SX1255_MODE_RX, tx = mode & SX1255_MODE_TX, pa = (mode & SX1255_MODE_PA) && h->tx_seen;
    double inc = pa && h->tx_avg_lin > 0 ? 10.0 * log10(h->tx_avg_lin) : DETECTOR_FLOOR_DBM;
    if (inc < DETECTOR_FLOOR_DBM)
        inc = DETECTOR_FLOOR_DBM;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = ts_sec(&now) - ts_sec(&h->temp_t);
    if (dt > 0.01)
        h->temp_t = now;
    double target = 34.0 + (rx ? 2.0 : 0.0) + (tx ? 3.0 : 0.0) + (pa ? 6.0 : 0.0);
    h->temp += (target - h->temp) * (1.0 - exp(-dt / 20.0));
    /* return loss wanders slowly, so RL and reflected power read in the same
     * supervision packet are consistent */
    if (dt > 0.01)
        h->rl += (18.5 - h->rl) * 0.05 + 0.05 * randn(&h->sens_seed);
    double rl = h->rl;
    int r = 0;
    switch (q) {
    case CARI_Q_TEMPERATURE:     *v = (float)(h->temp + 0.05 * randn(&h->sens_seed)); break;
    case CARI_Q_VOLTAGE:         *v = (float)(5.02 - (pa ? 0.06 : 0.0) + 0.004 * randn(&h->sens_seed)); break;
    case CARI_Q_CURRENT:         *v = (float)(0.21 + (rx ? 0.05 : 0) + (tx ? 0.07 : 0) + (pa ? 0.11 : 0)
                                              + 0.002 * randn(&h->sens_seed)); break;
    case CARI_Q_RETURN_LOSS:     *v = (float)rl; break;
    case CARI_Q_POWER_INCIDENT:  *v = (float)inc; break;
    case CARI_Q_POWER_REFLECTED: *v = (float)(pa ? inc - rl : DETECTOR_FLOOR_DBM); break;
    default:                     r = -1; break;
    }
    pthread_mutex_unlock(&h->m);
    return r;
}

const rru_hw_ops_t rru_hw_dummy = {
    .name = "dummy",
    .open = d_open,
    .close = d_close,
    .chip_reset = d_chip_reset,
    .spi_write = d_spi_write,
    .spi_read = d_spi_read,
    .pa_enable = d_pa_enable,
    .rx_open = d_rx_open,
    .rx_read = d_rx_read,
    .rx_close = d_rx_close,
    .tx_open = d_tx_open,
    .tx_write = d_tx_write,
    .tx_close = d_tx_close,
    .sensor = d_sensor,
};
