/*
 * Semtech SX1255 driver (register level). See sx1255.h for the datasheet
 * values used here.
 */
#include "sx1255.h"

#include <math.h>
#include <string.h>
#include <time.h>

/* LNA gain above the lowest setting (G6), the convention of libsx1255:
 * G1 = 48 dB (highest) ... G6 = 0 dB. */
const float sx1255_lna_steps[6] = { 0.0f, 12.0f, 24.0f, 36.0f, 42.0f, 48.0f };
static const float lna_by_code[7] = { NAN, 48.0f, 42.0f, 36.0f, 24.0f, 12.0f, 0.0f };
static const uint32_t clk_divs[9] = { 1, 2, 4, 8, 12, 16, 24, 32, 48 };

uint32_t sx1255_frf_word(uint64_t hz, double xtal_hz)
{
    double w = (double)hz * 1048576.0 / xtal_hz;
    if (w < 0)
        w = 0;
    if (w > 16777215.0)
        w = 16777215.0;
    return (uint32_t)llround(w);
}

double sx1255_frf_hz(uint32_t word, double xtal_hz)
{
    return (double)word * xtal_hz / 1048576.0;
}

int sx1255_lna_code(float db)
{
    for (int c = 1; c <= 6; c++)
        if (lna_by_code[c] == db)
            return c;
    return -1;
}

float sx1255_lna_db(int code)
{
    return code >= 1 && code <= 6 ? lna_by_code[code] : NAN;
}

float sx1255_tx_dbm(uint8_t dac, uint8_t mixer, float pmax_dbm)
{
    /* DAC: code 3 = 0 dB ... code 0 = -9 dB. Mixer: -37.5 + 2*code dB, the
     * maximum (code 15, -7.5 dB) is the reference for pmax. */
    return pmax_dbm + 3.0f * ((float)dac - 3.0f) + 2.0f * ((float)mixer - 15.0f);
}

float sx1255_tx_min_dbm(float pmax_dbm)
{
    return sx1255_tx_dbm(0, 0, pmax_dbm);
}

void sx1255_tx_gain_for(float dbm, float pmax_dbm, sx1255_txgain_t *out)
{
    float best_err = INFINITY;
    out->dac = 3;
    out->mixer = 15;
    out->dbm = pmax_dbm;
    /* prefer the higher DAC gain (better SNR) among equal results */
    for (int dac = 3; dac >= 0; dac--)
        for (int mix = 15; mix >= 0; mix--) {
            float p = sx1255_tx_dbm((uint8_t)dac, (uint8_t)mix, pmax_dbm);
            float err = fabsf(p - dbm);
            /* ties: the lower power wins (never exceed by rounding) */
            if (err < best_err - 1e-6f || (fabsf(err - best_err) <= 1e-6f && p < out->dbm)) {
                best_err = err;
                out->dac = (uint8_t)dac;
                out->mixer = (uint8_t)mix;
                out->dbm = p;
            }
        }
}

int sx1255_rate_config(double xtal_hz, double rate, unsigned slot_bits, sx1255_rate_t *out)
{
    if (!(rate > 0) || slot_bits < 8 || slot_bits > 32)
        return -1;
    for (int mant9 = 0; mant9 <= 1; mant9++)
        for (int m = 0; m <= 1; m++)
            for (int n = 0; n <= 6; n++) {
                uint32_t factor = (uint32_t)((mant9 ? 9 : 8) * (m ? 3 : 1)) << n;
                if (fabs(xtal_hz / factor - rate) > 1e-3)
                    continue;
                /* I2S mode B2: one WS period per sample, two slots (I, Q) */
                double bclk = rate * 2.0 * slot_bits;
                for (int dc = 0; dc < 9; dc++) {
                    if (fabs(xtal_hz / clk_divs[dc] - bclk) > 1e-3)
                        continue;
                    out->mant9 = (uint8_t)mant9;
                    out->m = (uint8_t)m;
                    out->n = (uint8_t)n;
                    out->factor = factor;
                    out->div_code = (uint8_t)dc;
                    out->clk_div = clk_divs[dc];
                    return 0;
                }
            }
    return -1;
}

/* ---------------------------------------------------------------- device */

void sx1255_attach(sx1255_t *d, const rru_hw_ops_t *ops, rru_hw_t *hw, double xtal_hz)
{
    memset(d, 0, sizeof *d);
    d->ops = ops;
    d->hw = hw;
    d->xtal_hz = xtal_hz;
    pthread_mutex_init(&d->lock, NULL);
}

void sx1255_detach(sx1255_t *d)
{
    pthread_mutex_destroy(&d->lock);
}

static int wr(sx1255_t *d, uint8_t reg, uint8_t val)
{
    int r = d->ops->spi_write(d->hw, reg, val);
    if (r)
        d->comm_error = true;
    return r;
}

int sx1255_write(sx1255_t *d, uint8_t reg, uint8_t val)
{
    pthread_mutex_lock(&d->lock);
    int r = wr(d, reg, val);
    pthread_mutex_unlock(&d->lock);
    return r;
}

int sx1255_read(sx1255_t *d, uint8_t reg, uint8_t *val)
{
    pthread_mutex_lock(&d->lock);
    int r = d->ops->spi_read(d->hw, reg, val);
    if (r)
        d->comm_error = true;
    pthread_mutex_unlock(&d->lock);
    return r;
}

/* RXFE2 as libsx1255 (M17 RRU, 32 MHz crystal) sets it: rx_adc_bw 1,
 * rx_adc_trim 5, analog pre-filter 500 kHz. Kept for all sample rates. */
#define RXFE2_RRU ((1 << 5) | (5 << 2) | 3)

int sx1255_init(sx1255_t *d)
{
    uint8_t v = 0;
    pthread_mutex_lock(&d->lock);
    d->comm_error = false;
    int r = d->ops->chip_reset(d->hw);
    if (!r)
        r = d->ops->spi_read(d->hw, SX1255_VERSION, &v);
    if (r || v != SX1255_CHIP_VERSION) {
        d->comm_error = true;
        pthread_mutex_unlock(&d->lock);
        return -1;
    }
    d->mode = SX1255_MODE_REF;
    r |= wr(d, SX1255_MODE, d->mode);
    r |= wr(d, SX1255_CK_SEL, 0x02);                    /* CLK_OUT on, internal DAC clock */
    r |= wr(d, SX1255_RXFE2, RXFE2_RRU);
    r |= wr(d, SX1255_TXFE4, 5);                        /* 64 FIR-DAC taps (libsx1255) */
    r |= wr(d, SX1255_RXFE3, 3 << 1);                   /* RX PLL BW 300 kHz */
    r |= wr(d, SX1255_TXFE3, 3 << 5);                   /* TX PLL BW 300 kHz, filter 418 kHz DSB */
    r |= wr(d, SX1255_IISM, 0x20);                      /* mode B2 (standard I2S), div set later */
    r |= wr(d, SX1255_DIG_BRIDGE, 0x04);                /* MSB aligned (LSBs truncated) */
    pthread_mutex_unlock(&d->lock);
    return r ? -1 : 0;
}

int sx1255_set_freq(sx1255_t *d, bool tx, uint64_t hz, float ppm)
{
    double xtal = d->xtal_hz * (1.0 + ppm * 1e-6);
    uint32_t w = sx1255_frf_word(hz, xtal);
    uint8_t base = tx ? SX1255_FRFH_TX : SX1255_FRFH_RX;
    pthread_mutex_lock(&d->lock);
    int r = wr(d, base, (uint8_t)(w >> 16));
    r |= wr(d, base + 1, (uint8_t)(w >> 8));
    r |= wr(d, base + 2, (uint8_t)w);                   /* the LSB write latches */
    pthread_mutex_unlock(&d->lock);
    return r ? -1 : 0;
}

int sx1255_set_rx_gain(sx1255_t *d, int lna_code, int pga_code)
{
    if (lna_code < 1 || lna_code > 6 || pga_code < 0 || pga_code > 15)
        return -1;
    /* rx_zin_200 = 0: 50 ohm input */
    return sx1255_write(d, SX1255_RXFE1, (uint8_t)((lna_code << 5) | (pga_code << 1)));
}

int sx1255_set_tx_gain(sx1255_t *d, uint8_t dac, uint8_t mixer)
{
    if (dac > 3 || mixer > 15)
        return -1;
    return sx1255_write(d, SX1255_TXFE1, (uint8_t)((dac << 4) | mixer));
}

int sx1255_set_rate(sx1255_t *d, const sx1255_rate_t *rc)
{
    pthread_mutex_lock(&d->lock);
    int r = wr(d, SX1255_DIG_BRIDGE,
               (uint8_t)((rc->mant9 << 7) | (rc->m << 6) | (rc->n << 3) | 0x04));
    r |= wr(d, SX1255_IISM, (uint8_t)(0x20 | rc->div_code));
    pthread_mutex_unlock(&d->lock);
    return r ? -1 : 0;
}

int sx1255_set_mode(sx1255_t *d, uint8_t mask, uint8_t bits)
{
    pthread_mutex_lock(&d->lock);
    d->mode = (uint8_t)((d->mode & ~mask) | (bits & mask) | SX1255_MODE_REF);
    int r = wr(d, SX1255_MODE, d->mode);
    pthread_mutex_unlock(&d->lock);
    return r;
}

bool sx1255_locked(sx1255_t *d, bool rx, bool tx)
{
    uint8_t st = 0;
    if (sx1255_read(d, SX1255_STAT, &st))
        return false;
    if (rx && !(st & SX1255_STAT_LOCK_RX))
        return false;
    if (tx && !(st & SX1255_STAT_LOCK_TX))
        return false;
    return true;
}

bool sx1255_wait_lock(sx1255_t *d, bool rx, bool tx, int timeout_us)
{
    for (int t = 0; t <= timeout_us; t += 100) {
        if (sx1255_locked(d, rx, tx))
            return true;
        struct timespec ts = { 0, 100000 };
        nanosleep(&ts, NULL);
    }
    return false;
}
