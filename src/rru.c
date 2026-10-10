/*
 * SX1255 RRU core - see rru.h.
 */
#include "rru.h"
#include "ring.h"
#include "sx1255.h"

#include <cari/cari_config.h>
#include <cari/cari_log.h>
#include <cari/cari_slave.h>

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define SUB 0                               /* the one subdevice */
#define MIRROR_BASE 0x80                    /* user regs 0x80..0x93 = SX1255 regs */
#define REG_BACKEND 0x02                    /* user reg: 0 dummy, 1 rpi */

#define LD(x)     __atomic_load_n(&(x), __ATOMIC_SEQ_CST)
#define ST(x, v)  __atomic_store_n(&(x), (v), __ATOMIC_SEQ_CST)
#define ADD(x, v) __atomic_fetch_add(&(x), (v), __ATOMIC_RELAXED)

static const float candidate_rates[] = { 125000.0f, 250000.0f, 500000.0f };
static const uint8_t quantities[] = {
    CARI_Q_TEMPERATURE, CARI_Q_VOLTAGE, CARI_Q_CURRENT,
    CARI_Q_RETURN_LOSS, CARI_Q_POWER_INCIDENT, CARI_Q_POWER_REFLECTED,
};

struct rru {
    rru_cfg_t       cfg;
    rru_hw_t       *hw;
    sx1255_t        sx;
    cari_slave_t   *slave;

    uint8_t         caps[256];
    size_t          caps_len;
    uint8_t         devcaps[8];
    size_t          devcaps_len;
    float           rates[3];
    sx1255_rate_t   rate_cfg[3];
    int             nrates;

    /* current settings (CTRL thread) */
    uint64_t        rx_freq, tx_freq;
    float           ppm;
    int             lna_code;
    sx1255_txgain_t txg;
    int             rate_idx;

    /* DL */
    pthread_t       dl_th;
    int             dl_run;
    bool            dl_active;
    size_t          dl_period;

    /* UL */
    pthread_t       feed_th, drain_th;
    int             ul_run, tripped, keyed;
    int             ul_end;             /* end marker read (atomic) */
    bool            ul_active;
    ring_t          ring;
    size_t          ul_period, ul_prefill, ul_jitter, ramp_len;
    size_t          ul_msg_max;         /* largest UL message so far (atomic) */
    int64_t         ul_first_ns;        /* first UL sample arrived (atomic) */
    int64_t         ul_last_ns;         /* last UL message arrived (atomic) */

    rru_stats_t     st;
};

void rru_cfg_defaults(rru_cfg_t *c)
{
    memset(c, 0, sizeof *c);
    c->hw = &rru_hw_dummy;
    c->hw_cfg.xtal_hz = 32e6;
    c->hw_cfg.pmax_dbm = 5.0f;
    c->hw_cfg.gpio_reset = 22;                      /* RRU wiring (libsx1255) */
    c->hw_cfg.gpio_pa = -1;
    c->hw_cfg.seed = 1;
    c->ident = "M17 SX1255 RRU (dummy)";
    c->ctrl_endpoint = "tcp://*:17002";
    c->bind_host = "*";
    c->pga_code = 15;
    c->period_ms = 10;
    c->ul_buffer_ms = 60;
    c->ul_period_us = 1000;
    c->ul_prefill_ms = 0;                           /* auto */
    c->ul_jitter_ms = 5;
    c->ramp_us = 1000;
    c->ul_rcvbuf = 32 * 1024;
    c->default_freq = 433475000;
    c->default_power = -10.0f;
    c->default_rate = 250000.0f;
}

/* ---------------------------------------------------------------- hardware */

static int rate_index(rru_t *r, float rate)
{
    for (int i = 0; i < r->nrates; i++)
        if (r->rates[i] == rate)
            return i;
    return -1;
}

/* Program the chip with the current settings. */
static int program_all(rru_t *r)
{
    int e = 0;
    e |= sx1255_set_freq(&r->sx, false, r->rx_freq, r->ppm);
    e |= sx1255_set_freq(&r->sx, true, r->tx_freq, r->ppm);
    e |= sx1255_set_rx_gain(&r->sx, r->lna_code, r->cfg.pga_code);
    e |= sx1255_set_tx_gain(&r->sx, r->txg.dac, r->txg.mixer);
    e |= sx1255_set_rate(&r->sx, &r->rate_cfg[r->rate_idx]);
    return e;
}

/* Defaults into the libcari parameter cache and into the chip. */
static int apply_defaults(rru_t *r)
{
    r->rx_freq = r->tx_freq = r->cfg.default_freq;
    r->ppm = 0.0f;
    r->lna_code = sx1255_lna_code(48.0f);              /* highest gain */
    sx1255_tx_gain_for(r->cfg.default_power, r->cfg.hw_cfg.pmax_dbm, &r->txg);
    r->rate_idx = rate_index(r, r->cfg.default_rate);
    if (r->rate_idx < 0)
        r->rate_idx = 0;

    cari_slave_t *s = r->slave;
    cari_slave_param_init(s, SUB, CARI_P_RX_FREQ, cari_value_u64(r->rx_freq));
    cari_slave_param_init(s, SUB, CARI_P_TX_FREQ, cari_value_u64(r->tx_freq));
    cari_slave_param_init(s, SUB, CARI_P_LNA_GAIN, cari_value_f32(48.0f));
    cari_slave_param_init(s, SUB, CARI_P_TX_POWER, cari_value_f32(r->txg.dbm));
    cari_slave_param_init(s, SUB, CARI_P_SAMPLE_RATE, cari_value_f32(r->rates[r->rate_idx]));
    cari_slave_param_init(s, SUB, CARI_P_FREQ_CORR, cari_value_f32(0.0f));
    cari_slave_param_init(s, SUB, CARI_P_FORMAT, cari_value_u8(CARI_FMT_CS16));
    cari_slave_param_init(s, SUB, CARI_P_MODE, cari_value_u8(CARI_MODE_IQ));
#ifdef CARI_HAVE_FLAC
    cari_slave_param_init(s, SUB, CARI_P_COMPRESSION, cari_value_bool(false));
#endif
    return program_all(r);
}

/* ---------------------------------------------------------------- DL */

static void *dl_thread(void *p)
{
    rru_t *r = p;
    int16_t *buf = malloc(r->dl_period * 4);
    if (!buf)
        return NULL;
    while (LD(r->dl_run)) {
        long n = r->cfg.hw->rx_read(r->hw, buf, r->dl_period);
        if (n <= 0) {
            cari_log(CARI_LOG_ERROR, "rx_read failed");
            break;
        }
        cari_slave_dl_write(r->slave, SUB, buf, (size_t)n);
        ADD(r->st.dl_produced, (uint64_t)n);
    }
    free(buf);
    return NULL;
}

static int dl_start(rru_t *r)
{
    uint32_t rate = (uint32_t)r->rates[r->rate_idx];
    r->dl_period = rate * r->cfg.period_ms / 1000;
    sx1255_set_mode(&r->sx, SX1255_MODE_RX, SX1255_MODE_RX);
    if (!sx1255_wait_lock(&r->sx, true, false, 2000))
        cari_log(CARI_LOG_WARN, "RX PLL not locked");
    if (r->cfg.hw->rx_open(r->hw, rate) < 0) {
        sx1255_set_mode(&r->sx, SX1255_MODE_RX, 0);
        return CARI_S_INVALID_STATE;
    }
    ST(r->dl_run, 1);
    if (pthread_create(&r->dl_th, NULL, dl_thread, r)) {
        r->cfg.hw->rx_close(r->hw);
        sx1255_set_mode(&r->sx, SX1255_MODE_RX, 0);
        return CARI_S_INVALID_STATE;
    }
    r->dl_active = true;
    return CARI_OK;
}

static void dl_stop(rru_t *r)
{
    if (!r->dl_active)
        return;
    ST(r->dl_run, 0);
    pthread_join(r->dl_th, NULL);
    r->cfg.hw->rx_close(r->hw);
    sx1255_set_mode(&r->sx, SX1255_MODE_RX, 0);
    r->dl_active = false;
}

/* ---------------------------------------------------------------- UL */

static void key(rru_t *r, bool on)
{
    sx1255_set_mode(&r->sx, SX1255_MODE_PA, on ? SX1255_MODE_PA : 0);
    if (r->cfg.hw->pa_enable) {
        pthread_mutex_lock(&r->sx.lock);
        r->cfg.hw->pa_enable(r->hw, on);
        pthread_mutex_unlock(&r->sx.lock);
    }
    ST(r->keyed, on ? 1 : 0);
}

/* Linear ramp from 1 down to 0 over n samples. */
static void ramp_down(int16_t *iq, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        float g = 1.0f - (float)(i + 1) / (float)n;
        iq[2 * i] = (int16_t)lrintf(iq[2 * i] * g);
        iq[2 * i + 1] = (int16_t)lrintf(iq[2 * i + 1] * g);
    }
}

static int64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

static void *feed_thread(void *p)
{
    rru_t *r = p;
    while (LD(r->ul_run) && !LD(r->tripped) && !LD(r->ul_end)) {
        const void *d;
        size_t n;
        int rd = cari_slave_ul_read(r->slave, SUB, &d, &n, 20);
        if (rd == CARI_UL_END) {
            /* End of transmission: everything before it is in the ring;
             * the socket is not read again until the next start. */
            ST(r->ul_end, 1);
            break;
        }
        if (rd != 1)
            continue;
        int64_t t = mono_ns();
        ST(r->ul_last_ns, t);
        if (!LD(r->ul_first_ns))
            ST(r->ul_first_ns, t);
        if (n > LD(r->ul_msg_max))
            ST(r->ul_msg_max, n);
        const int16_t *iq = d;
        /* Block here while the ring is full: we stop reading the socket and
         * ZMQ flow control holds the master back. */
        while (n && LD(r->ul_run) && !LD(r->tripped)) {
            size_t w = ring_write(&r->ring, iq, n, 20);
            iq += 2 * w;
            n -= w;
        }
    }
    return NULL;
}

/* Linear ramp-up over the first `ramp` samples after keying. */
static void apply_keyon_ramp(int16_t *iq, size_t n, size_t *left, size_t ramp)
{
    for (size_t i = 0; *left && i < n; i++, (*left)--) {
        float g = 1.0f - (float)*left / (float)ramp;
        iq[2 * i] = (int16_t)lrintf(iq[2 * i] * g);
        iq[2 * i + 1] = (int16_t)lrintf(iq[2 * i + 1] * g);
    }
}

static void *drain_thread(void *p)
{
    rru_t *r = p;
    const rru_hw_ops_t *hw = r->cfg.hw;
    size_t per = r->ul_period, ramp = r->ramp_len, ramp_left = 0;
    int16_t *buf = calloc((per + ramp) * 2, sizeof(int16_t));
    if (!buf)
        return NULL;

    /* Key once enough is buffered. Samples arrive one master message at a
     * time, so the buffer has to bridge the gap to the next message: the
     * automatic prefill is the largest message seen + one drain period + the
     * ramp reserve + a jitter margin. A master sending in real time therefore
     * keys after about one message; a master that is ahead keys at once. */
    size_t need = 0;
    while (LD(r->ul_run)) {
        size_t msg = LD(r->ul_msg_max);
        if (r->ul_prefill)
            need = r->ul_prefill;
        else if (msg)
            need = msg + per + ramp + r->ul_jitter;
        else
            need = SIZE_MAX;            /* nothing received yet */
        if (need != SIZE_MAX && need > r->ring.cap - per)
            need = r->ring.cap - per;   /* message larger than the ring */
        if (need != SIZE_MAX && ring_wait_level(&r->ring, need, 1))
            break;
        if (LD(r->ul_end))
            break;                      /* short transmission: play what there is */
        if (need == SIZE_MAX) {
            struct timespec ts = { 0, 1000000 };
            nanosleep(&ts, NULL);
        }
    }
    if (LD(r->ul_run) && LD(r->ul_end) && ring_level(&r->ring) == 0) {
        /* end marker without any samples: nothing to transmit */
        ADD(r->st.ul_ends, 1);
        cari_slave_ul_finished(r->slave, SUB);
        free(buf);
        return NULL;
    }
    if (LD(r->ul_run)) {
        key(r, true);
        ramp_left = ramp;
        int64_t delay = (mono_ns() - LD(r->ul_first_ns)) / 1000;
        ST(r->st.key_delay_us, (uint64_t)delay);
        cari_log(CARI_LOG_INFO, "PA keyed %.1f ms after the first sample (%zu buffered, messages of %zu)",
                 delay / 1000.0, ring_level(&r->ring), LD(r->ul_msg_max));
    }

    while (LD(r->ul_run)) {
        /* A period is only played while a ramp's worth of samples stays
         * behind it. Otherwise the buffer is about to run dry: ramp
         * everything that is left down to zero, play it, unkey and trip
         * (spec: "UL underrun handling"). */
        if (ring_level(&r->ring) < per + ramp && LD(r->ul_end)) {
            /* End of transmission: play the rest, ramping only its tail
             * down, unkey, and let the library return to idle. */
            size_t got = ring_read(&r->ring, buf, per + ramp);
            apply_keyon_ramp(buf, got, &ramp_left, ramp);
            size_t tail = got < ramp ? got : ramp;
            ramp_down(buf + 2 * (got - tail), tail);
            if (got)
                hw->tx_write(r->hw, buf, got);
            key(r, false);
            ADD(r->st.ul_consumed, (uint64_t)got);
            ADD(r->st.ul_ends, 1);
            cari_log(CARI_LOG_INFO, "end of transmission: all samples played, PA unkeyed");
            cari_slave_ul_finished(r->slave, SUB);
            free(buf);
            return NULL;
        }
        if (ring_level(&r->ring) < per + ramp) {
            size_t got = ring_read(&r->ring, buf, per + ramp);
            ramp_down(buf, got);
            if (got)
                hw->tx_write(r->hw, buf, got);
            key(r, false);
            ST(r->tripped, 1);
            ADD(r->st.ul_underruns, 1);
            ADD(r->st.ul_consumed, (uint64_t)got);
            cari_log(CARI_LOG_WARN, "UL underrun: last %zu samples ramped down, PA unkeyed "
                     "(last UL message %.1f ms ago)", got, (mono_ns() - LD(r->ul_last_ns)) / 1e6);
            cari_slave_ul_underrun(r->slave, SUB);
            free(buf);
            return NULL;
        }
        ring_read(&r->ring, buf, per);
        apply_keyon_ramp(buf, per, &ramp_left, ramp);
        if (hw->tx_write(r->hw, buf, per) < 0) {
            cari_log(CARI_LOG_ERROR, "tx_write failed");
            break;
        }
        ADD(r->st.ul_consumed, (uint64_t)per);
    }

    /* Stopped by the master: ramp down over the next few samples. */
    if (LD(r->keyed)) {
        size_t got = ring_read(&r->ring, buf, ramp);
        if (got) {
            ramp_down(buf, got);
            hw->tx_write(r->hw, buf, got);
        }
        key(r, false);
        cari_log(CARI_LOG_INFO, "PA unkeyed");
    }
    free(buf);
    return NULL;
}

static int ul_start(rru_t *r)
{
    uint32_t rate = (uint32_t)r->rates[r->rate_idx];
    r->ul_period = (size_t)((double)rate * r->cfg.ul_period_us / 1e6);
    if (r->ul_period < 16)
        r->ul_period = 16;
    r->ramp_len = (size_t)((double)rate * r->cfg.ramp_us / 1e6);
    if (r->ramp_len < 1)
        r->ramp_len = 1;
    r->ul_jitter = rate * r->cfg.ul_jitter_ms / 1000;
    r->ul_prefill = rate * r->cfg.ul_prefill_ms / 1000;     /* 0 = automatic */
    if (r->ul_prefill && r->ul_prefill < r->ul_period + r->ramp_len)
        r->ul_prefill = r->ul_period + r->ramp_len;
    size_t cap = rate * r->cfg.ul_buffer_ms / 1000;
    size_t min_cap = (r->ul_prefill ? r->ul_prefill : r->ul_period + r->ramp_len + r->ul_jitter) + 2 * r->ul_period;
    if (cap < min_cap)
        cap = min_cap;
    ST(r->ul_msg_max, 0);
    ST(r->ul_first_ns, 0);
    if (r->ring.cap != cap) {
        if (r->ring.buf)
            ring_free(&r->ring);
        if (ring_init(&r->ring, cap) < 0)
            return CARI_S_INVALID_STATE;
    }
    ring_reset(&r->ring);

    sx1255_set_mode(&r->sx, SX1255_MODE_TX, SX1255_MODE_TX);
    if (!sx1255_wait_lock(&r->sx, false, true, 2000))
        cari_log(CARI_LOG_WARN, "TX PLL not locked");
    if (r->cfg.hw->tx_open(r->hw, rate) < 0) {
        sx1255_set_mode(&r->sx, SX1255_MODE_TX, 0);
        return CARI_S_INVALID_STATE;
    }
    ST(r->tripped, 0);
    ST(r->keyed, 0);
    ST(r->ul_end, 0);
    ST(r->ul_run, 1);
    pthread_create(&r->feed_th, NULL, feed_thread, r);
    pthread_create(&r->drain_th, NULL, drain_thread, r);
    r->ul_active = true;
    return CARI_OK;
}

static void ul_stop(rru_t *r)
{
    if (!r->ul_active)
        return;
    ST(r->ul_run, 0);
    /* Do not close the ring yet: the drain may still ramp down from it. The
     * feeder and drain wake up within their 20 ms timeouts. */
    pthread_join(r->drain_th, NULL);
    ring_close(&r->ring);
    pthread_join(r->feed_th, NULL);
    r->cfg.hw->tx_close(r->hw);
    if (LD(r->keyed))
        key(r, false);
    sx1255_set_mode(&r->sx, SX1255_MODE_TX | SX1255_MODE_PA, 0);
    r->ul_active = false;
}

/* ---------------------------------------------------------------- CARI ops */

static int op_set_param(void *u, uint8_t sub, uint8_t p, cari_value_t *v)
{
    rru_t *r = u;
    (void)sub;
    switch (p) {
    case CARI_P_RX_FREQ:
        r->rx_freq = v->v.u64;
        sx1255_set_freq(&r->sx, false, r->rx_freq, r->ppm);
        return CARI_OK;
    case CARI_P_TX_FREQ:
        r->tx_freq = v->v.u64;
        sx1255_set_freq(&r->sx, true, r->tx_freq, r->ppm);
        return CARI_OK;
    case CARI_P_LNA_GAIN: {
        int code = sx1255_lna_code(v->v.f32);
        if (code < 0)
            return CARI_S_OUT_OF_RANGE;
        r->lna_code = code;
        sx1255_set_rx_gain(&r->sx, code, r->cfg.pga_code);
        return CARI_OK;
    }
    case CARI_P_TX_POWER:
        sx1255_tx_gain_for(v->v.f32, r->cfg.hw_cfg.pmax_dbm, &r->txg);
        sx1255_set_tx_gain(&r->sx, r->txg.dac, r->txg.mixer);
        v->v.f32 = r->txg.dbm;          /* read back what is really set */
        return CARI_OK;
    case CARI_P_SAMPLE_RATE: {
        int i = rate_index(r, v->v.f32);
        if (i < 0)
            return CARI_S_OUT_OF_RANGE;
        r->rate_idx = i;
        sx1255_set_rate(&r->sx, &r->rate_cfg[i]);
        return CARI_OK;
    }
    case CARI_P_FREQ_CORR:
        /* crystal correction: +-100 ppm covers any sane XO */
        if (fabsf(v->v.f32) > 100.0f)
            return CARI_S_OUT_OF_RANGE;
        r->ppm = v->v.f32;
        sx1255_set_freq(&r->sx, false, r->rx_freq, r->ppm);
        sx1255_set_freq(&r->sx, true, r->tx_freq, r->ppm);
        return CARI_OK;
    case CARI_P_FORMAT:
    case CARI_P_MODE:
    case CARI_P_COMPRESSION:
        return CARI_OK;                 /* validated by libcari; nothing to program */
    default:
        return CARI_S_UNSUPPORTED;
    }
}

static int op_stream_start(void *u, uint8_t sub, cari_dir_t dir)
{
    (void)sub;
    return dir == CARI_DIR_DL ? dl_start(u) : ul_start(u);
}

static void op_stream_stop(void *u, uint8_t sub, cari_dir_t dir)
{
    (void)sub;
    if (dir == CARI_DIR_DL)
        dl_stop(u);
    else
        ul_stop(u);
}

static int op_sub_reset(void *u, uint8_t sub)
{
    rru_t *r = u;
    (void)sub;
    /* Chip reset; the configuration is kept and reprogrammed. */
    if (sx1255_init(&r->sx) < 0)
        cari_log(CARI_LOG_ERROR, "SX1255 not responding after reset");
    program_all(r);
    return CARI_OK;
}

static void op_dev_reset(void *u)
{
    rru_t *r = u;
    if (sx1255_init(&r->sx) < 0)
        cari_log(CARI_LOG_ERROR, "SX1255 not responding after reset");
    apply_defaults(r);
}

static int op_reg_read(void *u, uint8_t reg, uint8_t *val)
{
    rru_t *r = u;
    if (reg == REG_BACKEND) {
        *val = strcmp(r->cfg.hw->name, "dummy") ? 1 : 0;
        return CARI_OK;
    }
    if (reg >= MIRROR_BASE && reg < MIRROR_BASE + SX1255_NREGS)
        return sx1255_read(&r->sx, reg - MIRROR_BASE, val) ? CARI_S_INVALID_ADDR : CARI_OK;
    return CARI_S_INVALID_ADDR;
}

static int op_reg_write(void *u, uint8_t reg, uint8_t val)
{
    (void)u; (void)val;
    if (reg == REG_BACKEND || (reg >= MIRROR_BASE && reg < MIRROR_BASE + SX1255_NREGS))
        return CARI_S_READ_ONLY;
    return CARI_S_INVALID_ADDR;
}

static uint32_t op_error_flags(void *u)
{
    rru_t *r = u;
    uint32_t f = 0;
    if (r->sx.comm_error)
        f |= CARI_EF_SUBDEV_COMM;
    if ((r->dl_active || r->ul_active) && !sx1255_locked(&r->sx, r->dl_active, r->ul_active))
        f |= CARI_EF_PLL_LOCK;
    float t;
    if (r->cfg.hw->sensor(r->hw, CARI_Q_TEMPERATURE, &t) == 0 && (t > 85.0f || t < -40.0f))
        f |= CARI_EF_TEMPERATURE;   /* SX1255 operating range */
    return f;
}

static int op_spvn_read(void *u, uint8_t q, uint8_t sub, float *val)
{
    rru_t *r = u;
    bool per_sub = cari_quantity_info(q)->per_subdevice;
    if (per_sub && sub != SUB)
        return CARI_S_UNSUPPORTED;
    return r->cfg.hw->sensor(r->hw, q, val) == 0 ? CARI_OK : CARI_S_UNSUPPORTED;
}

static const cari_slave_ops_t ops = {
    .set_param = op_set_param,
    .stream_start = op_stream_start,
    .stream_stop = op_stream_stop,
    .sub_reset = op_sub_reset,
    .dev_reset = op_dev_reset,
    .reg_read = op_reg_read,
    .reg_write = op_reg_write,
    .error_flags = op_error_flags,
    .spvn_read = op_spvn_read,
};

/* ---------------------------------------------------------------- setup */

static void build_caps(rru_t *r)
{
    cari_tlv_writer_t w;
    float pmax = r->cfg.hw_cfg.pmax_dbm;

    cari_tlv_init(&w, r->caps, sizeof r->caps);
    cari_tlv_flag(&w, CARI_SCAP_RESET);
    cari_tlv_flag(&w, CARI_SCAP_IQ);
    cari_tlv_flag(&w, CARI_SCAP_RX);
    cari_tlv_flag(&w, CARI_SCAP_TX);
    cari_tlv_flag(&w, CARI_SCAP_FULL_DUPLEX);
    cari_tlv_flag(&w, CARI_SCAP_FMT_CS16);
    cari_tlv_range_u64(&w, CARI_SCAP_RX_FREQ, SX1255_FREQ_MIN, SX1255_FREQ_MAX);
    cari_tlv_range_u64(&w, CARI_SCAP_TX_FREQ, SX1255_FREQ_MIN, SX1255_FREQ_MAX);
    for (int i = 0; i < 6; i++)                 /* LNA steps, discrete */
        cari_tlv_value_f32(&w, CARI_SCAP_LNA_GAIN, sx1255_lna_steps[i]);
    /* TX power: 1 dB steps over 39 dB below pmax (all integer offsets are
     * reachable except -1 and -38 dB, which round to a neighbour) */
    cari_tlv_range_f32(&w, CARI_SCAP_TX_POWER, sx1255_tx_min_dbm(pmax), pmax);
    for (int i = 0; i < r->nrates; i++)         /* discrete rates */
        cari_tlv_value_f32(&w, CARI_SCAP_SAMPLE_RATE, r->rates[i]);
    r->caps_len = w.len;

    cari_tlv_init(&w, r->devcaps, sizeof r->devcaps);
#ifdef CARI_HAVE_FLAC
    cari_tlv_flag(&w, CARI_DCAP_COMPRESSION);
#endif
    cari_tlv_flag(&w, CARI_DCAP_SUPERVISION);
    r->devcaps_len = w.len;
}

rru_t *rru_new(const rru_cfg_t *cfg)
{
    rru_t *r = calloc(1, sizeof *r);
    if (!r)
        return NULL;
    r->cfg = *cfg;

    /* Sample rates this crystal can produce with 16-bit I2S slots */
    for (size_t i = 0; i < sizeof candidate_rates / sizeof *candidate_rates; i++) {
        if (sx1255_rate_config(cfg->hw_cfg.xtal_hz, candidate_rates[i], SX1255_SLOT_BITS, &r->rate_cfg[r->nrates]) == 0)
            r->rates[r->nrates++] = candidate_rates[i];
        else
            cari_log(CARI_LOG_WARN, "%.0f Hz not available with a %.3f MHz crystal (I2S mode B2, 32-bit slots)",
                     candidate_rates[i], cfg->hw_cfg.xtal_hz / 1e6);
    }
    if (!r->nrates) {
        cari_log(CARI_LOG_ERROR, "no usable sample rate");
        goto fail;
    }

    r->hw = cfg->hw->open(&cfg->hw_cfg);
    if (!r->hw) {
        cari_log(CARI_LOG_ERROR, "hardware backend '%s' failed to open", cfg->hw->name);
        goto fail;
    }
    sx1255_attach(&r->sx, cfg->hw, r->hw, cfg->hw_cfg.xtal_hz);
    if (sx1255_init(&r->sx) < 0)
        cari_log(CARI_LOG_ERROR, "SX1255 not found (version register mismatch or SPI error)");

    build_caps(r);
    cari_subdev_desc_t sd = { r->caps, r->caps_len };
    cari_slave_desc_t d = {
        .ident = cfg->ident,
        .dev_caps = r->devcaps, .dev_caps_len = r->devcaps_len,
        .quantities = quantities, .nquantities = sizeof quantities,
        .nsubdevs = 1, .subdevs = &sd,
        .ctrl_endpoint = cfg->ctrl_endpoint,
        .bind_host = cfg->bind_host,
        .ul_rcvhwm = 2,
        .ul_rcvbuf = cfg->ul_rcvbuf,    /* keep the UL queue short (latency) */
        .dl_sndhwm = 16,
        .flac_blocksize = 0,            /* one FLAC frame per DL period (10 ms) */
    };
    r->slave = cari_slave_new(&d, &ops, r);
    if (!r->slave)
        goto fail;
    apply_defaults(r);
    if (cari_slave_start(r->slave) < 0)
        goto fail;
    return r;
fail:
    rru_free(r);
    return NULL;
}

int rru_run(rru_t *r, volatile int *stop)
{
    return cari_slave_run(r->slave, stop);
}

void rru_get_stats(rru_t *r, rru_stats_t *st)
{
    st->ul_underruns = LD(r->st.ul_underruns);
    st->ul_consumed = LD(r->st.ul_consumed);
    st->dl_produced = LD(r->st.dl_produced);
    st->key_delay_us = LD(r->st.key_delay_us);
    st->ul_ends = LD(r->st.ul_ends);
}

void rru_free(rru_t *r)
{
    if (!r)
        return;
    cari_slave_free(r->slave);          /* stops streams through the callbacks */
    if (r->hw) {
        sx1255_set_mode(&r->sx, 0xFF, 0);
        sx1255_detach(&r->sx);
        r->cfg.hw->close(r->hw);
    }
    if (r->ring.buf)
        ring_free(&r->ring);
    free(r);
}
