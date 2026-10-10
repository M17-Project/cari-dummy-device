/*
 * End-to-end test: spawns cari-dummy-device and drives it as a CARI master
 * with libcari. Checks the advertised SX1255 profile, parameter handling,
 * real-time behaviour (DL produced at the sample rate, UL consumed at the
 * sample rate with backpressure), the underrun trip and its recovery,
 * compression in both directions, supervision values and device reset.
 *
 * usage: test_e2e /path/to/cari-dummy-device
 */
#include "t.h"
#include "sx1255.h"

#include <cari/cari_config.h>
#include <cari/cari_flac.h>
#include <cari/cari_log.h>
#include <cari/cari_master.h>

#include <pthread.h>
#include <signal.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <zmq.h>

static cari_master_t *M;
static pid_t child;
static int base;
#define P(n) ((uint16_t)(base + (n)))
#define TOL 0.03                    /* sample-rate tolerance */

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void sleep_s(double s)
{
    struct timespec ts = { (time_t)s, (long)((s - (time_t)s) * 1e9) };
    nanosleep(&ts, NULL);
}

static int setf(uint8_t p, float v)    { cari_value_t x = cari_value_f32(v); return cari_m_set_param(M, 0, p, &x); }
static int setu(uint8_t p, uint64_t v) { cari_value_t x = cari_value_u64(v); return cari_m_set_param(M, 0, p, &x); }
static int set8(uint8_t p, uint8_t v)
{
    cari_value_t x = { .type = cari_param_info(p)->type };
    x.v.u8 = v;
    return cari_m_set_param(M, 0, p, &x);
}
static cari_value_t get(uint8_t p)
{
    cari_value_t v = { 0 };
    CHECK_EQ(cari_m_get_param(M, 0, p, &v), CARI_OK);
    return v;
}
static uint32_t flags(void)
{
    uint32_t f = 0xFFFFFFFF;
    CHECK_EQ(cari_m_ping(M, &f), CARI_OK);
    return f;
}

/* ---------------------------------------------------------------- tests */

static void test_profile(void)
{
    char ident[256];
    uint8_t v;
    CHECK_EQ(cari_m_get_ident(M, ident, sizeof ident), CARI_OK);
    CHECK(strstr(ident, "SX1255") != NULL);
    CHECK_EQ(cari_m_get_reg(M, 0, &v), CARI_OK);
    CHECK_EQ(v, 0x21);
    CHECK_EQ(cari_m_get_reg(M, 1, &v), CARI_OK);
    CHECK_EQ(v, 1);
    CHECK_EQ(cari_m_get_reg(M, 0x02, &v), CARI_OK);           /* backend: dummy */
    CHECK_EQ(v, 0);
    CHECK_EQ(flags(), 0);

    cari_caps_t c;
    CHECK_EQ(cari_m_get_dev_caps(M, &c), CARI_OK);
    CHECK(cari_caps_has_flag(&c, CARI_DCAP_SUPERVISION));
#ifdef CARI_HAVE_FLAC
    CHECK(cari_caps_has_flag(&c, CARI_DCAP_COMPRESSION));
#endif
    uint8_t q[16];
    size_t nq = 0;
    CHECK_EQ(cari_m_get_spvn_list(M, q, sizeof q, &nq), CARI_OK);
    CHECK_EQ(nq, 6);

    CHECK_EQ(cari_m_get_sub_caps(M, 0, &c), CARI_OK);
    const uint8_t want[] = { CARI_SCAP_RESET, CARI_SCAP_IQ, CARI_SCAP_RX, CARI_SCAP_TX,
                             CARI_SCAP_FULL_DUPLEX, CARI_SCAP_FMT_CS16 };
    for (size_t i = 0; i < sizeof want; i++)
        CHECK(cari_caps_has_flag(&c, want[i]));
    const uint8_t not_want[] = { CARI_SCAP_AGC, CARI_SCAP_AFC, CARI_SCAP_FM_MOD, CARI_SCAP_AM_DEMOD,
                                 CARI_SCAP_DC_CORR, CARI_SCAP_FMT_CF32, CARI_SCAP_FMT_S16 };
    for (size_t i = 0; i < sizeof not_want; i++)
        CHECK(!cari_caps_has_flag(&c, not_want[i]));
    int n_lna = 0, n_rate = 0, n_rx = 0, n_tx = 0, n_pwr = 0;
    for (size_t i = 0; i < c.nranges; i++) {
        const cari_range_t *r = &c.ranges[i];
        switch (r->id) {
        case CARI_SCAP_RX_FREQ:
            n_rx++;
            CHECK(r->lo.v.u64 == 400000000ull && r->hi.v.u64 == 510000000ull);
            break;
        case CARI_SCAP_TX_FREQ:
            n_tx++;
            CHECK(r->lo.v.u64 == 400000000ull && r->hi.v.u64 == 510000000ull);
            break;
        case CARI_SCAP_LNA_GAIN:
            n_lna++;
            CHECK(r->lo.v.f32 == r->hi.v.f32);              /* discrete steps */
            CHECK(sx1255_lna_code(r->lo.v.f32) > 0);
            break;
        case CARI_SCAP_TX_POWER:
            n_pwr++;
            CHECK(r->lo.v.f32 == -34.0f && r->hi.v.f32 == 5.0f);
            break;
        case CARI_SCAP_SAMPLE_RATE:
            CHECK(r->lo.v.f32 == r->hi.v.f32);              /* equal-bound pairs */
            CHECK(r->lo.v.f32 == 125000.0f * (float)(1 << n_rate));
            n_rate++;
            break;
        default:
            CHECK(!"unexpected range");
        }
    }
    CHECK(n_rx == 1 && n_tx == 1 && n_pwr == 1);
    CHECK_EQ(n_lna, 6);
    CHECK_EQ(n_rate, 3);
}

static void test_params(void)
{
    CHECK_EQ(setf(CARI_P_LNA_GAIN, 36), CARI_OK);
    CHECK(get(CARI_P_LNA_GAIN).v.f32 == 36.0f);
    CHECK_EQ(setf(CARI_P_LNA_GAIN, 40), CARI_S_OUT_OF_RANGE);
    CHECK_EQ(setf(CARI_P_LNA_GAIN, 48), CARI_OK);
    CHECK_EQ(setf(CARI_P_TX_POWER, 4), CARI_OK);
    CHECK(get(CARI_P_TX_POWER).v.f32 == 3.0f);              /* quantized to the SX1255 steps */
    CHECK_EQ(setf(CARI_P_TX_POWER, -10.3f), CARI_OK);
    CHECK(get(CARI_P_TX_POWER).v.f32 == -10.0f);
    CHECK_EQ(setf(CARI_P_TX_POWER, 7), CARI_S_OUT_OF_RANGE);
    CHECK_EQ(setu(CARI_P_RX_FREQ, 520000000), CARI_S_OUT_OF_RANGE);
    CHECK_EQ(setu(CARI_P_TX_FREQ, 399000000), CARI_S_OUT_OF_RANGE);
    CHECK_EQ(setf(CARI_P_SAMPLE_RATE, 200000), CARI_S_OUT_OF_RANGE);
    CHECK_EQ(setf(CARI_P_SAMPLE_RATE, 500000), CARI_OK);
    CHECK_EQ(set8(CARI_P_FORMAT, CARI_FMT_CF32), CARI_S_OUT_OF_RANGE);
    CHECK_EQ(set8(CARI_P_MODE, CARI_MODE_FM), CARI_S_OUT_OF_RANGE);
    CHECK_EQ(set8(CARI_P_AGC, 1), CARI_S_UNSUPPORTED);
    CHECK_EQ(setf(CARI_P_CH_WIDTH, 25000), CARI_S_UNSUPPORTED);
    CHECK_EQ(setf(CARI_P_DEVIATION, 2500), CARI_S_UNSUPPORTED);
    CHECK_EQ(setf(CARI_P_FREQ_CORR, 150), CARI_S_OUT_OF_RANGE);
    CHECK_EQ(set8(CARI_P_TX_STATE, 0), CARI_S_READ_ONLY);
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_IDLE);

    /* the SX1255 registers (mirrored at 0x80..0x93) follow the parameters */
    CHECK_EQ(setf(CARI_P_FREQ_CORR, 2.5f), CARI_OK);
    CHECK_EQ(setu(CARI_P_RX_FREQ, 433475000), CARI_OK);
    CHECK_EQ(setu(CARI_P_TX_FREQ, 439975000), CARI_OK);
    uint8_t r[SX1255_NREGS];
    for (int i = 0; i < SX1255_NREGS; i++)
        CHECK_EQ(cari_m_get_reg(M, (uint8_t)(0x80 + i), &r[i]), CARI_OK);
    CHECK_EQ(r[SX1255_VERSION], SX1255_CHIP_VERSION);
    CHECK_EQ(r[SX1255_MODE], SX1255_MODE_REF);
    uint32_t wrx = (uint32_t)r[1] << 16 | r[2] << 8 | r[3], wtx = (uint32_t)r[4] << 16 | r[5] << 8 | r[6];
    CHECK_EQ(wrx, sx1255_frf_word(433475000, 32e6 * (1 + 2.5e-6)));
    CHECK_EQ(wtx, sx1255_frf_word(439975000, 32e6 * (1 + 2.5e-6)));
    CHECK_EQ(r[SX1255_TXFE1], (2 << 4) | 9);                  /* -10 dBm = 5 - 15: DAC -3 dB, mixer -19.5 dB */
    CHECK_EQ(r[SX1255_RXFE1] >> 5, 1);                        /* LNA G1 */
    CHECK_EQ(r[SX1255_DIG_BRIDGE] >> 3 & 7, 3);               /* 500 kHz: 32 MHz / 64 */
    CHECK_EQ(r[SX1255_IISM], 0x20);                           /* B2, CLK_OUT = 32 MHz (32-bit slots) */
    CHECK_EQ(cari_m_set_reg(M, 0x80, 0), CARI_S_READ_ONLY);
    CHECK_EQ(cari_m_get_reg(M, 0x40, &r[0]), CARI_S_INVALID_ADDR);
    CHECK_EQ(setf(CARI_P_FREQ_CORR, 0), CARI_OK);
}

/* Count DL samples over a window; returns measured rate. */
static void drain(void *sock)
{
    uint8_t b[256];
    zmq_pollitem_t it = { sock, 0, ZMQ_POLLIN, 0 };
    while (zmq_poll(&it, 1, 0) > 0)
        zmq_recv(sock, b, sizeof b, 0);
}

static double measure_dl(void *sub, bool compressed, double warm, double window, size_t *wire)
{
    uint8_t *b = malloc(1 << 20);
    double t_end = now_s() + warm + window, t_start = now_s() + warm;
    size_t samples = 0, bytes = 0;
    bool counting = false;
    double t0 = 0, t1 = 0;
#ifdef CARI_HAVE_FLAC
    cari_flac_dec_t *d = compressed ? cari_flac_dec_new(CARI_FMT_CS16, 0) : NULL;
#else
    (void)compressed;
#endif
    while (now_s() < t_end) {
        zmq_pollitem_t it = { sub, 0, ZMQ_POLLIN, 0 };
        if (zmq_poll(&it, 1, 50) <= 0)
            continue;
        int n = zmq_recv(sub, b, 1 << 20, 0);
        if (n <= 0)
            continue;
        double t = now_s();
        size_t ns = (size_t)n / 4;
#ifdef CARI_HAVE_FLAC
        if (d) {
            CHECK(cari_flac_dec_feed(d, b, (size_t)n) >= 1);
            size_t dl;
            cari_flac_dec_take(d, &dl);
            ns = dl / 4;
        } else
#endif
            CHECK(n % 4 == 0);
        if (!counting && t >= t_start) {
            counting = true;
            t0 = t;                 /* count what arrives after this message */
            continue;
        }
        if (counting) {
            samples += ns;
            bytes += (size_t)n;
            t1 = t;
        }
    }
#ifdef CARI_HAVE_FLAC
    cari_flac_dec_free(d);
#endif
    free(b);
    if (wire)
        *wire = bytes;
    return t1 > t0 ? samples / (t1 - t0) : 0;
}

/* A rate measurement over 1 s; measured again once if a VM stall skewed it
 * (shared VMs delay threads by tens of milliseconds now and then). */
static double measure_dl_rate(void *sub, bool compressed, double warm, double expect, size_t *wire)
{
    double r = measure_dl(sub, compressed, warm, 1.0, wire);
    if (fabs(r / expect - 1.0) >= TOL) {
        fprintf(stderr, "   (%.0f samples/s, measuring again)\n", r);
        if (wire)
            *wire = 0;
        r = measure_dl(sub, compressed, 0.0, 1.0, wire);
    }
    return r;
}

static void test_rx_timing(void)
{
    const float rates[] = { 125000, 250000, 500000 };
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_START), CARI_S_INVALID_STATE);  /* not bound */
    CHECK_EQ(cari_m_bind_dl(M, 0, P(1)), CARI_OK);
    void *sub = cari_master_connect_dl(M, P(1), 64);
    for (int i = 0; i < 3; i++) {
        CHECK_EQ(setf(CARI_P_SAMPLE_RATE, rates[i]), CARI_OK);
        drain(sub);
        CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_START), CARI_OK);
        CHECK_EQ(setf(CARI_P_SAMPLE_RATE, rates[i]), CARI_S_INVALID_STATE);
        CHECK_EQ(setu(CARI_P_RX_FREQ, 434000000), CARI_OK);           /* retune while running */
        double r = measure_dl_rate(sub, false, 0.2, rates[i], NULL);
        fprintf(stderr, "   DL %6.0f Hz: measured %8.0f samples/s\n", rates[i], r);
        CHECK(fabs(r / rates[i] - 1.0) < TOL);
        CHECK_EQ(flags(), 0);
        CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_STOP), CARI_OK);
    }
#ifdef CARI_HAVE_FLAC
    CHECK_EQ(set8(CARI_P_COMPRESSION, 1), CARI_OK);
    sleep_s(0.05);
    drain(sub);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_START), CARI_OK);
    size_t wire = 0;
    double r = measure_dl_rate(sub, true, 0.2, 500000, &wire);
    fprintf(stderr, "   DL 500000 Hz FLAC: measured %8.0f samples/s, %.0f%% of raw size\n", r,
            100.0 * wire / (r * 4.0));
    CHECK(fabs(r / 500000 - 1.0) < TOL);
    CHECK(wire < r * 4.0);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_STOP), CARI_OK);
    CHECK_EQ(set8(CARI_P_COMPRESSION, 0), CARI_OK);
#endif
    zmq_close(sub);
    CHECK_EQ(cari_m_bind_dl(M, 0, 0), CARI_OK);
}

/* Send `secs` of a tone with amplitude amp as fast as the slave accepts it.
 * Returns the elapsed send time. */
static int stop_push;

static double push_tone(void *push, uint32_t rate, double secs, float amp, bool flac)
{
    int to = 3000;      /* never hang the test if the slave stops reading */
    zmq_setsockopt(push, ZMQ_SNDTIMEO, &to, sizeof to);
    size_t per = rate / 100, total = (size_t)(rate * secs);
    int16_t *iq = malloc(per * 4);
    double ph = 0;
    double t0 = now_s();
#ifdef CARI_HAVE_FLAC
    cari_flac_enc_t *e = flac ? cari_flac_enc_new(CARI_FMT_CS16, rate, (uint32_t)per, 5) : NULL;
#else
    (void)flac;
#endif
    for (size_t done = 0; done < total && !__atomic_load_n(&stop_push, __ATOMIC_SEQ_CST); done += per) {
        for (size_t i = 0; i < per; i++, ph += 0.05) {
            iq[2 * i] = (int16_t)lrint(amp * 32767 * cos(ph));
            iq[2 * i + 1] = (int16_t)lrint(amp * 32767 * sin(ph));
        }
#ifdef CARI_HAVE_FLAC
        if (e) {
            cari_flac_enc_push(e, iq, per);
            if (done + per >= total)
                cari_flac_enc_finish(e);
            size_t len;
            const uint8_t *m = cari_flac_enc_take(e, &len);
            if (m && zmq_send(push, m, len, 0) < 0)
                break;                  /* slave stopped reading: give up */
            continue;
        }
#endif
        if (zmq_send(push, iq, per * 4, 0) < 0)     /* blocks when the slave is full */
            break;                                  /* slave stopped reading (tripped) */
    }
#ifdef CARI_HAVE_FLAC
    cari_flac_enc_free(e);
#endif
    free(iq);
    return now_s() - t0;
}

/* Poll until the trip flag shows up; returns the time. */
static double wait_trip(double timeout)
{
    double end = now_s() + timeout;
    while (now_s() < end) {
        uint32_t f = 0;
        if (cari_m_ping(M, &f) == CARI_OK && (f & CARI_EF_UL_TRIP))
            return now_s();
        sleep_s(0.005);
    }
    return 0;
}

static void tx_run(uint32_t rate, double secs, bool flac)
{
    CHECK_EQ(setf(CARI_P_SAMPLE_RATE, (float)rate), CARI_OK);
#ifdef CARI_HAVE_FLAC
    CHECK_EQ(set8(CARI_P_COMPRESSION, flac), CARI_OK);
#endif
    void *push = cari_master_connect_ul(M, P(2), 2, 16 * 1024);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_START), CARI_OK);
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_TRANSMITTING);
    double t0 = now_s();
    double sent = push_tone(push, rate, secs, 0.5f, flac);
    /* still transmitting right after the last send: no premature trip */
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_TRANSMITTING);
    double t_trip = wait_trip(3.0);
    CHECK(t_trip > 0);
    double air = t_trip - t0;
    fprintf(stderr, "   UL %6u Hz%s: %.1f s of samples sent in %.2f s, trip after %.2f s\n", rate,
            flac ? " FLAC" : "", secs, sent, air);
    /* the slave consumed at the sample rate: the trip happens once all the
     * samples have been played out, i.e. after ~secs (plus the 20 ms prefill) */
    CHECK(air > secs - 0.05 && air < secs + 0.35);
    /* backpressure: the master could not send much faster than real time
     * (allowance for the slave's 60 ms ring, ZMQ queues and the TCP socket
     * buffers - 16 KiB send, 32 KiB receive, doubled by Linux).
     * Not checked for FLAC: socket buffers count bytes, and a compressed
     * tone is so small that the whole second fits in them. */
    if (!flac)
        CHECK(sent > secs - 0.6);
    zmq_close(push);

    /* tripped: flag set, state 2, start refused, stop keeps the trip */
    CHECK(flags() & CARI_EF_UL_TRIP);
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_TRIPPED);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_START), CARI_S_INVALID_STATE);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_STOP), CARI_OK);
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_TRIPPED);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_CLEAR_TRIP), CARI_OK);
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_IDLE);
    CHECK_EQ(flags(), 0);
}

/* CARI 2.1: end the stream with the zero-byte marker; the slave plays
 * every sample and returns to idle without a trip. */
static void tx_end_run(uint32_t rate, double secs, bool flac)
{
    bool v21 = false;
    CHECK_EQ(cari_m_supports(M, 2, 1, &v21), CARI_OK);
    CHECK(v21);
    CHECK_EQ(setf(CARI_P_SAMPLE_RATE, (float)rate), CARI_OK);
#ifdef CARI_HAVE_FLAC
    CHECK_EQ(set8(CARI_P_COMPRESSION, flac), CARI_OK);
#endif
    void *push = cari_master_connect_ul(M, P(2), 2, 16 * 1024);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_START), CARI_OK);
    double t0 = now_s();
    push_tone(push, rate, secs, 0.5f, flac);
    CHECK_EQ(cari_master_ul_send_end(push), 0);
    /* (the marker is still in transit behind ~0.15 s of queued samples, so
     * the slave is not playing out yet - test_slave covers status 8) */
    int rc = cari_m_wait_tx_idle(M, 0, (int)(secs * 1000) + 2000);
    double air = now_s() - t0;
    fprintf(stderr, "   UL %6u Hz%s + end marker: idle after %.2f s (%s)\n", rate, flac ? " FLAC" : "", air,
            rc == CARI_OK ? "no trip" : cari_strerror(rc));
    CHECK_EQ(rc, CARI_OK);
    CHECK(air > secs - 0.05 && air < secs + 0.35);
    CHECK_EQ(flags(), 0);
    zmq_close(push);
}

static void test_tx(void)
{
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_START), CARI_S_INVALID_STATE);  /* not bound */
    CHECK_EQ(cari_m_bind_ul(M, 0, P(2)), CARI_OK);
    CHECK_EQ(setf(CARI_P_TX_POWER, -10), CARI_OK);
    tx_run(250000, 2.0, false);
    tx_run(500000, 1.0, false);
    tx_run(125000, 1.0, false);
#ifdef CARI_HAVE_FLAC
    tx_run(250000, 1.0, true);
    CHECK_EQ(set8(CARI_P_COMPRESSION, 0), CARI_OK);
#endif
    tx_end_run(250000, 1.0, false);
#ifdef CARI_HAVE_FLAC
    tx_end_run(500000, 1.0, true);
    CHECK_EQ(set8(CARI_P_COMPRESSION, 0), CARI_OK);
#endif
    /* an explicit stop does not trip */
    void *push = cari_master_connect_ul(M, P(2), 2, 16 * 1024);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_START), CARI_OK);
    push_tone(push, 125000, 0.3, 0.5f, false);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_STOP), CARI_OK);
    sleep_s(0.4);
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_IDLE);
    CHECK_EQ(flags(), 0);
    zmq_close(push);
}

typedef struct {
    void   *push;
    double  secs;
} pusher_t;

static void *pusher(void *p)
{
    pusher_t *a = p;
    push_tone(a->push, 500000, a->secs, 0.5f, false);
    return NULL;
}

static void test_duplex_spvn(void)
{
    /* full duplex at 500 kHz with supervision running */
    CHECK_EQ(setf(CARI_P_SAMPLE_RATE, 500000), CARI_OK);
    CHECK_EQ(setf(CARI_P_TX_POWER, -10), CARI_OK);
    CHECK_EQ(cari_m_bind_dl(M, 0, P(1)), CARI_OK);
    const uint8_t q[] = { 0, 1, 2, 3, 4, 5 };
    CHECK_EQ(cari_m_bind_spvn(M, P(3), 100, q, 6), CARI_OK);
    void *spvn = cari_master_connect_spvn(M, P(3));
    void *sub = cari_master_connect_dl(M, P(1), 64);
    pusher_t pa = { cari_master_connect_ul(M, P(2), 2, 16 * 1024), 3.0 };
    int to = 100;       /* the pusher must notice the stop even while blocked */
    zmq_setsockopt(pa.push, ZMQ_SNDTIMEO, &to, sizeof to);
    __atomic_store_n(&stop_push, 0, __ATOMIC_SEQ_CST);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_START), CARI_OK);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_START), CARI_OK);
    pthread_t th;
    pthread_create(&th, NULL, pusher, &pa);

    double dl_rate = measure_dl_rate(sub, false, 0.3, 500000, NULL);
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_TRANSMITTING);

    /* supervision while keyed */
    drain(spvn);
    float inc = 0, refl = 0, rl = 0, temp = 0, volt = 0, cur = 0;
    int packets = 0;
    uint8_t b[4096];
    double t0 = now_s();
    while (now_s() - t0 < 0.55) {
        zmq_pollitem_t it = { spvn, 0, ZMQ_POLLIN, 0 };
        if (zmq_poll(&it, 1, 50) <= 0)
            continue;
        int n = zmq_recv(spvn, b, sizeof b, 0);
        cari_spvn_entry_t e[16];
        int ne = cari_spvn_parse(b, (size_t)n, e, 16);
        CHECK_EQ(ne, 6);
        for (int i = 0; i < ne && i < 16; i++) {
            switch (e[i].qid) {
            case 0: temp = e[i].value; break;
            case 1: volt = e[i].value; break;
            case 2: cur = e[i].value; break;
            case 3: rl = e[i].value; CHECK_EQ(e[i].sub, 0); break;
            case 4: inc = e[i].value; CHECK_EQ(e[i].sub, 0); break;
            case 5: refl = e[i].value; CHECK_EQ(e[i].sub, 0); break;
            }
        }
        packets++;
    }
    fprintf(stderr, "   duplex: DL %.0f samples/s while transmitting; %d SPVN packets in 0.55 s\n",
            dl_rate, packets);
    fprintf(stderr, "   SPVN: %.1f degC, %.2f V, %.3f A, RL %.1f dB, Pinc %.1f dBm, Prefl %.1f dBm\n",
            temp, volt, cur, rl, inc, refl);
    CHECK(fabs(dl_rate / 500000 - 1.0) < TOL);
    CHECK(packets >= 4 && packets <= 7);
    /* |x| = 0.5 at -10 dBm set power -> -16 dBm average (Table 19 scaling) */
    CHECK(fabs(inc - (-16.0f)) < 0.5f);
    CHECK(fabs(refl - (inc - rl)) < 0.01f);
    CHECK(temp > 20 && temp < 60 && volt > 4.5f && volt < 5.5f && cur > 0.2f);
    CHECK_EQ(flags(), 0);

    __atomic_store_n(&stop_push, 1, __ATOMIC_SEQ_CST);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_TX_STOP), CARI_OK);
    pthread_join(th, NULL);
    CHECK_EQ(get(CARI_P_TX_STATE).v.u8, CARI_TX_IDLE);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_STOP), CARI_OK);
    CHECK_EQ(cari_m_bind_spvn(M, 0, 0, NULL, 0), CARI_OK);
    zmq_close(spvn);
    zmq_close(sub);
    zmq_close(pa.push);
}

static void test_reset(void)
{
    CHECK_EQ(setu(CARI_P_RX_FREQ, 440000000), CARI_OK);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RESET), CARI_OK);              /* keeps configuration */
    CHECK_EQ(get(CARI_P_RX_FREQ).v.u64, 440000000);
    CHECK_EQ(cari_m_bind_dl(M, 0, P(1)), CARI_OK);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_START), CARI_OK);
    CHECK_EQ(cari_m_reset_device(M), CARI_OK);
    CHECK_EQ(get(CARI_P_RX_FREQ).v.u64, 433475000);                    /* defaults */
    CHECK(get(CARI_P_SAMPLE_RATE).v.f32 == 250000.0f);
    CHECK_EQ(cari_m_exec(M, 0, CARI_ACT_RX_START), CARI_S_INVALID_STATE);  /* sockets closed */
    uint8_t mode;
    CHECK_EQ(cari_m_get_reg(M, 0x80, &mode), CARI_OK);
    CHECK_EQ(mode, SX1255_MODE_REF);
    CHECK_EQ(flags(), 0);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s cari-dummy-device\n", argv[0]);
        return 2;
    }
    /* below the Linux ephemeral range (32768+) so a bind never collides with
     * the local port of an outgoing connection */
    base = 10000 + (getpid() % 1800) * 12;
    char ctrl[16];
    snprintf(ctrl, sizeof ctrl, "%u", P(0));
    child = fork();
    if (child == 0) {
        execl(argv[1], argv[1], "-c", ctrl, "-b", "127.0.0.1", getenv("E2E_VERBOSE") ? "-vv" : "-s1",
              (char *)NULL);
        _exit(127);
    }
    cari_log_set_handler(NULL, NULL, CARI_LOG_ERROR);
    char ep[64];
    snprintf(ep, sizeof ep, "127.0.0.1:%u", P(0));
    M = cari_master_new(NULL, ep, 300);
    cari_master_set_retries(M, 20);         /* wait for the device to come up */
    uint32_t f;
    if (cari_m_ping(M, &f) != CARI_OK) {
        fprintf(stderr, "device did not answer\n");
        kill(child, SIGKILL);
        return 1;
    }
    cari_master_set_retries(M, 0);
    cari_master_set_timeout(M, 2000);

    RUN(test_profile);
    RUN(test_params);
    RUN(test_rx_timing);
    RUN(test_tx);
    RUN(test_duplex_spvn);
    RUN(test_reset);

    cari_master_free(M);
    kill(child, SIGTERM);
    int status = 0;
    double t0 = now_s();
    while (waitpid(child, &status, WNOHANG) == 0 && now_s() - t0 < 3)
        sleep_s(0.01);
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    T_DONE();
}
