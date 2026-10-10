/*
 * UL key-up latency and robustness with masters that send in real time.
 *
 * Runs the RRU core with the dummy backend in-process and plays the master:
 * messages of M ms are sent every M ms (a live modulator), so the slave
 * cannot fill its buffer ahead. Checks that the transmitter keys after
 * about one message (the automatic prefill) and never trips while the
 * master keeps up, for several message sizes, and that a master sending
 * ahead of time keys immediately.
 */
#include "t.h"
#include "rru.h"

#include <cari/cari_log.h>
#include <cari/cari_master.h>

#include <math.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <zmq.h>

static int base;
static rru_t *R;
static int stop;

static void *run(void *p)
{
    (void)p;
    rru_run(R, &stop);
    return NULL;
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void sleep_until(double t)
{
    struct timespec ts = { (time_t)t, (long)((t - (time_t)t) * 1e9) };
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

/* Send `secs` of signal in messages of msg_ms, paced in real time (or as
 * fast as accepted when realtime is false). Returns 1 if the slave tripped. */
static int session(cari_master_t *m, void *push, double msg_ms, double secs, bool realtime)
{
    const uint32_t rate = 250000;
    size_t n = (size_t)(rate * msg_ms / 1000);
    int16_t *iq = calloc(n * 2, sizeof(int16_t));
    for (size_t i = 0; i < n; i++)
        iq[2 * i] = 8000;
    CHECK_EQ(cari_m_exec(m, 0, CARI_ACT_TX_START), CARI_OK);
    double t0 = now_s();
    int msgs = (int)(secs * 1000 / msg_ms);
    for (int k = 0; k < msgs; k++) {
        if (realtime)
            sleep_until(t0 + k * msg_ms / 1000.0);
        if (zmq_send(push, iq, n * 4, 0) < 0)
            break;                  /* slave stopped reading (tripped) */
    }
    if (realtime)
        sleep_until(t0 + msgs * msg_ms / 1000.0 - 0.003);
    uint32_t f = 0;
    cari_m_ping(m, &f);
    int tripped = (f & CARI_EF_UL_TRIP) != 0;
    if (tripped)
        cari_m_exec(m, 0, CARI_ACT_CLEAR_TRIP);
    CHECK_EQ(cari_m_exec(m, 0, CARI_ACT_TX_STOP), CARI_OK);
    free(iq);
    return tripped;
}

int main(void)
{
    /* below the Linux ephemeral range (32768+) so a bind never collides with
     * the local port of an outgoing connection */
    base = 10000 + (getpid() % 400) * 50;
    cari_log_set_handler(NULL, NULL, getenv("LAT_VERBOSE") ? CARI_LOG_INFO : CARI_LOG_ERROR);
    char ep[64], addr[64];
    snprintf(ep, sizeof ep, "tcp://127.0.0.1:%d", base);
    snprintf(addr, sizeof addr, "127.0.0.1:%d", base);

    rru_cfg_t c;
    rru_cfg_defaults(&c);
    c.ctrl_endpoint = ep;
    c.bind_host = "127.0.0.1";
    /* CI machines (VMs, sanitizers) stall threads for up to ~10 ms; the
     * device default (5 ms) is meant for a dedicated RRU */
    c.ul_jitter_ms = 12;
    R = rru_new(&c);
    CHECK(R != NULL);
    if (!R)
        T_DONE();
    pthread_t th;
    pthread_create(&th, NULL, run, NULL);

    cari_master_t *m = cari_master_new(NULL, addr, 2000);
    cari_value_t v = cari_value_f32(250000);
    CHECK_EQ(cari_m_set_param(m, 0, CARI_P_SAMPLE_RATE, &v), CARI_OK);

    /* automatic prefill = message + drain period + ramp + jitter; keying
     * happens when the k-th message arrives, k*M >= prefill, i.e. (k-1)*M
     * after the first sample */
    const double P = c.ul_period_us / 1000.0, Rm = c.ramp_us / 1000.0, J = c.ul_jitter_ms;
    const double sizes[] = { 1, 2, 5, 10, 20 };
    for (size_t i = 0; i < sizeof sizes / sizeof *sizes + 1; i++) {
        bool ahead = i == sizeof sizes / sizeof *sizes;
        double M = ahead ? 10 : sizes[i];
        /* A fresh UL socket per session, so nothing queued carries over.
         * Shared VMs stall threads for tens of ms now and then (measured
         * here: a lone 1 ms timer up to 14 ms late), more than any useful
         * margin; a session that trips is therefore retried (up to 3 tries). */
        int tripped = 1, attempt;
        double d = 0;
        for (attempt = 0; attempt < 3 && tripped; attempt++) {
            uint16_t port = (uint16_t)(base + 1 + 3 * i + attempt);
            CHECK_EQ(cari_m_bind_ul(m, 0, port), CARI_OK);
            void *push = cari_master_connect_ul(m, port, 2, 16 * 1024);
            int to = 300;
            zmq_setsockopt(push, ZMQ_SNDTIMEO, &to, sizeof to);
            tripped = session(m, push, M, ahead ? 0.5 : 1.0, !ahead);
            zmq_close(push);
            rru_stats_t st;
            rru_get_stats(R, &st);
            d = st.key_delay_us / 1000.0;
        }
        double expect = ahead ? 0 : (ceil((M + P + Rm + J) / M - 1e-9) - 1) * M;
        fprintf(stderr, "   %s, %4.0f ms messages: keyed after %5.1f ms (model %4.1f), %s\n",
                ahead ? "master ahead of time" : "real-time master    ", M, d, expect,
                tripped ? "TRIPPED in every attempt" : attempt > 1 ? "no trip (after a retry)" : "no trip");
        CHECK(!tripped);
        CHECK(fabs(d - expect) <= 2.0);
    }
    /* End of transmission: every sample sent before the marker is played,
     * for a real-time master and for one that is ahead, and the subdevice
     * returns to idle without a trip. */
    for (int k = 0; k < 3; k++) {
        bool realtime = k < 2;
        double M = k == 0 ? 2 : 10;
        uint16_t port = (uint16_t)(base + 30 + k);   /* < base + 50 */
        CHECK_EQ(cari_m_bind_ul(m, 0, port), CARI_OK);
        void *push = cari_master_connect_ul(m, port, 2, 16 * 1024);
        rru_stats_t st0, st;
        rru_get_stats(R, &st0);
        size_t n = (size_t)(250000 * M / 1000), msgs = (size_t)(0.5 * 1000 / M) + 3;  /* odd total */
        size_t cap = n > 777 ? n : 777;
        int16_t *iq = calloc(cap * 2, sizeof(int16_t));
        for (size_t i = 0; i < cap; i++)
            iq[2 * i] = 8000;
        CHECK_EQ(cari_m_exec(m, 0, CARI_ACT_TX_START), CARI_OK);
        double t0 = now_s();
        for (size_t j = 0; j < msgs; j++) {
            if (realtime)
                sleep_until(t0 + j * M / 1000.0);
            zmq_send(push, iq, n * 4, 0);
        }
        zmq_send(push, iq, 777 * 4, 0);         /* a short last message */
        CHECK_EQ(cari_master_ul_send_end(push), 0);
        int rc = cari_m_wait_tx_idle(m, 0, 3000);
        double t = now_s() - t0;
        rru_get_stats(R, &st);
        uint64_t played = st.ul_consumed - st0.ul_consumed;
        fprintf(stderr, "   end marker, %s, %2.0f ms messages: %llu of %zu samples played, idle after %.3f s (%s)\n",
                realtime ? "real-time" : "ahead    ", M, (unsigned long long)played, msgs * n + 777, t,
                rc == CARI_OK ? "no trip" : cari_strerror(rc));
        CHECK_EQ(rc, CARI_OK);
        CHECK_EQ(played, msgs * n + 777);
        CHECK_EQ(st.ul_underruns - st0.ul_underruns, 0);
        CHECK_EQ(st.ul_ends - st0.ul_ends, 1);
        double air = (msgs * n + 777) / 250000.0;
        CHECK(t >= air - 0.02 && t < air + 0.3);
        uint32_t f = 1;
        cari_m_ping(m, &f);
        CHECK_EQ(f, 0);
        free(iq);
        zmq_close(push);
    }
    /* end marker with no samples: back to idle, never keyed */
    {
        uint16_t port = (uint16_t)(base + 40);
        CHECK_EQ(cari_m_bind_ul(m, 0, port), CARI_OK);
        void *push = cari_master_connect_ul(m, port, 2, 16 * 1024);
        CHECK_EQ(cari_m_exec(m, 0, CARI_ACT_TX_START), CARI_OK);
        CHECK_EQ(cari_master_ul_send_end(push), 0);
        CHECK_EQ(cari_m_wait_tx_idle(m, 0, 1000), CARI_OK);
        zmq_close(push);
    }

    cari_master_free(m);
    __atomic_store_n(&stop, 1, __ATOMIC_SEQ_CST);
    pthread_join(th, NULL);
    rru_free(R);
    T_DONE();
}
