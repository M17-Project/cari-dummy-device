/*
 * cari-dummy-device - CARI 2.1 slave emulating the M17 SX1255 RRU
 *
 * Author: Wojciech Kaczmarski, SP5WWP (original dummy device)
 * Rewritten on libcari for CARI 2.0, updated for 2.1.
 */
#include "rru.h"
#include "term.h"

#include <cari/cari_config.h>
#include <cari/cari_log.h>
#include <cari/cari_proto.h>

#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile int stop;

static void on_signal(int sig)
{
    (void)sig;
    __atomic_store_n(&stop, 1, __ATOMIC_SEQ_CST);
}

static bool use_color;

static void log_handler(void *user, cari_log_level_t lvl, const char *msg)
{
    (void)user;
    static const char *tag[] = { "DBG", "INF", "WRN", "ERR" };
    static const char *col[] = { TERM_BLUE, TERM_GREEN, TERM_YELLOW, TERM_RED };
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);
    fprintf(stderr, "%02d:%02d:%02d.%03ld %s%s%s %s\n", tm.tm_hour, tm.tm_min, tm.tm_sec,
            ts.tv_nsec / 1000000, use_color ? col[lvl & 3] : "", tag[lvl & 3],
            use_color ? TERM_DEFAULT : "", msg);
}

static void usage(const char *p)
{
    printf("Usage: %s [OPTIONS]\n\n"
           "CARI %d.%d slave emulating the M17 SX1255 RRU: one full-duplex I/Q subdevice,\n"
           "CS16, 125/250/500 kHz, 400-510 MHz, FLAC compression and supervision.\n\n"
           "  -c, --ctrl=PORT|EP     CTRL REP port or ZMQ endpoint (default 17002)\n"
           "  -b, --bind=HOST        data-plane bind address (default * = all interfaces)\n"
           "  -i, --ident=STR        IDENT string\n"
           "  -x, --xtal=HZ          SX1255 crystal (default 32e6)\n"
           "  -P, --pmax=DBM         output power at full TX gain (default 5.0, SX1255 P1dB typ)\n"
           "  -g, --pga=CODE         RX PGA gain code 0..15 (default 15 = +6 dB)\n"
           "  -t, --period=MS        DL message period (default 10)\n"
           "      --ul-period=US     UL drain period (default 1000)\n"
           "  -u, --ul-buffer=MS     UL ring buffer (default 60)\n"
           "  -p, --prefill=MS       UL samples buffered before keying\n"
           "                         (default 0 = largest UL message + drain period + ramp + jitter)\n"
           "      --jitter=MS        UL arrival jitter margin for the automatic prefill (default 5)\n"
           "      --ul-rcvbuf=BYTES  UL socket receive buffer (default 32768, 0 = OS default)\n"
           "  -f, --freq=HZ          default RX/TX frequency (default 433475000)\n"
           "  -H, --hw=NAME          hardware backend: dummy (default)"
#ifdef RRU_HAVE_HW_RPI
           ", rpi (skeleton)"
#endif
           "\n"
           "      --spi=DEV          rpi: SPI device (default /dev/spidev0.0)\n"
           "      --reset-gpio=N     rpi: SX1255 RESET line (default 22)\n"
           "      --pa-gpio=N        rpi: PA / T-R switch line\n"
           "  -s, --seed=N           dummy: noise seed\n"
           "  -v, --verbose          more logging (-vv: debug, -vvv: SPI trace)\n"
           "  -h, --help             this help\n\n"
           "Built with libcari %s%s.\n",
           p, CARI_VERSION_MAJOR, CARI_VERSION_MINOR, CARI_LIB_VERSION,
#ifdef CARI_HAVE_FLAC
           " (FLAC)"
#else
           " (no FLAC)"
#endif
    );
}

int main(int argc, char **argv)
{
    rru_cfg_t cfg;
    rru_cfg_defaults(&cfg);
    static char ep[256];
    int verbose = 0;

    static const struct option lo[] = {
        { "ctrl", required_argument, 0, 'c' },     { "bind", required_argument, 0, 'b' },
        { "ident", required_argument, 0, 'i' },    { "xtal", required_argument, 0, 'x' },
        { "pmax", required_argument, 0, 'P' },     { "pga", required_argument, 0, 'g' },
        { "period", required_argument, 0, 't' },   { "ul-buffer", required_argument, 0, 'u' },
        { "prefill", required_argument, 0, 'p' },  { "freq", required_argument, 0, 'f' },
        { "hw", required_argument, 0, 'H' },       { "spi", required_argument, 0, 1 },
        { "reset-gpio", required_argument, 0, 2 }, { "pa-gpio", required_argument, 0, 3 },
        { "ul-rcvbuf", required_argument, 0, 4 },
        { "ul-period", required_argument, 0, 5 },   { "jitter", required_argument, 0, 6 },
        { "seed", required_argument, 0, 's' },     { "verbose", no_argument, 0, 'v' },
        { "help", no_argument, 0, 'h' },           { 0, 0, 0, 0 },
    };
    int o;
    while ((o = getopt_long(argc, argv, "c:b:i:x:P:g:t:u:p:f:H:s:vh", lo, NULL)) != -1) {
        switch (o) {
        case 'c':
            if (strchr(optarg, ':'))
                snprintf(ep, sizeof ep, "%s", optarg);
            else
                snprintf(ep, sizeof ep, "tcp://*:%s", optarg);
            cfg.ctrl_endpoint = ep;
            break;
        case 'b': cfg.bind_host = optarg; break;
        case 'i': cfg.ident = optarg; break;
        case 'x': cfg.hw_cfg.xtal_hz = atof(optarg); break;
        case 'P': cfg.hw_cfg.pmax_dbm = (float)atof(optarg); break;
        case 'g': cfg.pga_code = atoi(optarg); break;
        case 't': cfg.period_ms = (unsigned)atoi(optarg); break;
        case 'u': cfg.ul_buffer_ms = (unsigned)atoi(optarg); break;
        case 'p': cfg.ul_prefill_ms = (unsigned)atoi(optarg); break;
        case 'f': cfg.default_freq = (uint64_t)atof(optarg); break;
        case 'H':
            if (!strcmp(optarg, "dummy"))
                cfg.hw = &rru_hw_dummy;
#ifdef RRU_HAVE_HW_RPI
            else if (!strcmp(optarg, "rpi"))
                cfg.hw = &rru_hw_rpi;
#endif
            else {
                fprintf(stderr, "unknown backend '%s'\n", optarg);
                return 1;
            }
            break;
        case 1: cfg.hw_cfg.spi_dev = optarg; break;
        case 2: cfg.hw_cfg.gpio_reset = atoi(optarg); break;
        case 3: cfg.hw_cfg.gpio_pa = atoi(optarg); break;
        case 4: cfg.ul_rcvbuf = atoi(optarg); break;
        case 5: cfg.ul_period_us = (unsigned)atoi(optarg); break;
        case 6: cfg.ul_jitter_ms = (unsigned)atoi(optarg); break;
        case 's': cfg.hw_cfg.seed = (unsigned)atoi(optarg); break;
        case 'v': verbose++; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 1;
        }
    }
    if (cfg.pga_code < 0 || cfg.pga_code > 15 || cfg.period_ms < 1 || cfg.period_ms > 100 ||
        cfg.ul_period_us < 100 || cfg.ul_period_us > 20000 ||
        cfg.hw_cfg.xtal_hz < 32e6 || cfg.hw_cfg.xtal_hz > 36.864e6) {
        fprintf(stderr, "invalid option value (see --help; XTAL range is 32..36.864 MHz)\n");
        return 1;
    }
    cfg.hw_cfg.verbose = verbose >= 3;

    use_color = isatty(STDERR_FILENO);
    cari_log_set_handler(log_handler, NULL, verbose >= 2 ? CARI_LOG_DEBUG : CARI_LOG_INFO);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    cari_log(CARI_LOG_INFO, "%s - backend '%s', XTAL %.3f MHz, Pmax %+.1f dBm", cfg.ident,
             cfg.hw->name, cfg.hw_cfg.xtal_hz / 1e6, cfg.hw_cfg.pmax_dbm);
    rru_t *r = rru_new(&cfg);
    if (!r)
        return 1;
    int rc = rru_run(r, &stop);

    rru_stats_t st;
    rru_get_stats(r, &st);
    cari_log(CARI_LOG_INFO, "shutting down: DL %llu samples, UL %llu samples, %llu underrun(s)",
             (unsigned long long)st.dl_produced, (unsigned long long)st.ul_consumed,
             (unsigned long long)st.ul_underruns);
    rru_free(r);
    return rc ? 1 : 0;
}
