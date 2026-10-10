/*
 * SX1255 RRU core - the hardware-independent part of the RRU slave.
 *
 * Presents one full-duplex I/Q subdevice (CS16, 125/250/500 kHz,
 * 400..510 MHz) over CARI 2.1 using libcari, drives the SX1255 through the
 * register-level driver and moves samples through an rru_hw_ops_t backend.
 *
 * UL path:  ZMQ PULL -> feeder thread -> ring buffer -> drain thread ->
 *           hw->tx_write() (blocks at the converter rate)
 *   The feeder only reads from the socket when the ring has room, so a
 *   master that sends too fast is held back by ZMQ flow control. The drain
 *   keys the PA once the prefill is buffered (by default: the largest UL
 *   message + one drain period + the ramp + a jitter margin) and plays one
 *   drain period (1 ms) at a time while at least one period plus one ramp
 *   length is buffered. When less is left and the end-of-transmission
 *   marker has been read (CARI 2.1), it plays the rest with a ramp-down on
 *   the tail, unkeys and reports the end; otherwise the buffer is about to
 *   run dry: it ramps all remaining samples down to zero, plays them, unkeys
 *   and reports the underrun.
 * DL path:  hw->rx_read() (blocks at the converter rate) -> DL thread ->
 *           ZMQ PUB (never blocks; FLAC-compressed if enabled)
 */
#ifndef RRU_H
#define RRU_H

#include "hw.h"

typedef struct rru rru_t;

typedef struct {
    const rru_hw_ops_t *hw;
    rru_hw_cfg_t        hw_cfg;
    const char         *ident;
    const char         *ctrl_endpoint;      /* tcp://<any>:17002 */
    const char         *bind_host;          /* data planes, "*" */
    int                 pga_code;           /* fixed RX PGA gain code 0..15 */
    unsigned            period_ms;          /* DL message period */
    unsigned            ul_period_us;       /* UL drain period (converter writes) */
    unsigned            ul_buffer_ms;       /* UL ring capacity */
    unsigned            ul_prefill_ms;      /* buffered before keying, 0 = auto */
    unsigned            ul_jitter_ms;       /* auto prefill: arrival jitter margin */
    unsigned            ramp_us;            /* key-on / underrun ramp reserve */
    int                 ul_rcvbuf;          /* UL socket receive buffer (bytes) */
    uint64_t            default_freq;
    float               default_power;
    float               default_rate;
} rru_cfg_t;

void   rru_cfg_defaults(rru_cfg_t *c);
rru_t *rru_new(const rru_cfg_t *cfg);
int    rru_run(rru_t *r, volatile int *stop);
void   rru_free(rru_t *r);

/* For tests / logs */
typedef struct {
    uint64_t ul_underruns;
    uint64_t ul_consumed;       /* samples handed to the converter */
    uint64_t dl_produced;
    uint64_t key_delay_us;      /* last key-up: first UL sample -> PA keyed */
    uint64_t ul_ends;           /* transmissions ended by the UL end marker */
} rru_stats_t;
void rru_get_stats(rru_t *r, rru_stats_t *st);

#endif
