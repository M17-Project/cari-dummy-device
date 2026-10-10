/*
 * Single-producer single-consumer ring of CS16 I/Q samples with blocking
 * writes - the RRU's UL buffer between the network and the converter.
 */
#ifndef RRU_RING_H
#define RRU_RING_H

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int16_t        *buf;        /* cap * 2 int16 (I, Q) */
    size_t          cap;        /* samples */
    size_t          r, count;
    bool            closed;
    pthread_mutex_t m;
    pthread_cond_t  not_full;
    pthread_cond_t  changed;
} ring_t;

int    ring_init(ring_t *rg, size_t cap_samples);
void   ring_free(ring_t *rg);
void   ring_reset(ring_t *rg);              /* empty and reopen */
void   ring_close(ring_t *rg);              /* wake all waiters */
size_t ring_level(ring_t *rg);
/* Write up to n samples, waiting for space up to timeout_ms. Returns the
 * number written (0 on timeout or close). */
size_t ring_write(ring_t *rg, const int16_t *iq, size_t n, int timeout_ms);
/* Read up to n samples without blocking. */
size_t ring_read(ring_t *rg, int16_t *iq, size_t n);
/* Wait until at least n samples are buffered (or close/timeout). */
bool   ring_wait_level(ring_t *rg, size_t n, int timeout_ms);

#endif
