#include "ring.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void deadline(struct timespec *ts, int ms)
{
    clock_gettime(CLOCK_MONOTONIC, ts);
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

int ring_init(ring_t *rg, size_t cap)
{
    memset(rg, 0, sizeof *rg);
    rg->buf = calloc(cap * 2, sizeof(int16_t));
    if (!rg->buf)
        return -1;
    rg->cap = cap;
    pthread_condattr_t ca;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_mutex_init(&rg->m, NULL);
    pthread_cond_init(&rg->not_full, &ca);
    pthread_cond_init(&rg->changed, &ca);
    pthread_condattr_destroy(&ca);
    return 0;
}

void ring_free(ring_t *rg)
{
    free(rg->buf);
    rg->buf = NULL;
    pthread_mutex_destroy(&rg->m);
    pthread_cond_destroy(&rg->not_full);
    pthread_cond_destroy(&rg->changed);
}

void ring_reset(ring_t *rg)
{
    pthread_mutex_lock(&rg->m);
    rg->r = rg->count = 0;
    rg->closed = false;
    pthread_mutex_unlock(&rg->m);
}

void ring_close(ring_t *rg)
{
    pthread_mutex_lock(&rg->m);
    rg->closed = true;
    pthread_cond_broadcast(&rg->not_full);
    pthread_cond_broadcast(&rg->changed);
    pthread_mutex_unlock(&rg->m);
}

size_t ring_level(ring_t *rg)
{
    pthread_mutex_lock(&rg->m);
    size_t c = rg->count;
    pthread_mutex_unlock(&rg->m);
    return c;
}

size_t ring_write(ring_t *rg, const int16_t *iq, size_t n, int timeout_ms)
{
    struct timespec ts;
    deadline(&ts, timeout_ms);
    pthread_mutex_lock(&rg->m);
    while (!rg->closed && rg->count == rg->cap)
        if (pthread_cond_timedwait(&rg->not_full, &rg->m, &ts) == ETIMEDOUT)
            break;
    size_t done = 0;
    if (!rg->closed) {
        size_t space = rg->cap - rg->count;
        size_t k = n < space ? n : space;
        size_t w = (rg->r + rg->count) % rg->cap;
        size_t first = k < rg->cap - w ? k : rg->cap - w;
        memcpy(rg->buf + 2 * w, iq, first * 4);
        memcpy(rg->buf, iq + 2 * first, (k - first) * 4);
        rg->count += k;
        done = k;
        if (k)
            pthread_cond_broadcast(&rg->changed);
    }
    pthread_mutex_unlock(&rg->m);
    return done;
}

size_t ring_read(ring_t *rg, int16_t *iq, size_t n)
{
    pthread_mutex_lock(&rg->m);
    size_t k = n < rg->count ? n : rg->count;
    size_t first = k < rg->cap - rg->r ? k : rg->cap - rg->r;
    memcpy(iq, rg->buf + 2 * rg->r, first * 4);
    memcpy(iq + 2 * first, rg->buf, (k - first) * 4);
    rg->r = (rg->r + k) % rg->cap;
    rg->count -= k;
    if (k)
        pthread_cond_broadcast(&rg->not_full);
    pthread_mutex_unlock(&rg->m);
    return k;
}

bool ring_wait_level(ring_t *rg, size_t n, int timeout_ms)
{
    struct timespec ts;
    deadline(&ts, timeout_ms);
    pthread_mutex_lock(&rg->m);
    while (!rg->closed && rg->count < n)
        if (pthread_cond_timedwait(&rg->changed, &rg->m, &ts) == ETIMEDOUT)
            break;
    bool ok = rg->count >= n;
    pthread_mutex_unlock(&rg->m);
    return ok;
}
