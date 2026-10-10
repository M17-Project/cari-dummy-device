/* UL ring buffer tests */
#include "t.h"
#include "ring.h"

#include <pthread.h>
#include <time.h>

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static ring_t rg;

static void *consumer(void *p)
{
    (void)p;
    int16_t b[2 * 10];
    struct timespec ts = { 0, 20 * 1000000L };
    nanosleep(&ts, NULL);
    ring_read(&rg, b, 10);
    return NULL;
}

int main(void)
{
    int16_t in[2 * 100], out[2 * 100];
    for (int i = 0; i < 200; i++)
        in[i] = (int16_t)i;
    CHECK_EQ(ring_init(&rg, 64), 0);

    /* wrap-around keeps I/Q pairs intact */
    CHECK_EQ(ring_write(&rg, in, 50, 0), 50);
    CHECK_EQ(ring_read(&rg, out, 40), 40);
    CHECK_EQ(ring_write(&rg, in + 100, 50, 0), 50);
    CHECK_EQ(ring_level(&rg), 60);
    CHECK_EQ(ring_read(&rg, out, 100), 60);
    CHECK_EQ(out[0], 80);
    CHECK_EQ(out[19], 99);
    CHECK_EQ(out[20], 100);
    CHECK_EQ(out[119], 199);

    /* full: a write returns what fits; then blocks until timeout */
    CHECK_EQ(ring_write(&rg, in, 100, 0), 64);
    double t0 = now_s();
    CHECK_EQ(ring_write(&rg, in, 10, 50), 0);
    CHECK(now_s() - t0 >= 0.045);

    /* a consumer frees space: the blocked writer proceeds */
    pthread_t th;
    pthread_create(&th, NULL, consumer, NULL);
    t0 = now_s();
    CHECK_EQ(ring_write(&rg, in, 10, 1000), 10);
    CHECK(now_s() - t0 >= 0.015 && now_s() - t0 < 0.5);
    pthread_join(th, NULL);

    /* wait_level */
    ring_reset(&rg);
    CHECK(!ring_wait_level(&rg, 5, 20));
    ring_write(&rg, in, 5, 0);
    CHECK(ring_wait_level(&rg, 5, 0));

    /* close wakes and refuses */
    ring_close(&rg);
    CHECK_EQ(ring_write(&rg, in, 1, 1000), 0);
    CHECK(!ring_wait_level(&rg, 10, 1000));
    ring_free(&rg);
    T_DONE();
}
