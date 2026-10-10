/* SX1255 driver tests: datasheet numbers, register encodings, driver I/O. */
#include "t.h"
#include "sx1255.h"

/* ---- fake hardware recording register writes ---- */
struct rru_hw {
    uint8_t reg[SX1255_NREGS];
    uint8_t log_a[256], log_v[256];
    int     nlog;
    int     resets;
    uint8_t version;
};

static int f_reset(rru_hw_t *h) { h->resets++; return 0; }
static int f_w(rru_hw_t *h, uint8_t a, uint8_t v)
{
    h->reg[a] = v;
    if (h->nlog < 256) { h->log_a[h->nlog] = a; h->log_v[h->nlog] = v; h->nlog++; }
    return 0;
}
static int f_r(rru_hw_t *h, uint8_t a, uint8_t *v)
{
    *v = a == SX1255_VERSION ? h->version : a == SX1255_STAT ? 0x07 : h->reg[a];
    return 0;
}
static const rru_hw_ops_t fake = { .name = "fake", .chip_reset = f_reset, .spi_write = f_w, .spi_read = f_r };

static void test_frequency(void)
{
    /* the reset value 0xC0E38E is 434 MHz at 36 MHz: proves the 2^20 divisor */
    CHECK_EQ(sx1255_frf_word(434000000, 36e6), 0xC0E38E);
    CHECK_NEAR(sx1255_frf_hz(0xC0E38E, 36e6), 434e6, 20);
    /* 32 MHz: step 30.5 Hz, rounding error at most half a step */
    for (uint64_t f = 400000000; f <= 510000000; f += 1234567) {
        double got = sx1255_frf_hz(sx1255_frf_word(f, 32e6), 32e6);
        CHECK(fabs(got - (double)f) <= 32e6 / 1048576.0 / 2 + 1e-6);
    }
    /* ppm correction: a crystal 10 ppm fast needs a smaller word */
    CHECK(sx1255_frf_word(433475000, 32e6 * (1 + 10e-6)) < sx1255_frf_word(433475000, 32e6));
}

static void test_lna(void)
{
    /* dB above the lowest step G6, as libsx1255: G1 = 48 ... G6 = 0 */
    CHECK_EQ(sx1255_lna_code(48), 1);
    CHECK_EQ(sx1255_lna_code(42), 2);
    CHECK_EQ(sx1255_lna_code(36), 3);
    CHECK_EQ(sx1255_lna_code(24), 4);
    CHECK_EQ(sx1255_lna_code(12), 5);
    CHECK_EQ(sx1255_lna_code(0), 6);
    CHECK_EQ(sx1255_lna_code(30), -1);
    CHECK_EQ(sx1255_lna_code(-6), -1);
    for (int i = 0; i < 6; i++)
        CHECK(sx1255_lna_db(sx1255_lna_code(sx1255_lna_steps[i])) == sx1255_lna_steps[i]);
}

static void test_tx_power(void)
{
    sx1255_txgain_t g;
    const float pmax = 5.0f;
    CHECK_NEAR(sx1255_tx_dbm(3, 15, pmax), 5.0, 1e-6);           /* DAC 0 dB, mixer -7.5 dB */
    CHECK_NEAR(sx1255_tx_dbm(0, 0, pmax), -34.0, 1e-6);          /* -9 dB, -37.5 dB */
    CHECK_NEAR(sx1255_tx_min_dbm(pmax), -34.0, 1e-6);
    sx1255_tx_gain_for(5, pmax, &g);
    CHECK(g.dac == 3 && g.mixer == 15 && g.dbm == 5.0f);
    sx1255_tx_gain_for(-34, pmax, &g);
    CHECK(g.dac == 0 && g.mixer == 0);
    sx1255_tx_gain_for(10, pmax, &g);                            /* clamp */
    CHECK(g.dbm == 5.0f);
    sx1255_tx_gain_for(-60, pmax, &g);
    CHECK(g.dbm == -34.0f);
    /* every integer offset 0..39 dB below pmax except 1 and 38 is exact */
    for (int off = 0; off <= 39; off++) {
        sx1255_tx_gain_for(pmax - off, pmax, &g);
        if (off == 1 || off == 38)
            CHECK_NEAR(fabs(g.dbm - (pmax - off)), 1.0, 1e-6);
        else
            CHECK_NEAR(g.dbm, pmax - off, 1e-6);
        CHECK(g.dac <= 3 && g.mixer <= 15);
    }
    /* unreachable values round down (ties go to the lower power) */
    sx1255_tx_gain_for(4, pmax, &g);
    CHECK(g.dbm == 3.0f);
    sx1255_tx_gain_for(-33, pmax, &g);
    CHECK(g.dbm == -34.0f);
    sx1255_tx_gain_for(-10.4f, pmax, &g);
    CHECK(g.dbm == -10.0f);
    /* equal power: the higher DAC gain is used */
    sx1255_tx_gain_for(-1.0f, pmax, &g);   /* -6 dB: DAC -6 dB + mixer max, or DAC 0 dB + mixer -6 dB */
    CHECK(g.dac == 3 && g.mixer == 12);
}

static void test_rates(void)
{
    sx1255_rate_t r;
    /* 32 MHz crystal, I2S mode B2 with 32-bit slots (as libsx1255):
     * CLK_OUT = 64 x rate */
    CHECK_EQ(sx1255_rate_config(32e6, 125000, SX1255_SLOT_BITS, &r), 0);
    CHECK(r.factor == 256 && r.mant9 == 0 && r.m == 0 && r.n == 5 && r.clk_div == 4 && r.div_code == 2);
    CHECK_EQ(sx1255_rate_config(32e6, 250000, SX1255_SLOT_BITS, &r), 0);
    CHECK(r.factor == 128 && r.n == 4 && r.clk_div == 2 && r.div_code == 1);
    CHECK_EQ(sx1255_rate_config(32e6, 500000, SX1255_SLOT_BITS, &r), 0);
    CHECK(r.factor == 64 && r.n == 3 && r.clk_div == 1 && r.div_code == 0);
    /* 16-bit slots would use dividers 8/4/2 */
    CHECK_EQ(sx1255_rate_config(32e6, 125000, 16, &r), 0);
    CHECK(r.clk_div == 8 && r.div_code == 3);
    /* lowest rate of the chip, 32 MHz / 1536: BCLK = XTAL / 24 */
    CHECK_EQ(sx1255_rate_config(32e6, 32e6 / 1536, SX1255_SLOT_BITS, &r), 0);
    CHECK(r.factor == 1536 && r.clk_div == 24 && r.div_code == 6);
    CHECK_EQ(sx1255_rate_config(32e6, 300000, SX1255_SLOT_BITS, &r), -1);  /* not a factor */
    CHECK_EQ(sx1255_rate_config(32e6, 1e6, SX1255_SLOT_BITS, &r), -1);     /* BCLK > XTAL */
}

static void test_driver(void)
{
    rru_hw_t h = { .version = SX1255_CHIP_VERSION };
    sx1255_t d;
    sx1255_attach(&d, &fake, &h, 32e6);
    CHECK_EQ(sx1255_init(&d), 0);
    CHECK_EQ(h.resets, 1);
    CHECK(!d.comm_error);
    CHECK_EQ(h.reg[SX1255_MODE], SX1255_MODE_REF);
    CHECK_EQ(h.reg[SX1255_IISM] & 0x30, 0x20);                   /* I2S mode B2 */
    CHECK(h.reg[SX1255_DIG_BRIDGE] & 0x04);                      /* MSB aligned */
    CHECK_EQ(h.reg[SX1255_RXFE2], 0x37);                         /* as libsx1255 */
    CHECK_EQ(h.reg[SX1255_TXFE4], 5);

    h.nlog = 0;
    CHECK_EQ(sx1255_set_freq(&d, false, 434000000, 0), 0);
    CHECK_EQ(h.nlog, 3);
    CHECK(h.log_a[0] == SX1255_FRFH_RX && h.log_a[2] == SX1255_FRFL_RX);  /* LSB last (latch) */
    uint32_t w = (uint32_t)h.reg[1] << 16 | h.reg[2] << 8 | h.reg[3];
    CHECK_EQ(w, sx1255_frf_word(434000000, 32e6));
    sx1255_set_freq(&d, true, 435000000, 0);
    w = (uint32_t)h.reg[4] << 16 | h.reg[5] << 8 | h.reg[6];
    CHECK_EQ(w, sx1255_frf_word(435000000, 32e6));

    CHECK_EQ(sx1255_set_rx_gain(&d, 3, 15), 0);
    CHECK_EQ(h.reg[SX1255_RXFE1], (3 << 5) | (15 << 1));        /* 50 ohm input */
    CHECK_EQ(sx1255_set_rx_gain(&d, 7, 15), -1);
    CHECK_EQ(sx1255_set_tx_gain(&d, 2, 9), 0);
    CHECK_EQ(h.reg[SX1255_TXFE1], 0x29);

    sx1255_rate_t rc;
    sx1255_rate_config(32e6, 250000, SX1255_SLOT_BITS, &rc);
    sx1255_set_rate(&d, &rc);
    /* the exact values libsx1255 writes for 250 kHz */
    CHECK_EQ(h.reg[SX1255_DIG_BRIDGE], (0 << 7) | (0 << 6) | (4 << 3) | (1 << 2));
    CHECK_EQ(h.reg[SX1255_IISM], (2 << 4) | 1);

    sx1255_set_mode(&d, SX1255_MODE_RX | SX1255_MODE_TX, SX1255_MODE_RX | SX1255_MODE_TX);
    CHECK_EQ(h.reg[SX1255_MODE], 0x07);
    sx1255_set_mode(&d, SX1255_MODE_PA, SX1255_MODE_PA);
    CHECK_EQ(h.reg[SX1255_MODE], 0x0F);
    sx1255_set_mode(&d, 0xFF, 0);
    CHECK_EQ(h.reg[SX1255_MODE], SX1255_MODE_REF);               /* oscillator stays on */
    CHECK(sx1255_locked(&d, true, true));

    /* wrong chip: communication error */
    h.version = 0x00;
    CHECK_EQ(sx1255_init(&d), -1);
    CHECK(d.comm_error);
    sx1255_detach(&d);
}

int main(void)
{
    RUN(test_frequency);
    RUN(test_lna);
    RUN(test_tx_power);
    RUN(test_rates);
    RUN(test_driver);
    T_DONE();
}
