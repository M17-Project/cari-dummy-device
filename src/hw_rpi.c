/*
 * Raspberry Pi RRU backend - SKELETON
 *
 * Shows where the real hardware plugs into the shared RRU core. The SPI
 * (spidev) and GPIO (GPIO character device, uAPI v2) parts are real code;
 * the I2S sample path is not written yet (TODO: ALSA, see below). Selecting
 * this backend therefore fails when a stream is started.
 *
 * Wiring and settings follow libsx1255 (M17-Project/libsx1255):
 *   SPI0 CE0 -> SX1255 NSS/SCK/MOSI/MISO      (/dev/spidev0.0, mode 0, 500 kHz)
 *   GPIO 22  -> SX1255 RESET (active high)    (/dev/gpiochip0, cfg.gpio_reset)
 *   GPIO m   -> external PA / T-R switch       (cfg.gpio_pa, optional)
 *   I2S      <- SX1255 in mode B2 as I2S master: CLK_OUT -> BCLK,
 *               DIO2 (WS) -> LRCLK, IQ_OUT -> DIN, IQ_IN <- DOUT
 *               32-bit slots, 16 significant bits MSB-aligned
 *               (BCLK = 64 x rate: 8/16/32 MHz at 125/250/500 kHz)
 */
#include "hw.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

struct rru_hw {
    rru_hw_cfg_t cfg;
    int          spi_fd;
    int          reset_fd;      /* GPIO line request fds, -1 if unused */
    int          pa_fd;
};

static int gpio_request(unsigned line, const char *label)
{
    int chip = open("/dev/gpiochip0", O_RDWR | O_CLOEXEC);
    if (chip < 0)
        return -1;
    struct gpio_v2_line_request req;
    memset(&req, 0, sizeof req);
    req.offsets[0] = line;
    req.num_lines = 1;
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    snprintf(req.consumer, sizeof req.consumer, "%s", label);
    int r = ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req);
    close(chip);
    return r < 0 ? -1 : req.fd;
}

static int gpio_set(int fd, int v)
{
    if (fd < 0)
        return 0;
    struct gpio_v2_line_values vals = { .bits = v ? 1 : 0, .mask = 1 };
    return ioctl(fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals) < 0 ? -1 : 0;
}

static rru_hw_t *p_open(const rru_hw_cfg_t *cfg)
{
    rru_hw_t *h = calloc(1, sizeof *h);
    if (!h)
        return NULL;
    h->cfg = *cfg;
    h->reset_fd = h->pa_fd = -1;
    h->spi_fd = open(cfg->spi_dev ? cfg->spi_dev : "/dev/spidev0.0", O_RDWR | O_CLOEXEC);
    if (h->spi_fd < 0) {
        perror("spidev");
        free(h);
        return NULL;
    }
    uint8_t mode = SPI_MODE_0;
    uint32_t speed = 500000;
    ioctl(h->spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(h->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (cfg->gpio_reset >= 0)
        h->reset_fd = gpio_request((unsigned)cfg->gpio_reset, "sx1255-reset");
    if (cfg->gpio_pa >= 0)
        h->pa_fd = gpio_request((unsigned)cfg->gpio_pa, "rru-pa");
    return h;
}

static void p_close(rru_hw_t *h)
{
    gpio_set(h->pa_fd, 0);
    if (h->pa_fd >= 0) close(h->pa_fd);
    if (h->reset_fd >= 0) close(h->reset_fd);
    close(h->spi_fd);
    free(h);
}

static int p_chip_reset(rru_hw_t *h)
{
    /* RESET high for >= 100 us, release, wait 5 ms (datasheet 6.2) */
    struct timespec hi = { 0, 200000 }, wait = { 0, 6000000 };
    if (gpio_set(h->reset_fd, 1))
        return -1;
    nanosleep(&hi, NULL);
    if (gpio_set(h->reset_fd, 0))
        return -1;
    nanosleep(&wait, NULL);
    return 0;
}

static int spi_xfer(rru_hw_t *h, uint8_t a0, uint8_t a1, uint8_t *rx1)
{
    uint8_t tx[2] = { a0, a1 }, rx[2] = { 0 };
    struct spi_ioc_transfer t;
    memset(&t, 0, sizeof t);
    t.tx_buf = (unsigned long)tx;
    t.rx_buf = (unsigned long)rx;
    t.len = 2;
    if (ioctl(h->spi_fd, SPI_IOC_MESSAGE(1), &t) < 0)
        return -1;
    if (rx1)
        *rx1 = rx[1];
    return 0;
}

/* SX1255 SPI: first byte = wnr (bit 7, 1 = write) | address */
static int p_spi_write(rru_hw_t *h, uint8_t a, uint8_t v) { return spi_xfer(h, 0x80 | a, v, NULL); }
static int p_spi_read(rru_hw_t *h, uint8_t a, uint8_t *v)  { return spi_xfer(h, a & 0x7F, 0, v); }
static int p_pa_enable(rru_hw_t *h, bool on)               { return gpio_set(h->pa_fd, on); }

/*
 * TODO(I2S/ALSA): open cfg.pcm_dev for capture and playback, S32_LE, 2
 * channels (I = left, Q = right), rate = sample rate, slave clocking (the
 * SX1255 is the I2S master). Convert to/from the core's CS16 by taking the
 * upper 16 bits of each 32-bit slot (MSB-aligned, DIG_BRIDGE bit 2). Keep the playback buffer short (a few periods):
 * the RRU core's ring buffer is where the UL ramp/trip happens, so samples
 * already handed to ALSA cannot be ramped any more.
 *   rx_read:  snd_pcm_readi(), recover with snd_pcm_prepare() on -EPIPE
 *   tx_write: snd_pcm_writei(), blocks while the buffer is full (this is
 *             what paces the UL plane); start threshold = one period
 */
static int p_rx_open(rru_hw_t *h, uint32_t rate)
{
    (void)h; (void)rate;
    fprintf(stderr, "rpi backend: I2S capture not implemented\n");
    return -1;
}
static long p_rx_read(rru_hw_t *h, int16_t *iq, size_t n) { (void)h; (void)iq; (void)n; return -ENOSYS; }
static void p_rx_close(rru_hw_t *h) { (void)h; }
static int p_tx_open(rru_hw_t *h, uint32_t rate)
{
    (void)h; (void)rate;
    fprintf(stderr, "rpi backend: I2S playback not implemented\n");
    return -1;
}
static long p_tx_write(rru_hw_t *h, const int16_t *iq, size_t n) { (void)h; (void)iq; (void)n; return -ENOSYS; }
static void p_tx_close(rru_hw_t *h) { (void)h; }

/* TODO(sensors): temperature (SX1255 ADC temperature mode or a board
 * sensor), supply voltage/current (e.g. INA219), RF power detector. */
static int p_sensor(rru_hw_t *h, uint8_t q, float *v) { (void)h; (void)q; (void)v; return -1; }

const rru_hw_ops_t rru_hw_rpi = {
    .name = "rpi",
    .open = p_open,
    .close = p_close,
    .chip_reset = p_chip_reset,
    .spi_write = p_spi_write,
    .spi_read = p_spi_read,
    .pa_enable = p_pa_enable,
    .rx_open = p_rx_open,
    .rx_read = p_rx_read,
    .rx_close = p_rx_close,
    .tx_open = p_tx_open,
    .tx_write = p_tx_write,
    .tx_close = p_tx_close,
    .sensor = p_sensor,
};
