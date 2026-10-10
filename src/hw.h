/*
 * RRU hardware abstraction
 *
 * Everything above this interface (CARI handling, SX1255 register logic,
 * UL ring buffer, underrun ramp/trip, DL pump) is shared between the
 * emulated device (hw_dummy.c) and the real RRU (Raspberry Pi: SPI to the
 * SX1255, GPIO reset/PA lines, I2S through ALSA - see hw_rpi.c).
 *
 * A backend only moves bytes and samples:
 *   - SPI register access and the chip reset line
 *   - an optional PA / T-R switch line
 *   - blocking sample I/O paced by the converter clock (the I2S bit clock of
 *     the SX1255, which is the timing reference for the whole RRU)
 *   - sensors for the supervision plane
 *
 * Threading: control functions are called with the core's SPI lock held.
 * rx_read() and tx_write() are called from two different threads (full
 * duplex) and must not block each other.
 */
#ifndef RRU_HW_H
#define RRU_HW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct rru_hw rru_hw_t;

typedef struct {
    double      xtal_hz;        /* SX1255 reference crystal */
    float       pmax_dbm;       /* output power at full TX gain, |x| = 1 */
    const char *spi_dev;        /* e.g. /dev/spidev0.0 */
    const char *pcm_dev;        /* e.g. hw:CARD=sx1255 (ALSA) */
    int         gpio_reset;     /* BCM numbering, -1 = none */
    int         gpio_pa;        /* -1 = none */
    unsigned    seed;           /* dummy: noise generator seed */
    bool        verbose;
} rru_hw_cfg_t;

typedef struct {
    const char *name;
    rru_hw_t *(*open)(const rru_hw_cfg_t *cfg);
    void      (*close)(rru_hw_t *hw);

    /* control plane */
    int  (*chip_reset)(rru_hw_t *hw);               /* RESET high >= 100 us, then wait 5 ms */
    int  (*spi_write)(rru_hw_t *hw, uint8_t addr, uint8_t val);
    int  (*spi_read)(rru_hw_t *hw, uint8_t addr, uint8_t *val);
    int  (*pa_enable)(rru_hw_t *hw, bool on);       /* optional (NULL) */

    /* data plane: CS16 interleaved I/Q at the converter rate */
    int  (*rx_open)(rru_hw_t *hw, uint32_t rate);
    long (*rx_read)(rru_hw_t *hw, int16_t *iq, size_t n);           /* blocks */
    void (*rx_close)(rru_hw_t *hw);
    int  (*tx_open)(rru_hw_t *hw, uint32_t rate);
    long (*tx_write)(rru_hw_t *hw, const int16_t *iq, size_t n);    /* blocks */
    void (*tx_close)(rru_hw_t *hw);

    /* supervision: CARI quantity IDs (Table 20). RF quantities refer to the
     * single subdevice. Return 0 on success. */
    int  (*sensor)(rru_hw_t *hw, uint8_t qid, float *val);
} rru_hw_ops_t;

extern const rru_hw_ops_t rru_hw_dummy;
#ifdef RRU_HAVE_HW_RPI
extern const rru_hw_ops_t rru_hw_rpi;
#endif

#endif
