/*
 * Bit-banged I²C — implementation. See i2c_bitbang.h for the API and the
 * open-drain / bus-timing conventions this module lives by.
 */

#include "i2c_bitbang.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define BB_HALF_US 5   /* ~100 kHz SCL: 5 us per half-cycle */

static inline void bb_high(int pin) { gpio_set_direction(pin, GPIO_MODE_INPUT); }
static inline void bb_low(int pin)
{
    gpio_set_level(pin, 0);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
}

void bb_i2c_init(int sda, int scl)
{
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
    gpio_set_level(sda, 0);
    gpio_set_level(scl, 0);
    bb_high(sda);
    bb_high(scl);
    esp_rom_delay_us(BB_HALF_US * 4);
}

static void bb_start(int sda, int scl)
{
    bb_high(sda); bb_high(scl); esp_rom_delay_us(BB_HALF_US);
    bb_low(sda);                esp_rom_delay_us(BB_HALF_US);
    bb_low(scl);                esp_rom_delay_us(BB_HALF_US);
}

static void bb_stop(int sda, int scl)
{
    bb_low(sda);                esp_rom_delay_us(BB_HALF_US);
    bb_high(scl);               esp_rom_delay_us(BB_HALF_US);
    bb_high(sda);               esp_rom_delay_us(BB_HALF_US);
}

/* SCL must be low on entry; leaves SCL low. */
static void bb_send_bit(int sda, int scl, int bit)
{
    if (bit) bb_high(sda); else bb_low(sda);
    esp_rom_delay_us(BB_HALF_US);
    bb_high(scl);               esp_rom_delay_us(BB_HALF_US);
    bb_low(scl);                esp_rom_delay_us(BB_HALF_US);
}

/* Releases SDA, clocks SCL once, samples SDA. Returns 0 (ACK) or 1 (NACK). */
static int bb_read_bit(int sda, int scl)
{
    bb_high(sda);               esp_rom_delay_us(BB_HALF_US);
    bb_high(scl);               esp_rom_delay_us(BB_HALF_US);
    int bit = gpio_get_level(sda);
    bb_low(scl);                esp_rom_delay_us(BB_HALF_US);
    return bit;
}

/* Sends one byte MSB-first, returns 0 if slave ACKed, 1 if NACK. */
static int bb_send_byte(int sda, int scl, uint8_t byte)
{
    for (int b = 7; b >= 0; --b) {
        bb_send_bit(sda, scl, (byte >> b) & 1);
    }
    return bb_read_bit(sda, scl);
}

bool bb_probe(int sda, int scl, uint8_t addr7)
{
    bb_start(sda, scl);
    int ack = bb_send_byte(sda, scl, (uint8_t)(addr7 << 1));  /* write */
    bb_stop(sda, scl);
    return (ack == 0);
}

bool bb_write(int sda, int scl, uint8_t addr7, const uint8_t *buf, size_t n)
{
    bb_start(sda, scl);
    if (bb_send_byte(sda, scl, (uint8_t)(addr7 << 1)) != 0) {
        bb_stop(sda, scl);
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        if (bb_send_byte(sda, scl, buf[i]) != 0) {
            bb_stop(sda, scl);
            return false;
        }
    }
    bb_stop(sda, scl);
    return true;
}

bool bb_write_reg(int sda, int scl, uint8_t addr7, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return bb_write(sda, scl, addr7, buf, 2);
}

bool bb_read_reg(int sda, int scl, uint8_t addr7, uint8_t reg, uint8_t *out)
{
    bb_start(sda, scl);
    if (bb_send_byte(sda, scl, (uint8_t)(addr7 << 1)) != 0) {
        bb_stop(sda, scl);
        return false;
    }
    if (bb_send_byte(sda, scl, reg) != 0) {
        bb_stop(sda, scl);
        return false;
    }
    /* Repeated START, then addr with R/W=1. */
    bb_start(sda, scl);
    if (bb_send_byte(sda, scl, (uint8_t)((addr7 << 1) | 1)) != 0) {
        bb_stop(sda, scl);
        return false;
    }
    /* Read 8 bits MSB-first, slave drives SDA while master pulses SCL. */
    uint8_t v = 0;
    for (int b = 7; b >= 0; --b) {
        bb_high(sda);                       /* release SDA */
        esp_rom_delay_us(BB_HALF_US);
        bb_high(scl);                       esp_rom_delay_us(BB_HALF_US);
        if (gpio_get_level(sda)) v |= (uint8_t)(1u << b);
        bb_low(scl);                        esp_rom_delay_us(BB_HALF_US);
    }
    /* Master NACK (SDA high during the 9th SCL pulse) to end the read. */
    bb_high(sda);                           esp_rom_delay_us(BB_HALF_US);
    bb_high(scl);                           esp_rom_delay_us(BB_HALF_US);
    bb_low(scl);                            esp_rom_delay_us(BB_HALF_US);
    bb_stop(sda, scl);
    *out = v;
    return true;
}
