/*
 * ES8311 driver — see es8311.h for the API and provenance. Register
 * values here trace to ESP-ADF commit e019b05c…4a2eb73, MIT-licensed,
 * further pared down for the 16 kHz mono ADC-only path.
 */

#include "es8311.h"
#include "pins.h"
#ifndef ES8311_MOCK_TRANSPORT
#include "i2c_bitbang.h"
#endif

/* Registers we touch. Names match ADF's es8311.h for grep-back. */
#define ES8311_RESET_REG00              0x00
#define ES8311_CLK_MANAGER_REG01        0x01
#define ES8311_CLK_MANAGER_REG02        0x02
#define ES8311_CLK_MANAGER_REG03        0x03
#define ES8311_CLK_MANAGER_REG04        0x04
#define ES8311_CLK_MANAGER_REG05        0x05
#define ES8311_CLK_MANAGER_REG06        0x06
#define ES8311_CLK_MANAGER_REG07        0x07
#define ES8311_CLK_MANAGER_REG08        0x08
#define ES8311_SDPIN_REG09              0x09
#define ES8311_SDPOUT_REG0A             0x0A
#define ES8311_SYSTEM_REG0B             0x0B
#define ES8311_SYSTEM_REG0C             0x0C
#define ES8311_SYSTEM_REG0D             0x0D
#define ES8311_SYSTEM_REG0E             0x0E
#define ES8311_SYSTEM_REG10             0x10
#define ES8311_SYSTEM_REG11             0x11
#define ES8311_SYSTEM_REG12             0x12
#define ES8311_SYSTEM_REG13             0x13
#define ES8311_SYSTEM_REG14             0x14
#define ES8311_ADC_REG15                0x15
#define ES8311_ADC_REG16                0x16   /* analog PGA */
#define ES8311_ADC_REG17                0x17   /* ADC digital volume (0xBF = 0 dB) */
#define ES8311_ADC_REG1B                0x1B
#define ES8311_ADC_REG1C                0x1C
#define ES8311_DAC_REG37                0x37   /* DAC ramp rate (unused for ADC path) */
#define ES8311_GPIO_REG44               0x44
#define ES8311_GP_REG45                 0x45
#define ES8311_CHD1_REGFD               0xFD   /* chip ID, expect 0x83 */
#define ES8311_CHD2_REGFE               0xFE   /* chip version */

/*
 * bb_write_reg / bb_read_reg are the transport by default. The host test
 * (test/host/es8311_init_test.c) redefines these to logging mocks BEFORE
 * including this file so it can capture the register writes without a
 * real bus. The macro-guarded declarations keep both paths clean.
 */
#ifndef ES8311_MOCK_TRANSPORT
#include <stdbool.h>
static inline bool w(uint8_t reg, uint8_t val)
{
    return bb_write_reg(ES8311_I2C_SDA, ES8311_I2C_SCL, ES8311_I2C_ADDR, reg, val);
}
static inline bool r(uint8_t reg, uint8_t *out)
{
    return bb_read_reg(ES8311_I2C_SDA, ES8311_I2C_SCL, ES8311_I2C_ADDR, reg, out);
}
#else
extern int mock_bb_write_reg(int sda, int scl, uint8_t addr, uint8_t reg, uint8_t val);
extern int mock_bb_read_reg (int sda, int scl, uint8_t addr, uint8_t reg, uint8_t *out);
static inline int w(uint8_t reg, uint8_t val)
{
    return mock_bb_write_reg(0, 0, ES8311_I2C_ADDR, reg, val) == 0;
}
static inline int r(uint8_t reg, uint8_t *out)
{
    return mock_bb_read_reg(0, 0, ES8311_I2C_ADDR, reg, out) == 0;
}
#endif

#define TRY(expr) do { if (!(expr)) return ESP_FAIL; } while (0)

esp_err_t es8311_init(void)
{
    /* --- Reset ------------------------------------------------------------
     * REG00 = 0x1F asserts every reset bit; = 0x00 releases them. Not
     * strictly in ADF's init (ADF assumes the chip is fresh out of
     * power-on reset), but the plan test contract puts this pair first
     * and it also makes es8311_init idempotent across re-runs. */
    TRY(w(ES8311_RESET_REG00, 0x1F));
    TRY(w(ES8311_RESET_REG00, 0x00));

    /* --- Init mirrors ADF es8311_codec_init verbatim -------------------- */

    /* I²C noise immunity, per ADF: "Due to occasional failures during
     * the first I²C write with the ES8311 chip, a second write is
     * performed to ensure reliability." */
    TRY(w(ES8311_GPIO_REG44, 0x08));
    TRY(w(ES8311_GPIO_REG44, 0x08));

    /* Clock scheme: enable clocks (REG01), zero prescaler (REG02),
     * ADC fs-mode+osr baseline (REG03), analog PGA baseline (REG16),
     * DAC osr baseline (REG04), ADC/DAC divider (REG05). REG02/03/04/05
     * get their final 16 kHz @ 4.096 MHz values below via the
     * config_sample recipe. */
    TRY(w(ES8311_CLK_MANAGER_REG01, 0x30));
    TRY(w(ES8311_CLK_MANAGER_REG02, 0x00));
    TRY(w(ES8311_CLK_MANAGER_REG03, 0x10));
    TRY(w(ES8311_ADC_REG16,         0x24));
    TRY(w(ES8311_CLK_MANAGER_REG04, 0x10));
    TRY(w(ES8311_CLK_MANAGER_REG05, 0x00));

    /* Analog domain defaults (charge pump / VMID / bias). */
    TRY(w(ES8311_SYSTEM_REG0B, 0x00));
    TRY(w(ES8311_SYSTEM_REG0C, 0x00));
    TRY(w(ES8311_SYSTEM_REG10, 0x1F));
    TRY(w(ES8311_SYSTEM_REG11, 0x7F));

    /* CSM power-on — bit 7. Bit 6 = 0 → slave (we're the I²S slave).
     * Without this the ADC digital output is frozen at zero. */
    TRY(w(ES8311_RESET_REG00, 0x80));

    /* Enable all internal clocks (REG01 = 0x3F), MCLK from MCLK pin
     * (bit 7 stays 0), MCLK not inverted (bit 6 stays 0). */
    TRY(w(ES8311_CLK_MANAGER_REG01, 0x3F));

    /* --- config_sample(16000) equivalent, MCLK = 4.096 MHz --------------
     * ADF coeff_div row {mclk=4096000, rate=16000}: pre_div=1, pre_multi=1,
     * adc_div=1, dac_div=1, fs_mode=0, lrck_h=0, lrck_l=0xff, bclk_div=4,
     * adc_osr=0x10, dac_osr=0x20. Values here are the bit-packed form
     * ADF's config_sample would produce for that row. */
    TRY(w(ES8311_CLK_MANAGER_REG02, 0x00));   /* pre_div-1=0 << 5, pre_multi=0 << 3 */
    TRY(w(ES8311_CLK_MANAGER_REG05, 0x00));   /* adc_div-1=0 << 4, dac_div-1=0 */
    TRY(w(ES8311_CLK_MANAGER_REG03, 0x10));   /* fs_mode=0 << 6, adc_osr=0x10 */
    TRY(w(ES8311_CLK_MANAGER_REG04, 0x20));   /* dac_osr=0x20 */
    TRY(w(ES8311_CLK_MANAGER_REG07, 0x00));   /* lrck_h=0 */
    TRY(w(ES8311_CLK_MANAGER_REG08, 0xFF));   /* lrck_l=0xff */
    TRY(w(ES8311_CLK_MANAGER_REG06, 0x03));   /* bclk_div=4 → (4-1) */

    /* Serial data ports: 16-bit width on both, I²S normal format. */
    TRY(w(ES8311_SDPIN_REG09,  0x0C));
    TRY(w(ES8311_SDPOUT_REG0A, 0x0C));

    /* Post-config housekeeping. */
    TRY(w(ES8311_SYSTEM_REG13, 0x10));
    TRY(w(ES8311_ADC_REG1B,    0x0A));
    TRY(w(ES8311_ADC_REG1C,    0x6A));

    /* --- ADC start (folded from ADF es8311_start(ES_MODULE_ADC)) -------
     * Order matches ADF exactly: SDP unmute, ADC volume, per-block enables,
     * PGA gain / DMIC deselect (analog mic), system power-up ramp, DAC
     * ramp (harmless for ADC-only), GP0 clear, internal-reference route. */
    TRY(w(ES8311_SDPIN_REG09,  0x4C));       /* DAC SDP: bit 6 = 1 → mute (unused) */
    TRY(w(ES8311_SDPOUT_REG0A, 0x0C));       /* ADC SDP: bit 6 = 0 → unmute */

    TRY(w(ES8311_ADC_REG17,    0xBF));       /* ADC digital volume 0 dB, unmuted */
    TRY(w(ES8311_SYSTEM_REG0E, 0x02));       /* enable ADC block */
    TRY(w(ES8311_SYSTEM_REG12, 0x00));
    TRY(w(ES8311_SYSTEM_REG14, 0x1A));       /* DMIC=0 (analog), PGA gain select */
    TRY(w(ES8311_SYSTEM_REG0D, 0x01));       /* system power up */
    TRY(w(ES8311_ADC_REG15,    0x40));       /* ADC ramp rate */
    TRY(w(ES8311_DAC_REG37,    0x08));       /* DAC ramp (ADF sets even for ADC-only) */
    TRY(w(ES8311_GP_REG45,     0x00));
    TRY(w(ES8311_GPIO_REG44,   0x58));       /* internal reference: ADCL + DACR */

    return ESP_OK;
}

esp_err_t es8311_set_mic_gain_db(int gain_db)
{
    /* ES8311 PGA has 6 dB steps from 0 dB to 42 dB → enum 0..7. Round to
     * the nearest step; clip out-of-range to the endpoints. Negative
     * input clips to 0 dB (no attenuation on this codec). */
    int step = (gain_db + 3) / 6;
    if (step < 0) step = 0;
    if (step > 7) step = 7;
    return w(ES8311_ADC_REG16, (uint8_t)step) ? ESP_OK : ESP_FAIL;
}

esp_err_t es8311_read_id(uint8_t *id_out, uint8_t *ver_out)
{
    if (!id_out || !ver_out) return ESP_ERR_INVALID_ARG;
    if (!r(ES8311_CHD1_REGFD, id_out))  return ESP_FAIL;
    if (!r(ES8311_CHD2_REGFE, ver_out)) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t es8311_start(void)
{
    /* es8311_init already leaves the ADC unmuted (REG17=0xBF, REG0A bit 6
     * cleared). Kept as a no-op so callers can start/stop symmetrically. */
    return ESP_OK;
}

esp_err_t es8311_stop(void)
{
    /* Mute the ADC SDPOUT (bit 6 of REG0A = 1). Clock config is preserved
     * so es8311_start can un-mute without touching the clock tree. */
    uint8_t v = 0;
    if (!r(ES8311_SDPOUT_REG0A, &v)) return ESP_FAIL;
    return w(ES8311_SDPOUT_REG0A, (uint8_t)(v | 0x40)) ? ESP_OK : ESP_FAIL;
}
