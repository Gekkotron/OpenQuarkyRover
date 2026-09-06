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
     * REG 0x00: writing 0x1F asserts CSM + digital + clock resets; 0x00
     * releases all reset bits and leaves the codec powered but idle.
     * Datasheet §Register 0x00 — the plan test encodes this pair as the
     * first two writes and it doubles as a "start from a known state"
     * guarantee across re-inits. */
    TRY(w(ES8311_RESET_REG00, 0x1F));
    TRY(w(ES8311_RESET_REG00, 0x00));

    /* --- Clock configuration ---------------------------------------------
     * Target: MCLK = 4.096 MHz (I2S_MCLK_MULTIPLE_256 × 16 kHz), sample
     * rate 16 kHz, slave mode, MCLK from the dedicated MCLK pin. Values
     * come from ADF's coeff_div[] row {mclk=4096000, rate=16000}: pre_div
     * pre_multi adc_div dac_div fs_mode lrck_h lrck_l bclk_div adc_osr
     * dac_osr = 1 1 1 1 0 0 0xff 4 0x10 0x20. Each REG* below is the
     * bit-packed form of that row per ADF es8311_config_sample(). */
    TRY(w(ES8311_CLK_MANAGER_REG02, 0x00));   /* pre_div-1=0, pre_multi=0 → x1 */
    TRY(w(ES8311_CLK_MANAGER_REG03, 0x10));   /* fs_mode=0, adc_osr=0x10 */
    TRY(w(ES8311_CLK_MANAGER_REG04, 0x20));   /* dac_osr=0x20 */
    TRY(w(ES8311_CLK_MANAGER_REG05, 0x00));   /* adc_div-1=0, dac_div-1=0 */
    TRY(w(ES8311_CLK_MANAGER_REG06, 0x03));   /* bclk_div=4 → (4-1)=3 */
    TRY(w(ES8311_CLK_MANAGER_REG07, 0x00));   /* lrck_h=0 */
    TRY(w(ES8311_CLK_MANAGER_REG08, 0xFF));   /* lrck_l=0xff */

    /* Enable all internal clocks; leave bit 7 = 0 (MCLK from MCLK_PIN). */
    TRY(w(ES8311_CLK_MANAGER_REG01, 0x3F));

    /* Slave-mode audio interface: REG00 bit 6 = 0 (default after reset). */

    /* --- Serial digital ports --------------------------------------------
     * 16-bit width on both DAC (REG09) and ADC (REG0A) SDP so the same
     * driver is future-proof for M4 duplex, and I²S "normal" format
     * (LJ bit cleared). ADF: es8311_set_bits_per_sample + config_fmt. */
    TRY(w(ES8311_SDPIN_REG09,  0x0C));
    TRY(w(ES8311_SDPOUT_REG0A, 0x0C));

    /* --- System register defaults from ADF init --------------------------
     * These configure charge pump, VMID, reference, etc. Copying ADF's
     * exact sequence is safer than guessing — the datasheet §"System
     * Register" section documents each bit but the interactions are
     * subtle enough that the tested-in-production combo wins. */
    TRY(w(ES8311_SYSTEM_REG0B, 0x00));
    TRY(w(ES8311_SYSTEM_REG0C, 0x00));
    TRY(w(ES8311_SYSTEM_REG10, 0x1F));
    TRY(w(ES8311_SYSTEM_REG11, 0x7F));
    TRY(w(ES8311_SYSTEM_REG13, 0x10));
    TRY(w(ES8311_ADC_REG1B,    0x0A));
    TRY(w(ES8311_ADC_REG1C,    0x6A));

    /* Default mic PGA to 0 dB. Writes REG16 exactly once. */
    (void)es8311_set_mic_gain_db(0);

    /* --- ADC path power-up (folded from ADF es8311_start) ---------------
     * REG17 = 0xBF: ADC digital volume 0 dB, unmuted (bit 0 = 1 — the
     * plan test checks that bit specifically as "ADC enabled"). REG0E /
     * REG12 / REG14 configure PGA/DMIC selection; REG0D powers the
     * system rails; REG15 sets ADC ramp; REG44 = 0x58 selects the
     * internal reference (ADCL + DACR). */
    TRY(w(ES8311_ADC_REG17,   0xBF));
    TRY(w(ES8311_SYSTEM_REG0E, 0x02));
    TRY(w(ES8311_SYSTEM_REG12, 0x00));
    TRY(w(ES8311_SYSTEM_REG14, 0x1A));   /* analog PGA + mic input */
    TRY(w(ES8311_SYSTEM_REG0D, 0x01));   /* system power up */
    TRY(w(ES8311_ADC_REG15,    0x40));   /* ADC ramp rate */
    TRY(w(ES8311_GP_REG45,     0x00));
    TRY(w(ES8311_GPIO_REG44,   0x58));   /* internal reference: ADCL + DACR */

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
