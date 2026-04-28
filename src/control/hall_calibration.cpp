#include "control/hall_calibration.h"
#include "control/motion_control.h"
#include "hardware/adc_dma.h"
#include "storage/nvm_storage.h"
#include "platform/hal/time_hw.h"
#include "platform/debug_log.h"
#include "platform/runtime_api.h"

extern void RGB_update();

static inline float adc_pull_raw_ch(int ch, const float *v8)
{
    switch (ch)
    {
    case 0: return (float)v8[6];
    case 1: return (float)v8[4];
    case 2: return (float)v8[2];
    default:return (float)v8[0];
    }
}

static inline float adc_key_raw_ch(int ch, const float *v8)
{
    switch (ch)
    {
    case 0: return (float)v8[7];
    case 1: return (float)v8[5];
    case 2: return (float)v8[3];
    default:return (float)v8[1];
    }
}

static inline uint8_t dm_key_round_up_to_centi(float v)
{
    if (v <= 0.0f) return 0u;

    float x = v * 100.0f - 0.0001f;
    int iv = (int)x;
    if ((float)iv < x) iv++;

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

static inline float dm_key_none_threshold_from_idle(float key_value)
{
    const uint8_t key_cv = dm_key_round_up_to_centi(key_value);

    uint8_t thr_cv = (uint8_t)(key_cv + 10u);
    if (thr_cv < 60u) thr_cv = 60u;
    if (thr_cv > 139u) thr_cv = 139u;

    return 0.01f * (float)thr_cv;
}

static inline float adc_pull_v_centered(int ch)
{
    const float *d = ADC_DMA_get_value();
    return adc_pull_raw_ch(ch, d) + MC_PULL_V_OFFSET[ch];
}

static inline float cal_apply_polarity(float v, int8_t pol)
{
    return (pol < 0) ? (3.30f - v) : v;
}

static void blink_all(uint8_t r, uint8_t g, uint8_t b, int times = 4, int on_ms = 60, int off_ms = 60)
{
    for (int k = 0; k < times; k++)
    {
        for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, r, g, b);
        RGB_update(); delay(on_ms);
        for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update(); delay(off_ms);
    }
}

void MC_PULL_calibration_clear()
{
    Flash_NVM_full_clear();
}

void MC_PULL_calibration_boot()
{
    for (int i = 0; i < 6; i++) { ADC_DMA_poll(); delay(20); }

    MC_PULL_detect_channels_inserted();

    float offs[4], vmin[4], vmax[4];
    int8_t pol[4];
    if (Flash_MC_PULL_cal_read(offs, vmin, vmax, pol))
    {
        for (int ch = 0; ch < 4; ch++)
        {
            MC_PULL_V_OFFSET[ch] = offs[ch];
            MC_PULL_V_MIN[ch]    = vmin[ch];
            MC_PULL_V_MAX[ch]    = vmax[ch];
            MC_PULL_POLARITY[ch] = (pol[ch] < 0) ? -1 : 1;
        }
        return;
    }

    const bool ok_wipe = Flash_NVM_full_clear();

    double sum_raw[4] = {0, 0, 0, 0};
    double sum_key[4] = {0, 0, 0, 0};
    const int N = 90;
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();

    for (int k = 0; k < N; k++)
    {
        const float *v = ADC_DMA_get_value();

        for (int ch = 0; ch < 4; ch++)
        {
            if (!filament_channel_inserted[ch]) continue;
            sum_raw[ch] += adc_pull_raw_ch(ch, v);
            sum_key[ch] += adc_key_raw_ch(ch, v);
        }

        const uint32_t now_t = time_ticks32();
        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        bool on = (((elapsed_ms / 200u) & 1u) == 0u);
        for (int ch = 0; ch < 4; ch++)
            MC_PULL_ONLINE_RGB_set(ch, on ? 0x10 : 0, on ? 0x10 : 0, 0x00);
        RGB_update();
        delay(15);
    }

    float center_raw[4] = {1.65f, 1.65f, 1.65f, 1.65f};
    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch]) continue;
        center_raw[ch] = (float)(sum_raw[ch] / (double)N);
    }

    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch])
        {
            MC_PULL_V_OFFSET[ch] = 0.0f;
            MC_PULL_POLARITY[ch] = 1;
            MC_DM_KEY_NONE_THRESH[ch] = 0.60f;
            continue;
        }

        MC_PULL_V_OFFSET[ch] = 1.65f - center_raw[ch];
        MC_PULL_POLARITY[ch] = 1;
        MC_DM_KEY_NONE_THRESH[ch] =
            dm_key_none_threshold_from_idle((float)(sum_key[ch] / (double)N));
    }

    float center_v_ref[4] = {1.65f, 1.65f, 1.65f, 1.65f};
    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch]) continue;
        center_v_ref[ch] = center_raw[ch] + MC_PULL_V_OFFSET[ch];
    }

    // Offline standalone mode:
    // avoid blocking interactive Hall calibration at boot.
    // We keep passive center calibration and store generic travel defaults.
    (void)center_v_ref;
    for (int ch = 0; ch < 4; ch++)
    {
        MC_PULL_V_MIN[ch] = 1.55f;
        MC_PULL_V_MAX[ch] = 1.75f;

        if (!filament_channel_inserted[ch])
        {
            MC_PULL_POLARITY[ch] = 1;
            continue;
        }

        // Hall polarity auto-detect is skipped in offline mode.
        // Default to the common polarity and keep runtime behavior silent.
        MC_PULL_POLARITY[ch] = 1;
    }

    const bool ok_cal = Flash_MC_PULL_cal_write_all(MC_PULL_V_OFFSET, MC_PULL_V_MIN, MC_PULL_V_MAX, MC_PULL_POLARITY);
    const bool ok_mot = Motion_control_save_dm_key_none_thresholds();
    const bool ok = ok_wipe && ok_cal && ok_mot;

    if (ok) blink_all(0x00, 0x10, 0x00, 1, 100, 40);
    else    blink_all(0x10, 0x00, 0x00, 2, 180, 120);
}
