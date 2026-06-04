#pragma once
#include <stdint.h>

struct HallCalibrationResult
{
    uint8_t channel;
    bool inserted;
    bool ok_first;
    bool ok_second;
    bool ok_cal_save;
    bool ok_motion_save;
    float center_raw;
    float center_v;
    float offset;
    float vmin;
    float vmax;
    int8_t polarity;
    float key_idle;
    float key_none_threshold;
};

void MC_PULL_calibration_boot();
void MC_PULL_calibration_clear();
bool MC_PULL_calibration_channel(uint8_t channel_id, HallCalibrationResult *result);
