#pragma once

#include <stdint.h>

namespace buffer_constants {

namespace geometry {
constexpr uint8_t channel_count = 4u;
constexpr int pwm_limit = 1000;
constexpr float as5600_pi = 3.14159265358979323846f;
constexpr float as5600_wheel_diameter_mm = 7.5f;
constexpr float as5600_counts_per_turn = 4096.0f;
}

namespace retract_curve {
constexpr float deadband_err = 0.10f;
constexpr float low_err = 0.35f;
constexpr float high_err = 2.35f;
constexpr float low_pwm_start = 450.0f;
constexpr float low_pwm_end = 550.0f;
constexpr float high_pwm_end = 850.0f;
}

namespace key_state {
constexpr float none_threshold_volts = 0.60f;
constexpr float first_switch_threshold_volts = 1.40f;
constexpr float loaded_threshold_volts = 1.70f;
}

namespace as5600_health {
constexpr uint8_t fail_trip = 3u;
constexpr uint8_t ok_recover = 2u;
}

namespace pullback {
constexpr float speed_fast_mm_s = 60.0f;
constexpr float speed_end_mm_s = 12.0f;
constexpr float ramp_m = 0.015f;
constexpr float pwm_min = 400.0f;
}

namespace dm_autoload {
constexpr uint64_t stage1_debounce_ms = 100ull;
constexpr uint64_t stage1_timeout_ms = 5000ull;
constexpr uint64_t stage1_fail_retract_ms = 1500ull;
constexpr float stage2_target_m = 0.120f;
constexpr float buffer_abort_pct = 75.0f;
constexpr float buffer_recover_pct = 50.2f;
constexpr uint64_t fail_extra_ms = 1500ull;
constexpr float pwm_push = 900.0f;
constexpr float pwm_pull = 900.0f;
constexpr float idle_pwm_limit = 950.0f;
}

namespace auto_unload {
constexpr float start_pct = 80.0f;
constexpr float neutral_lo_pct = 45.0f;
constexpr float neutral_hi_pct = 55.0f;
constexpr float abort_pct = 35.0f;
constexpr uint64_t arm_ms = 1000ull;
constexpr uint64_t max_ms = 15000ull;
constexpr uint64_t empty_ms = 1500ull;
constexpr float pwm_pull = 850.0f;
}

namespace standalone {
constexpr uint64_t autoload_debounce_ms = 80ull;
constexpr uint64_t autoload_max_ms = 6000ull;

constexpr float manual_feed_start_pct = 30.0f;
constexpr float manual_feed_release_pct = 40.0f;
constexpr float manual_retract_start_pct = 70.0f;
constexpr float manual_retract_release_pct = 60.0f;
constexpr uint64_t manual_hold_ms = 1000ull;

constexpr float autoload_pwm_push_default = 850.0f;
constexpr float autoload_pwm_push_high_force = 960.0f;
constexpr float manual_feed_pwm_default = autoload_pwm_push_default;
constexpr float manual_feed_pwm_high_force = autoload_pwm_push_high_force;
constexpr float manual_retract_pwm_default = 850.0f;
constexpr float manual_retract_pwm_high_force = 900.0f;
}

namespace load_control {
constexpr float pidp_pct = 25.0f;
constexpr int deadband_pct_low = 30;
constexpr int deadband_pct_high = 70;
constexpr int stage1_fast_pct = 85;
constexpr int stage1_hard_stop_pct = 95;
constexpr int stage1_hard_hys = 2;
constexpr float hold_target_pct = 90.0f;
constexpr float hold_band_lo_delta = 0.3f;
constexpr float push_start_pct = 80.0f;
constexpr float pwm_hi = 480.0f;
constexpr float pwm_lo = 1000.0f;
constexpr float on_use_target_pct = 52.0f;
constexpr float on_use_band_lo_delta = 0.2f;
constexpr float on_use_band_hi_pct = 60.0f;
constexpr float on_use_hold_target_pct = 50.0f;
constexpr float on_use_pwm_lo = 380.0f;
constexpr float on_use_fast_pct = 50.0f;
constexpr float on_use_fast_pwm = 900.0f;
constexpr float on_use_pwm_cap = 900.0f;
constexpr float on_use_retrigger_pct = 55.0f;
constexpr float on_use_retrigger_pwm_cap = 950.0f;
constexpr float stop_pwm_limit = 800.0f;
constexpr uint64_t send_softstart_ms = 300ull;
constexpr float send_softstart_v0 = 10.0f;
constexpr float send_softstart_v = 60.0f;
constexpr float send_max_m = 10.0f;
}

namespace calibration_reset {
constexpr uint32_t hold_ms = 5000u;
constexpr int pct_thresh = 15;
constexpr float v_delta = 0.10f;
constexpr float near_min = 0.03f;
}

}  // namespace buffer_constants
