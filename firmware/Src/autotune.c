#include "autotune.h"
#include "foc_math.h"
#include <stdio.h>
#include <math.h>

extern void HAL_UART2_DMA_Transmit(const char *msg);

// PID gains defined as volatile in main.c — qualifier must be preserved
extern volatile float Kp_gimbal;
extern volatile float Ki_gimbal;
extern volatile float Kd_gimbal;

#define SYSID_SAMPLES 1000  // 1 s of data at 1 kHz

volatile bool is_system_id_running = false;
static int sysid_index = 0;
static float sysid_timer = 0.0f;

static float sysid_time_data[SYSID_SAMPLES];  // timestamp per sample (s)
static float sysid_gyro_data[SYSID_SAMPLES];  // gyro reading per sample (deg/s)

// Estimate motor K (rad/s/V) and time constant T, then compute PID gains via pole-zero cancellation
void AutoTuner_Calculate(void) {
    // K: average velocity at peak of step response (index 280-300 = 0.28-0.30 s)
    float sum_max_vel = 0.0f;
    for (int i = 280; i < 300; i++) sum_max_vel += fabsf(sysid_gyro_data[i]);
    float max_vel     = sum_max_vel / 20.0f;                    // deg/s
    float max_vel_rad = max_vel * (FOC_PI / 180.0f);            // rad/s
    float K           = max_vel_rad / 2.0f;                     // step was 2.0V
    if (K < 0.1f) K = 0.1f;                                     // guard divide-by-zero

    // T: time to reach 63.2% of peak velocity after step onset (t=0.200 s)
    float target_63 = max_vel * 0.632f;
    float T = 0.018f;
    for (int i = 200; i < 300; i++) {
        if (fabsf(sysid_gyro_data[i]) >= target_63) {
            T = sysid_time_data[i] - 0.200f;
            break;
        }
    }
    if (T < 0.005f) T = 0.005f;
    if (T > 0.100f) T = 0.100f;

    // Pole-zero cancellation: target bandwidth ωc=30 rad/s
    // (reduced from 60 to compensate MPU6050 DLPF 4.8 ms phase delay)
    float omega_c   = 30.0f;
    float Kp_vel    = (omega_c * T) / K;
    float Ki_vel    = Kp_vel / T;
    // Position loop bandwidth ≥10× slower than velocity loop to avoid mechanical shock
    float Kp_pos    = omega_c / 10.0f;

    if (Kp_pos > 8.0f)   Kp_pos = 8.0f;
    if (Kp_vel > 0.4f)   Kp_vel = 0.4f;
    if (Ki_vel > 15.0f)  Ki_vel = 15.0f;

    Kp_gimbal = Kp_pos;
    Ki_gimbal = Ki_vel;
    Kd_gimbal = 0.0f;  // D set to 0: manual gyro-D in main loop would conflict

    static char msg[128];
    HAL_UART2_DMA_Transmit("\r\n--- AUTO-TUNING BASARILI! ---\r\n");
    for (volatile int j = 0; j < 50000; j++);
    snprintf(msg, sizeof(msg), "Sistem K: %.2f rad/s/V, T: %.3fs\r\n", K, T);
    HAL_UART2_DMA_Transmit(msg);
    for (volatile int j = 0; j < 50000; j++);
    snprintf(msg, sizeof(msg), "YENI PID -> Kp: %.2f | Ki: %.2f | Kd: 0.00\r\n", Kp_gimbal, Ki_gimbal);
    HAL_UART2_DMA_Transmit(msg);
}

void SystemID_Start(void) {
    if (is_system_id_running) return;
    
    is_system_id_running = true;
    sysid_index = 0;
    sysid_timer = 0.0f;
    
    HAL_UART2_DMA_Transmit("--- SYSTEM ID BASLIYOR ---\r\nLutfen motoru 1 saniye boyunca BIRAKIN!\r\n");
}

// Step excitation protocol (called every 1 ms):
//   0.00–0.20 s : 0.0V  (sensors settle)
//   0.20–0.30 s : +2.0V (positive acceleration pulse)
//   0.30–0.40 s : -2.0V (braking pulse)
//   0.40–1.00 s : 0.0V  (free decay observation)
float SystemID_Update(float current_angle, float current_gyro, float dt) {
    if (!is_system_id_running) return 0.0f;

    sysid_timer += dt;
    float applied_voltage = 0.0f;

    if      (sysid_timer >= 0.20f && sysid_timer < 0.30f) applied_voltage =  2.0f;
    else if (sysid_timer >= 0.30f && sysid_timer < 0.40f) applied_voltage = -2.0f;

    if (sysid_index < SYSID_SAMPLES) {
        sysid_time_data[sysid_index] = sysid_timer;
        sysid_gyro_data[sysid_index] = current_gyro;
        sysid_index++;
    }

    if (sysid_index >= SYSID_SAMPLES) {
        is_system_id_running = false;
        applied_voltage      = 0.0f;
        AutoTuner_Calculate();
    }

    return applied_voltage;
}
