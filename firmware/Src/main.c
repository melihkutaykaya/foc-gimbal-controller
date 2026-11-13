#include "foc_core.h"
#include "system_errors.h"
#include "hardware_hal.h"
#include "autotune.h"
#include <stdio.h>
#include <math.h>

// PID gains — adjusted live via UART (q/a=Kp±1, w/s=Ki±0.1, e/d=Kd±0.1)
// volatile: USART2_IRQHandler may write at any time; prevents stale register cache under -O2/-O3
volatile float Kp_gimbal = 8.0f;
volatile float Ki_gimbal = 0.30f;
volatile float Kd_gimbal = 0.30f;

// Live PID tuning via UART receive interrupt
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char c = USART2->DR;
        if (c == 'q') Kp_gimbal += 1.0f;
        if (c == 'a') Kp_gimbal -= 1.0f;
        if (c == 'w') Ki_gimbal += 0.1f;
        if (c == 's') Ki_gimbal -= 0.1f;
        if (c == 'e') Kd_gimbal += 0.1f;
        if (c == 'd') Kd_gimbal -= 0.1f;
        if (Kp_gimbal < 0.0f) Kp_gimbal = 0.0f;
        if (Ki_gimbal < 0.0f) Ki_gimbal = 0.0f;
        if (Kd_gimbal < 0.0f) Kd_gimbal = 0.0f;
    }
}

FOCMotor_t   gimbal_motor;
BLDCDriver_t motor_driver;

uint8_t mpu_rx_buffer[8];    // accel_y[2], accel_z[2], skip[2], gyro_x[2]
uint8_t as5600_rx_buffer[2];

// DWT-based loop timing — readable in Keil Watch Window (1 unit ≈ 5.95 ns @ 168 MHz)
volatile uint32_t dbg_loop_cycles = 0;
volatile float    dbg_loop_us     = 0.0f;
volatile uint32_t dbg_loop_max_us = 0;
volatile uint32_t dbg_loop_min_us = 999999u;

// Initialises all peripherals and the motor driver
void System_Setup(void) {
    HAL_SystemClock_Init();  // 168 MHz + SysTick
    HAL_UART2_DMA_Init();    // UART telemetry (DMA, non-blocking)
    HAL_I2C1_Init();         // MPU6050 — 400 kHz
    HAL_I2C2_Init();         // AS5600  — 100 kHz (weak pull-ups require standard mode)
    HAL_GPIO_Button_Init();  // PA0 blue button

    // Enable DWT cycle counter for loop timing (works in production and debug)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_TIM1_PWM_Init(4200); // 20 kHz center-aligned PWM (ARR = 4200 @ 168 MHz)

    // HAL_ADC_Injected_Init() intentionally omitted — PA0 shared with blue button

    MotorDriver_Init(&motor_driver, 12.0f, 12.0f, 4200);

    // Flycat 2805 140KV: 14 poles → pole_pairs = 7
    // Incorrect value produces an impossible field vector and locks the rotor
    FOC_Motor_Init(&gimbal_motor, 7, &motor_driver);
}

int main(void) {
    System_Setup();

    static char telemetry_msg[128];

    // Cable-dependent: flip to +1.0f if motor locks instead of rotating
    float sensor_direction = -1.0f;

    // --- ENCODER CONFIGURATION ---
    snprintf(telemetry_msg, sizeof(telemetry_msg), "Kalibrasyon Basliyor...\r\n");
    HAL_UART2_DMA_Transmit(telemetry_msg);
    for (volatile int i = 0; i < 1000000; i++);

    // AS5600: OUTS=analog, HYST=1LSB, SF=4×, FTH=6LSBs, WD=ON (tested values)
    HAL_I2C2_Write(0x36, 0x07, 0x26);
    HAL_I2C2_Write(0x36, 0x08, 0x04);

    // Magnet health check — STATUS:0x0B (MD=bit5, ML=bit4, MH=bit3), AGC:0x0C
    {
        uint8_t as5600_status = 0;
        uint8_t as5600_agc    = 0;
        HAL_I2C2_Read(0x36, 0x0B, &as5600_status, 1);
        HAL_I2C2_Read(0x36, 0x0C, &as5600_agc,    1);
        bool md = (as5600_status >> 5) & 1;
        bool ml = (as5600_status >> 4) & 1;
        bool mh = (as5600_status >> 3) & 1;
        snprintf(telemetry_msg, sizeof(telemetry_msg),
                 "AS5600: MD=%d ML=%d MH=%d AGC=%d%s\r\n",
                 md, ml, mh, as5600_agc,
                 !md ? " !! MIKNATIZ ALGILANMADI !!" :
                 (ml ? " Zayif miknatiz" : (mh ? " Guclu miknatiz" : " OK")));
        HAL_UART2_DMA_Transmit(telemetry_msg);

        if (!md) Error_Handle(ERR_ENC_NO_MAGNET,    "INIT", 0);
        if (ml)  Error_Handle(ERR_ENC_MAGNET_WEAK,  "INIT", (uint32_t)as5600_agc);
        if (mh)  Error_Handle(ERR_ENC_MAGNET_STRONG,"INIT", (uint32_t)as5600_agc);

        for (volatile int i = 0; i < 500000; i++);
    }

    // MPU6050 wake-up and configuration
    HAL_I2C1_Write(0x68, 0x6B, 0x00);          // disable sleep
    HAL_I2C1_Write(0x68, 0x1A, 0x03);          // DLPF 42 Hz / 4.8 ms delay
    HAL_I2C1_Write(0x68, 0x1B, 0x18);          // gyro ±2000 deg/s (prevents clipping on vibration)
    for (volatile int i = 0; i < 2000000; i++); // 100 ms settle

    float gyro_x_offset = 0.0f;

    // --- D-AXIS ALIGNMENT (FOC zero-angle calibration) ---
    // Apply Ud=8V, Uq=0 to lock rotor on D-axis, then read encoder as the 0-rad reference
    FOC_SetPhaseVoltage(&gimbal_motor, 0.0f, 8.0f, 0.0f);
    for (volatile int i = 0; i < 20000000; i++); // 1 s settle

    int  retry_cnt = 0;
    bool calib_ok  = false;
    while (!calib_ok) {
        if (HAL_I2C2_Read(0x36, 0x0E, as5600_rx_buffer, 2)) {
            calib_ok = true;
        } else {
            retry_cnt++;
            if (retry_cnt > 100) {
                FOC_SetPhaseVoltage(&gimbal_motor, 0.0f, 0.0f, 0.0f);
                Error_Handle(ERR_CALIB_ENC_FAIL, "INIT", (uint32_t)retry_cnt);
            }
            I2C2->CR1 |= I2C_CR1_SWRST;
            for (volatile int i = 0; i < 100; i++);
            I2C2->CR1 &= ~I2C_CR1_SWRST;
            HAL_I2C2_Init();
            for (volatile int i = 0; i < 5000; i++);
        }
    }
    uint16_t calib_raw          = ((as5600_rx_buffer[0] << 8) | as5600_rx_buffer[1]) & 0x0FFF;
    float    calib_physical_rad = sensor_direction * (calib_raw * 6.2831853f) / 4096.0f;

    FOC_SetPhaseVoltage(&gimbal_motor, 0.0f, 0.0f, 0.0f);
    HAL_MotorDriver_Enable(false);

    // --- CONTROL LOOP STATE ---
    float    theta_platform     = 0.0f;
    uint32_t last_time_us       = Sys_Get_Time_Us();
    uint32_t uart_counter       = 0u;
    float    integral_pos_error = 0.0f;
    float    target_voltage     = 0.0f;
    float    physical_angle_rad = 0.0f;
    float    omega_gyro         = 0.0f;

    bool     motor_enabled      = false;
    bool     button_was_pressed = false;
    uint32_t last_button_time   = 0u;

    float    rt_calib_accum = 0.0f;
    uint16_t rt_calib_count = 0u;
    bool     rt_calib_done  = false;

    uint16_t sat_counter      = 0u;
    uint16_t lock_counter     = 0u;
    float    lock_ref_angle   = 0.0f;
    uint8_t  lock_recovery    = 0u;
    bool     windup_active    = false;
    bool     angle_lim_active = false;

    // Block until IMU responds — prevents calibration without sensor
    // and seeds theta_platform from real angle to avoid startup jerk on hot-plug
    while (!HAL_I2C1_Read(0x68, 0x3D, mpu_rx_buffer, 8)) {
        I2C1->CR1 |= I2C_CR1_SWRST;
        for (volatile int i = 0; i < 100; i++);
        I2C1->CR1 &= ~I2C_CR1_SWRST;
        HAL_I2C1_Init();
    }
    {
        int16_t ay = (int16_t)((mpu_rx_buffer[0] << 8) | mpu_rx_buffer[1]);
        int16_t az = (int16_t)((mpu_rx_buffer[2] << 8) | mpu_rx_buffer[3]);
        theta_platform = atan2f((float)ay, (float)az) * 57.2957795f;
    }

    motor_enabled = true;
    HAL_MotorDriver_Enable(true);

    while (1) {
        uint32_t current_time_us = Sys_Get_Time_Us();
        uint32_t elapsed_us      = current_time_us - last_time_us;

        if (elapsed_us >= 1000) {  // 1 kHz control loop
            uint32_t _dwt_start = DWT->CYCCNT;

            float dt = (float)elapsed_us / 1000000.0f;
            last_time_us = current_time_us;

            // 20 ms deadline — 5 ms was too tight (DMA UART block ~7 ms per transmit)
            if (dt > 0.020f) Error_Handle(ERR_CTRL_DEADLINE_MISS, "LOOP", elapsed_us);

            // PA0 blue button — toggle motor with 100 ms debounce
            bool button_state = HAL_GPIO_Button_Read();
            if (button_state && !button_was_pressed &&
                (current_time_us - last_button_time > 100000u)) {
                motor_enabled = !motor_enabled;
                HAL_MotorDriver_Enable(motor_enabled);
                last_button_time = current_time_us;
            }
            button_was_pressed = button_state;

            if (!motor_enabled) integral_pos_error = 0.0f;  // prevent windup while off

            // --- AS5600 ENCODER READ ---
            if (!HAL_I2C2_Read(0x36, 0x0E, as5600_rx_buffer, 2)) {
                // I2C2 lockup (EMI): software reset then re-init
                I2C2->CR1 |= I2C_CR1_SWRST;
                for (volatile int i = 0; i < 100; i++);
                I2C2->CR1 &= ~I2C_CR1_SWRST;
                HAL_I2C2_Init();
                Error_Handle(ERR_ENC_READ_FAIL, "LOOP", 0);
            } else {
                uint16_t raw_angle = ((as5600_rx_buffer[0] << 8) | as5600_rx_buffer[1]) & 0x0FFF;
                float new_angle    = sensor_direction * (raw_angle * 6.2831853f) / 4096.0f;
                new_angle         -= calib_physical_rad;  // offset to D-axis reference

                // No software LPF — even 1 ms lag degrades FOC torque quality
                float candidate_angle = foc_normalize_angle(new_angle);

                // Outlier rejection: >0.2 rad jump in 1 ms is physically impossible
                // reject_count guard prevents lock during sustained fast motion
                static float last_valid_angle = 0.0f;
                static bool  first_read       = true;
                static int   reject_count     = 0;

                if (first_read) {
                    physical_angle_rad = candidate_angle;
                    last_valid_angle   = candidate_angle;
                    first_read         = false;
                } else {
                    float diff = candidate_angle - last_valid_angle;
                    if (diff >  FOC_PI) diff -= 2.0f * FOC_PI;
                    if (diff < -FOC_PI) diff += 2.0f * FOC_PI;

                    if (fabsf(diff) < 0.2f || reject_count > 5) {
                        physical_angle_rad = candidate_angle;
                        last_valid_angle   = candidate_angle;
                        reject_count       = 0;
                    } else {
                        reject_count++;  // I2C noise spike — hold last valid
                    }
                }
            }

            // --- MPU6050 READ AND COMPLEMENTARY FILTER ---
            if (!HAL_I2C1_Read(0x68, 0x3D, mpu_rx_buffer, 8)) {
                // Zero omega on failure — stale value would drive motor continuously
                omega_gyro = 0.0f;
                I2C1->CR1 |= I2C_CR1_SWRST;
                for (volatile int i = 0; i < 100; i++);
                I2C1->CR1 &= ~I2C_CR1_SWRST;
                HAL_I2C1_Init();
                Error_Handle(ERR_IMU_READ_FAIL, "LOOP", 0);
            } else {
                int16_t accel_y = (mpu_rx_buffer[0] << 8) | mpu_rx_buffer[1];
                int16_t accel_z = (mpu_rx_buffer[2] << 8) | mpu_rx_buffer[3];
                int16_t gyro_x  = (mpu_rx_buffer[6] << 8) | mpu_rx_buffer[7];

                float theta_acc = atan2f((float)accel_y, (float)accel_z) * 57.2957795f;

                if (!rt_calib_done) {
                    // First 500 ms: accumulate raw gyro samples to compute DC offset
                    rt_calib_accum += (float)gyro_x;
                    if (++rt_calib_count >= 500u) {
                        gyro_x_offset = rt_calib_accum / 500.0f;
                        rt_calib_done = true;
                    }
                    omega_gyro     = 0.0f;
                    theta_platform = theta_acc;
                } else {
                    omega_gyro = ((float)gyro_x - gyro_x_offset) / 16.4f;
                    // Complementary filter: tau=100 ms, gyro-dominant for fast transients
                    float alpha    = 0.100f / (0.100f + dt);
                    theta_platform = alpha * (theta_platform + omega_gyro * dt)
                                   + (1.0f - alpha) * theta_acc;
                }
            }

            float theta_platform_rad = theta_platform * (FOC_PI / 180.0f);
            float omega_gyro_rad     = omega_gyro     * (FOC_PI / 180.0f);

            float target_imu_angle = 0.0f;
            float pid_direction    = -1.0f;

            // --- SINGLE-LOOP PID ---
            // Snapshot volatile gains once — consistent values for this iteration
            float kp = Kp_gimbal;
            float ki = Ki_gimbal;
            float kd = Kd_gimbal;

            float error_pos = target_imu_angle - theta_platform_rad;
            if (error_pos >  FOC_PI) error_pos -= FOC_2PI;  // shortest-path wrap
            if (error_pos < -FOC_PI) error_pos += FOC_2PI;

            integral_pos_error += error_pos * dt;

            // I-limit 0.5V: compensates gravity/friction without transient overshoot
            float max_integral = 0.5f / (ki + 0.0001f);
            if (integral_pos_error > max_integral) {
                integral_pos_error = max_integral;
                if (!windup_active) { Error_Handle(ERR_CTRL_WINDUP, "PID", 0); windup_active = true; }
            } else if (integral_pos_error < -max_integral) {
                integral_pos_error = -max_integral;
                if (!windup_active) { Error_Handle(ERR_CTRL_WINDUP, "PID", 0); windup_active = true; }
            } else if (fabsf(integral_pos_error) < max_integral * 0.8f) {
                windup_active = false;  // hysteresis — lower flag when integral retreats
            }

            float p_term = pid_direction * kp * error_pos;
            float i_term = pid_direction * ki * integral_pos_error;
            // D uses gyro directly: d/dt(angle) = angular_velocity → zero lag, no noise amplification
            float d_term = pid_direction * kd * (-omega_gyro_rad);

            float raw_voltage = p_term + i_term + d_term;

            // Log sustained saturation (200 consecutive ms above 6.5V)
            if (fabsf(raw_voltage) >= 6.5f) {
                if (++sat_counter >= 200u) { Error_Handle(ERR_MOTOR_VOLTAGE_SAT, "PID", 0); sat_counter = 0u; }
            } else {
                sat_counter = 0u;
            }

            if (raw_voltage >  6.9f) raw_voltage =  6.9f;
            if (raw_voltage < -6.9f) raw_voltage = -6.9f;

            // System ID overrides PID with step excitation voltage
            if (is_system_id_running) {
                target_voltage     = SystemID_Update(physical_angle_rad, omega_gyro, dt);
                integral_pos_error = 0.0f;
            } else {
                target_voltage = raw_voltage;
            }

            // Signed angle (−π..+π) for limit and lock detection
            float signed_pa = physical_angle_rad;
            if (signed_pa > FOC_PI) signed_pa -= FOC_2PI;

            // ±170° hard limit with proportional restoring force
            // Hysteresis: active above 2.967 rad (170°), clears below 2.705 rad (155°)
            if (fabsf(signed_pa) > 2.967296f) {
                float overshoot_deg = fabsf(signed_pa) * 57.2957795f - 170.0f;
                float restore_v     = 4.0f + overshoot_deg * 0.1f;
                if (restore_v > 6.9f) restore_v = 6.9f;
                target_voltage     = (signed_pa > 0.0f) ? -restore_v : restore_v;
                integral_pos_error = 0.0f;
                if (!angle_lim_active) {
                    Error_Handle(ERR_MOTOR_ANGLE_LIMIT, "LOOP", 0);
                    angle_lim_active = true;
                }
            } else if (fabsf(signed_pa) < 2.705260f) {
                angle_lim_active = false;
            }

            // Locked rotor: >4V applied for 500 ms with <0.05 rad movement
            if (fabsf(target_voltage) > 4.0f && motor_enabled) {
                float lock_diff = signed_pa - lock_ref_angle;
                if (fabsf(lock_diff) < 0.05f) {
                    if (++lock_counter >= 500u) {
                        lock_counter = 0u;
                        Error_Handle(ERR_MOTOR_LOCKED_ROTOR, "LOOP", (uint32_t)lock_recovery);
                        if (lock_recovery < 5u) {
                            integral_pos_error = 0.0f;
                            lock_recovery++;
                        } else {
                            // 5 recovery attempts failed — disable motor
                            motor_enabled = false;
                            HAL_MotorDriver_Enable(false);
                            lock_recovery = 0u;
                        }
                    }
                } else {
                    lock_counter   = 0u;
                    lock_ref_angle = signed_pa;
                    lock_recovery  = 0u;
                }
            } else {
                lock_counter   = 0u;
                lock_ref_angle = signed_pa;
            }

            // Suppress output during gyro calibration phase
            if (!rt_calib_done) {
                target_voltage     = 0.0f;
                integral_pos_error = 0.0f;
            }
            if (!motor_enabled) target_voltage = 0.0f;

            // --- FOC OUTPUT ---
            float current_electrical_angle = physical_angle_rad * (float)gimbal_motor.pole_pairs;
            FOC_SetPhaseVoltage(&gimbal_motor, target_voltage, 0.0f, current_electrical_angle);

            // --- TELEMETRY (2 Hz) ---
            if (++uart_counter > 500u) {
                float enc_deg = physical_angle_rad * 57.2957795f;
                if (enc_deg > 180.0f) enc_deg -= 360.0f;  // display as ±180°

                snprintf(telemetry_msg, sizeof(telemetry_msg),
                         "[%.2fs] %-3s  IMU:%+06.1f ENC:%+06.1f V:%+05.2f | P:%.1f I:%.2f D:%.2f | E:%u\r\n",
                         (float)current_time_us / 1000000.0f,
                         !rt_calib_done ? "CAL" : (motor_enabled ? "ON " : "OFF"),
                         theta_platform, enc_deg, target_voltage,
                         Kp_gimbal, Ki_gimbal, Kd_gimbal,
                         (unsigned)Error_Get_Active_Count());
                HAL_UART2_DMA_Transmit(telemetry_msg);

                // Print error detail every 5 s (10 × 500 ms) — avoid log spam
                static uint8_t err_divider = 0u;
                if (Error_Get_Active_Count() > 0u) {
                    if (++err_divider >= 10u) { Error_Print_Telemetry(); err_divider = 0u; }
                } else {
                    err_divider = 0u;
                }

                uart_counter = 0u;
            }

            // --- DWT LOOP TIMING ---
            dbg_loop_cycles = DWT->CYCCNT - _dwt_start;
            dbg_loop_us     = (float)dbg_loop_cycles / 168.0f;
            uint32_t _us    = (uint32_t)dbg_loop_us;
            if (_us > dbg_loop_max_us) dbg_loop_max_us = _us;
            if (_us < dbg_loop_min_us) dbg_loop_min_us = _us;

        }  // end 1 kHz block
    }
}
