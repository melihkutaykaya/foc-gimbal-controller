#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

// DRV8313 (3-PWM) motor sürücü ayarları
typedef struct {
    float voltage_power_supply; // Sistemin besleme voltajı (VBUS, örn: 12.0V)
    float voltage_limit;        // Sürücünün verebileceği max voltaj (örn: 12.0V)
    
    uint16_t pwm_period_arr;    // Timer'ın Auto-Reload Register (ARR) değeri
} BLDCDriver_t;

// Initialise driver struct; clamps voltage_limit to voltage_power_supply if needed
void MotorDriver_Init(BLDCDriver_t *driver, float v_supply, float v_limit, uint16_t timer_arr);

// Apply phase voltages Ua/Ub/Uc to TIM1 CCR registers (clamps to 0..voltage_limit)
void MotorDriver_SetPwm(BLDCDriver_t *driver, float Ua, float Ub, float Uc);

#endif // MOTOR_DRIVER_H
