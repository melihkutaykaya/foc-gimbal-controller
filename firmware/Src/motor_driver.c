#include "motor_driver.h"
#include "foc_math.h" // FOC_CONSTRAIN için
#include "stm32f4xx.h" // TIM1 ve donanım tanımlamaları için

void MotorDriver_Init(BLDCDriver_t *driver, float v_supply, float v_limit, uint16_t timer_arr) {
    driver->voltage_power_supply = v_supply;
    driver->voltage_limit = v_limit;
    driver->pwm_period_arr = timer_arr;
    
    if (driver->voltage_limit > driver->voltage_power_supply) {
        driver->voltage_limit = driver->voltage_power_supply;
    }
}

// Convert phase voltages to TIM1 duty cycles and write CCR registers
void MotorDriver_SetPwm(BLDCDriver_t *driver, float Ua, float Ub, float Uc) {
    Ua = FOC_CONSTRAIN(Ua, 0.0f, driver->voltage_limit);
    Ub = FOC_CONSTRAIN(Ub, 0.0f, driver->voltage_limit);
    Uc = FOC_CONSTRAIN(Uc, 0.0f, driver->voltage_limit);

    TIM1->CCR1 = (uint16_t)((Ua / driver->voltage_power_supply) * driver->pwm_period_arr);
    TIM1->CCR2 = (uint16_t)((Ub / driver->voltage_power_supply) * driver->pwm_period_arr);
    TIM1->CCR3 = (uint16_t)((Uc / driver->voltage_power_supply) * driver->pwm_period_arr);
}
