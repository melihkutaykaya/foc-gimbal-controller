#ifndef FOC_CORE_H
#define FOC_CORE_H

#include <stdint.h>
#include "foc_math.h"
#include "motor_driver.h"

// FOC motor state (voltage-mode, open-loop torque)
typedef struct {
    int          pole_pairs;
    float        voltage_limit;
    float        electrical_angle;  // last commanded angle (radians)
    BLDCDriver_t *driver;
} FOCMotor_t;

// Initialise motor struct from driver settings
void FOC_Motor_Init(FOCMotor_t *motor, int pole_pairs, BLDCDriver_t *drv);

// Apply Uq/Ud voltages via inverse Park → Clarke → SVPWM → CCR
void FOC_SetPhaseVoltage(FOCMotor_t *motor, float Uq, float Ud, float angle_el);

#endif // FOC_CORE_H
