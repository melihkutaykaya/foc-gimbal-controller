#include "foc_core.h"

void FOC_Motor_Init(FOCMotor_t *motor, int pole_pairs, BLDCDriver_t *drv) {
    motor->pole_pairs      = pole_pairs;
    motor->driver          = drv;
    motor->voltage_limit   = drv->voltage_limit;
    motor->electrical_angle = 0.0f;
}

void FOC_SetPhaseVoltage(FOCMotor_t *motor, float Uq, float Ud, float angle_el) {
    float Ualpha, Ubeta;
    float Ua, Ub, Uc;
    
    motor->electrical_angle = angle_el;
    
    foc_inverse_park(Uq, Ud, angle_el, &Ualpha, &Ubeta);
    foc_inverse_clarke(Ualpha, Ubeta, &Ua, &Ub, &Uc);
    foc_svm_midpoint_clamp(&Ua, &Ub, &Uc, motor->voltage_limit);
    MotorDriver_SetPwm(motor->driver, Ua, Ub, Uc);
}

