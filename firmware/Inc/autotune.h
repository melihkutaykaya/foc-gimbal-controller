#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <stdint.h>
#include <stdbool.h>

extern volatile bool is_system_id_running;

// Arm a new system-identification run (ignored if one is already active)
void SystemID_Start(void);

// Call at 1 kHz while is_system_id_running is true.
// Returns the step excitation voltage to apply to the motor.
// current_angle: rotor angle (rad), current_gyro: angular velocity (deg/s)
float SystemID_Update(float current_angle, float current_gyro, float dt);

#endif // AUTOTUNE_H
