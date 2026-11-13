#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>
#include <math.h>

// To use CMSIS-DSP: add CMSIS>DSP in RTE, define ARM_MATH_CM4, include arm_math.h
// #include "arm_math.h"

#define FOC_PI      3.14159265359f
#define FOC_2PI     6.28318530718f
#define FOC_SQRT3   1.73205080757f
#define FOC_SQRT3_2 0.86602540378f

// Three-phase voltage vector
typedef struct { float a; float b; float c; } ABC_t;

// Clamp value to [low, high]
#define FOC_CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Wrap angle to [0, 2π)
static __inline float foc_normalize_angle(float angle) {
    float a = fmodf(angle, FOC_2PI);
    return a >= 0 ? a : (a + FOC_2PI);
}

// sinf/cosf — replace with arm_sin/cos_f32 for CMSIS-DSP acceleration
static __inline void foc_sincos(float angle, float *s, float *c) {
    *s = sinf(angle);
    *c = cosf(angle);
}

// Inverse Park: rotating d-q frame → stationary α-β frame
void foc_inverse_park(float Uq, float Ud, float angle_el, float *Ualpha, float *Ubeta);

// Inverse Clarke: α-β → three-phase Ua/Ub/Uc
void foc_inverse_clarke(float Ualpha, float Ubeta, float *Ua, float *Ub, float *Uc);

// SVPWM midpoint clamp: shift all phases by (limit/2 − midpoint) for DC utilisation
void foc_svm_midpoint_clamp(float *Ua, float *Ub, float *Uc, float limit);

#endif // FOC_MATH_H
