#include "foc_math.h"

// Inverse Park: d-q rotating frame → α-β stationary frame
void foc_inverse_park(float Uq, float Ud, float angle_el, float *Ualpha, float *Ubeta) {
    float sa, ca;
    foc_sincos(angle_el, &sa, &ca);
    
    *Ualpha = ca * Ud - sa * Uq;
    *Ubeta  = sa * Ud + ca * Uq;
}

// Inverse Clarke: α-β stationary frame → three-phase Ua/Ub/Uc
void foc_inverse_clarke(float Ualpha, float Ubeta, float *Ua, float *Ub, float *Uc) {
    *Ua = Ualpha;
    *Ub = -0.5f * Ualpha + FOC_SQRT3_2 * Ubeta;
    *Uc = -0.5f * Ualpha - FOC_SQRT3_2 * Ubeta;
}

// SVPWM midpoint clamp: shift all phases by (limit/2 − midpoint) to maximise DC bus utilisation
void foc_svm_midpoint_clamp(float *Ua, float *Ub, float *Uc, float limit) {
    float Umin = *Ua;
    if (*Ub < Umin) Umin = *Ub;
    if (*Uc < Umin) Umin = *Uc;
    
    float Umax = *Ua;
    if (*Ub > Umax) Umax = *Ub;
    if (*Uc > Umax) Umax = *Uc;
    
    float center = (limit / 2.0f) - ((Umax + Umin) / 2.0f);
    
    *Ua += center;
    *Ub += center;
    *Uc += center;
}
