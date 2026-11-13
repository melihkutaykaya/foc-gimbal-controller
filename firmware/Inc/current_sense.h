#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>
#include "foc_math.h"

// INA240A2 (Gain: 50 V/V) ve 0.01R (10mOhm) şönt direnci kullanan
// Inline (Hat Üzeri) akım algılama parametreleri
typedef struct {
    float shunt_resistor;       // 0.01R
    float amp_gain;             // 50.0f
    float volts_to_amps_ratio;  // 1.0 / (shunt_resistor * amp_gain)
    float offset_voltage;       // Sıfır akımdaki çıkış voltajı (örn: 1.65V)
    
    // ADC'den okunan voltaj (Offset çıkarılmış ham ölçüm)
    float voltage_a;
    float voltage_c;
    
    // Hesaplanmış Faz Akımları
    ABC_t phase_currents;
} InlineCurrentSense_t;

// Sisteme özgü akım okuma yapılandırması
void CurrentSense_Init(InlineCurrentSense_t *cs, float shunt, float gain, float offset_v);

// ADC'den gelen voltajları akıma çevirir ve I_b akımını Kirchhoff yasasıyla hesaplar
// Sadece A ve C fazlarının ölçüldüğü varsayılmıştır.
void CurrentSense_Update(InlineCurrentSense_t *cs, float v_a, float v_c);

#endif // CURRENT_SENSE_H
