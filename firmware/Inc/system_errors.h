#ifndef SYSTEM_ERRORS_H
#define SYSTEM_ERRORS_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// HATA KODLARI
// ============================================================================
typedef uint16_t SystemError_t;
#define SYS_OK  0x0000u

// 0x01xx — IMU (MPU6050 / I2C1)
#define ERR_IMU_READ_FAIL       0x0101u   // I2C okuma başarısız  → Critical : Sys_Safe_Shutdown + 250ms + reset
#define ERR_IMU_CALIB_FAIL      0x0102u   // Gyro kalibrasyonu    → Major : offset=0 ile devam

// 0x02xx — Encoder (AS5600 / I2C2)
#define ERR_ENC_READ_FAIL       0x0201u   // I2C okuma başarısız  → Minor : son değeri kullan
#define ERR_ENC_NO_MAGNET       0x0202u   // Mıknatıs yok         → Critical : dur
#define ERR_ENC_MAGNET_WEAK     0x0203u   // Mıknatıs zayıf       → Minor : logla
#define ERR_ENC_MAGNET_STRONG   0x0204u   // Mıknatıs güçlü       → Minor : logla

// 0x03xx — Motor
#define ERR_MOTOR_ANGLE_LIMIT   0x0301u   // ±170° sınırı         → Minor : geri kuvvet
#define ERR_MOTOR_LOCKED_ROTOR  0x0302u   // Kilitli rotor        → Major : 5 kere recovery
#define ERR_MOTOR_VOLTAGE_SAT   0x0303u   // Voltaj doyumu        → Info  : logla

// 0x04xx — Kontrol Döngüsü
#define ERR_CTRL_DEADLINE_MISS  0x0401u   // Deadline aşımı       → Minor : dt klampla
#define ERR_CTRL_WINDUP         0x0402u   // Integral windup      → Info  : logla

// 0x05xx — Kalibrasyon / Başlatma
#define ERR_CALIB_ENC_FAIL      0x0501u   // Encoder kalibrasyon  → Critical : dur

// ============================================================================
// ŞİDDET SEVİYELERİ
// ============================================================================
typedef enum {
    SEV_INFO     = 0,   // Bilgi — logla, devam et
    SEV_MINOR    = 1,   // Küçük — logla, devam et
    SEV_MAJOR    = 2,   // Büyük — logla, recovery dene
    SEV_CRITICAL = 3    // Kritik — Sys_Safe_Shutdown() + while(1)
} ErrorSeverity_t;

// ============================================================================
// HATA LOG GİRİŞİ
// ============================================================================
#define ERROR_LOG_SIZE 12u

typedef struct {
    SystemError_t code;
    uint32_t      count;
    uint32_t      last_time_ms;
} ErrorEntry_t;

// ============================================================================
// FONKSİYONLAR
// ============================================================================

// Ana hata işleyici: logla + Critical için kapat/durdur
// location: kısa string ("MAIN", "INIT", "PID" ...)
// data    : ek bilgi (satır no, sayaç vb.)
void Error_Handle(SystemError_t code, const char *location, uint32_t data);

// Kaç farklı hata türü kayıtlı?
uint8_t Error_Get_Active_Count(void);

// Soft RT telemetri bloğundan çağrılır — UART ile log basar
void Error_Print_Telemetry(void);

// Donanım kapatma (Critical'dan çağrılır, main.c de kullanabilir)
void Sys_Safe_Shutdown(void);

#endif // SYSTEM_ERRORS_H
