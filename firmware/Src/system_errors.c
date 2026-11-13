#include "system_errors.h"
#include "hardware_hal.h"
#include <stdio.h>
#include <string.h>

// Maps error code → severity + display name
typedef struct {
    SystemError_t   code;
    ErrorSeverity_t severity;
    const char     *name;
} ErrorDef_t;

static const ErrorDef_t s_error_defs[] = {
    { ERR_IMU_READ_FAIL,      SEV_CRITICAL, "IMU okuma"           },
    { ERR_IMU_CALIB_FAIL,     SEV_MAJOR,    "Gyro kalibrasyon"    },
    { ERR_ENC_READ_FAIL,      SEV_MINOR,    "ENC okuma"           },
    { ERR_ENC_NO_MAGNET,      SEV_CRITICAL, "Miknatiz yok"        },
    { ERR_ENC_MAGNET_WEAK,    SEV_MINOR,    "Miknatiz zayif"      },
    { ERR_ENC_MAGNET_STRONG,  SEV_MINOR,    "Miknatiz guclu"      },
    { ERR_MOTOR_ANGLE_LIMIT,  SEV_MINOR,    "Aci siniri"          },
    { ERR_MOTOR_LOCKED_ROTOR, SEV_MAJOR,    "Kilitli rotor"       },
    { ERR_MOTOR_VOLTAGE_SAT,  SEV_INFO,     "Voltaj doyumu"       },
    { ERR_CTRL_DEADLINE_MISS, SEV_MINOR,    "Deadline asimi"      },
    { ERR_CTRL_WINDUP,        SEV_INFO,     "Integral windup"     },
    { ERR_CALIB_ENC_FAIL,     SEV_CRITICAL, "ENC kalibrasyon"     },
};

#define ERROR_DEF_COUNT  (sizeof(s_error_defs) / sizeof(s_error_defs[0]))

static ErrorEntry_t s_log[ERROR_LOG_SIZE];
static uint8_t      s_active = 0;   // number of unique errors recorded

// Find existing log entry for code, or allocate a new one
static ErrorEntry_t *get_entry(SystemError_t code) {
    for (uint8_t i = 0; i < s_active; i++) {
        if (s_log[i].code == code) return &s_log[i];
    }
    if (s_active < ERROR_LOG_SIZE) {
        ErrorEntry_t *e = &s_log[s_active++];
        e->code          = code;
        e->count         = 0u;
        e->last_time_ms  = 0u;
        return e;
    }
    return NULL;  // log full — oldest entry could be evicted; ignored for now
}

// Find static definition for code
static const ErrorDef_t *get_def(SystemError_t code) {
    for (uint8_t i = 0; i < ERROR_DEF_COUNT; i++) {
        if (s_error_defs[i].code == code) return &s_error_defs[i];
    }
    return NULL;
}

// Log error and, for CRITICAL severity, safe-shutdown then reset
void Error_Handle(SystemError_t code, const char *location, uint32_t data) {
    (void)location;
    (void)data;

    ErrorEntry_t *entry = get_entry(code);
    if (entry != NULL) {
        entry->count++;
        entry->last_time_ms = Sys_Get_Time_Ms();
    }

    const ErrorDef_t *def = get_def(code);
    if (def != NULL && def->severity == SEV_CRITICAL) {
        Sys_Safe_Shutdown();
        // 250 ms delay: allows sensor reconnection before calibration restarts
        uint32_t t0 = Sys_Get_Time_Ms();
        while (Sys_Get_Time_Ms() - t0 < 250u) {}
        NVIC_SystemReset();
    }
}

uint8_t Error_Get_Active_Count(void) {
    return s_active;
}

// Serialise all log entries into one buffer and transmit via single DMA call.
// Multiple DMA calls each block ~5-8 ms → deadline miss cascade.
void Error_Print_Telemetry(void) {
    if (s_active == 0u) return;

    static char out[750];  // 12 entries × ~55 chars + header ≈ 670

    int pos = 0;
    int rem = (int)sizeof(out);
    int n;

    n = snprintf(out + pos, (size_t)rem, "ERR(%u):", s_active);
    pos += n; rem -= n;

    for (uint8_t i = 0; i < s_active && rem > 1; i++) {
        const ErrorDef_t *def = get_def(s_log[i].code);
        const char *name = (def != NULL) ? def->name : "?";

        n = snprintf(out + pos, (size_t)rem, " [%04X]%s x%lu @%lums",
                     (unsigned)s_log[i].code, name,
                     (unsigned long)s_log[i].count,
                     (unsigned long)s_log[i].last_time_ms);
        pos += n; rem -= n;
    }

    if (rem > 2) { out[pos++] = '\r'; out[pos++] = '\n'; out[pos] = '\0'; }
    HAL_UART2_DMA_Transmit(out);
}

// Zero all PWM channels and pull DRV8313 EN low (gate Hi-Z, motor coasts)
void Sys_Safe_Shutdown(void) {
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    GPIOC->BSRR = (1u << (9u + 16u));  // PC9 LOW → DRV8313 Hi-Z
}
