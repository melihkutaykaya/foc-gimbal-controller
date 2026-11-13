#ifndef HARDWARE_HAL_H
#define HARDWARE_HAL_H

#include "stm32f4xx.h" // CMSIS Çekirdek ve Çevre birimi tanımları
#include <stdint.h>
#include <stdbool.h>

void HAL_SystemClock_Init(void);          // 168 MHz PLL + SysTick

void HAL_UART2_DMA_Init(void);            // USART2 PA2 TX, 115200 baud, DMA non-blocking
void HAL_UART2_DMA_Transmit(const char *data);

void HAL_TIM1_PWM_Init(uint16_t arr_value); // Center-aligned PWM on PE9/PE11/PE13
void HAL_MotorDriver_Enable(bool enable);   // PC9 = DRV8313 EN

// I2C1 (PB6=SCL, PB7=SDA) — MPU6050, 400 kHz
void HAL_I2C1_Init(void);
bool HAL_I2C1_Read (uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint16_t length);
void HAL_I2C1_Write(uint8_t dev_address, uint8_t reg_address, uint8_t data);

// I2C2 (PB10=SCL, PB11=SDA) — AS5600, 100 kHz
void HAL_I2C2_Init(void);
bool HAL_I2C2_Read (uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint16_t length);
void HAL_I2C2_Write(uint8_t dev_address, uint8_t reg_address, uint8_t data);

uint32_t Sys_Get_Time_Ms(void);  // millisecond timestamp
uint32_t Sys_Get_Time_Us(void);  // microsecond timestamp (SysTick-derived)

void HAL_GPIO_Button_Init(void); // PA0 blue Discovery button — pull-down input
bool HAL_GPIO_Button_Read(void);

#endif // HARDWARE_HAL_H
