#include "hardware_hal.h"
#include <string.h>

volatile uint32_t sys_time_ms = 0;

// PLL: HSI/16 × 336 / 2 = 168 MHz; APB1 = /4 = 42 MHz; APB2 = /2 = 84 MHz
void HAL_SystemClock_Init(void) {
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // enable FPU
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
    RCC->PLLCFGR = (16ul << 0) | (336ul << 6) | (0ul << 16) | (7ul << 24);
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    SysTick_Config(168000);  // 1 ms tick
}

void SysTick_Handler(void) {
    sys_time_ms++;
}

__attribute__((weak)) uint32_t Sys_Get_Time_Ms(void) {
    return sys_time_ms;
}

// Microsecond timestamp derived from SysTick down-counter (168 ticks = 1 µs)
uint32_t Sys_Get_Time_Us(void) {
    uint32_t ms, val;
    do {
        ms  = sys_time_ms;
        val = SysTick->VAL;
    } while (ms != sys_time_ms);  // re-read on ms rollover
    return (ms * 1000) + ((168000 - val) / 168);
}

// USART2 + DMA1 Stream6 — non-blocking TX on PA2 (115200 baud, 8N1)
void HAL_UART2_DMA_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER  &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
    GPIOA->MODER  |=  (2 << GPIO_MODER_MODE2_Pos) | (2 << GPIO_MODER_MODE3_Pos);
    GPIOA->AFR[0] |=  (7 << (2 * 4)) | (7 << (3 * 4));  // AF7 = USART2

    USART2->BRR  = 0x016C;  // 115200 @ 42 MHz APB1
    USART2->CR3 |= USART_CR3_DMAT;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART2_IRQn);

    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream6->CR & DMA_SxCR_EN);
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream6->CR  = (4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
}

// Start DMA transfer — waits only for the previous transfer to finish (non-blocking otherwise)
void HAL_UART2_DMA_Transmit(const char *data) {
    uint32_t len = strlen(data);
    while (DMA1_Stream6->CR & DMA_SxCR_EN);
    DMA1->HIFCR       |= DMA_HIFCR_CTCIF6;
    DMA1_Stream6->M0AR = (uint32_t)data;
    DMA1_Stream6->NDTR = len;
    DMA1_Stream6->CR  |= DMA_SxCR_EN;
}

// TIM1 center-aligned PWM on PE9/PE11/PE13 (CH1/2/3), 20 kHz at ARR=4200
// PC9 = DRV8313 EN pin (HIGH = active)
void HAL_TIM1_PWM_Init(uint16_t arr_value) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // PC9 — DRV8313 enable output, set HIGH immediately
    GPIOC->MODER &= ~(3 << (9 * 2));
    GPIOC->MODER |=  (1 << (9 * 2));
    GPIOC->BSRR   =  (1 << 9);

    // PE9/PE11/PE13 — AF1 (TIM1 CH1/2/3)
    GPIOE->MODER  &= ~((3 << 18) | (3 << 22) | (3 << 26));
    GPIOE->MODER  |=  ((2 << 18) | (2 << 22) | (2 << 26));
    GPIOE->AFR[1] &= ~((15 << 4) | (15 << 12) | (15 << 20));
    GPIOE->AFR[1] |=   ((1 << 4) | (1 << 12)  | (1 << 20));

    TIM1->CR1   = (1 << TIM_CR1_CMS_Pos);  // center-aligned mode 1
    TIM1->ARR   = arr_value;
    TIM1->PSC   = 0;

    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                   (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM1->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->BDTR  |= TIM_BDTR_MOE;  // advanced timer: main output enable required

    // TRGO = Update event — future ADC injection sync at PWM mid-point
    TIM1->CR2 &= ~TIM_CR2_MMS_Msk;
    TIM1->CR2 |=  (2 << TIM_CR2_MMS_Pos);

    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->CR1 |= TIM_CR1_CEN;
}

// PC9 HIGH = DRV8313 active; LOW = Hi-Z coast
void HAL_MotorDriver_Enable(bool enable) {
    if (enable) GPIOC->BSRR = (1u << 9);
    else        GPIOC->BSRR = (1u << (9 + 16));
}

// HAL_ADC_Injected_Init() removed — PA0 is shared with the blue button

// I2C1 (PB6=SCL, PB7=SDA) — MPU6050, 400 kHz fast mode, APB1=42 MHz
void HAL_I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER  &= ~((3 << 12) | (3 << 14));
    GPIOB->MODER  |=  ((2 << 12) | (2 << 14));   // AF mode
    GPIOB->OTYPER |=  (1 << 6) | (1 << 7);        // open-drain (required for I2C)
    GPIOB->OSPEEDR|=  (3 << 12) | (3 << 14);
    GPIOB->PUPDR  &= ~((3 << 12) | (3 << 14));
    GPIOB->PUPDR  |=  ((1 << 12) | (1 << 14));    // internal pull-up
    GPIOB->AFR[0] &= ~((15 << 24) | (15 << 28));
    GPIOB->AFR[0] |=   (4 << 24)  | (4 << 28);   // AF4 = I2C1

    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    I2C1->CR2   = 42;               // PCLK1 = 42 MHz
    I2C1->CCR   = I2C_CCR_FS | 35;  // 400 kHz: 42 MHz / (3 × 400 kHz) = 35
    I2C1->TRISE = 14;               // (42 MHz × 300 ns) + 1
    I2C1->CR1  |= I2C_CR1_PE;
}

// Polling burst read from I2C1 device. Returns false on any timeout/NACK.
bool HAL_I2C1_Read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint16_t length) {
    uint32_t timeout;

    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB))   { if (--timeout == 0) return false; }

    I2C1->DR = (dev_address << 1);
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0) return false; }
    (void)I2C1->SR2;

    I2C1->DR = reg_address;
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_TXE))  { if (--timeout == 0) return false; }

    I2C1->CR1 |= I2C_CR1_START;  // repeated start
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB))   { if (--timeout == 0) return false; }

    I2C1->DR = (dev_address << 1) | 1;
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0) return false; }

    if (length == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
        timeout = 10000;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return false; }
        *buffer = I2C1->DR;
    } else {
        // RM0090 §27.3.3: ACK must be set BEFORE clearing ADDR (SR2 read).
        // Reversed order NACKs the first byte → slave I2C lockup.
        I2C1->CR1 |= I2C_CR1_ACK;
        (void)I2C1->SR2;
        while (length > 0) {
            if (length == 1) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                I2C1->CR1 |= I2C_CR1_STOP;
            }
            timeout = 10000;
            while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return false; }
            *buffer++ = I2C1->DR;
            length--;
        }
    }
    return true;
}

// Single-register write to I2C1 device (sensor configuration)
void HAL_I2C1_Write(uint8_t dev_address, uint8_t reg_address, uint8_t data) {
    uint32_t timeout;

    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_SB))   { if (--timeout == 0) return; }

    I2C1->DR = (dev_address << 1);
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0) return; }
    (void)I2C1->SR2;

    I2C1->DR = reg_address;
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_TXE))  { if (--timeout == 0) return; }

    I2C1->DR = data;
    timeout = 10000;
    while (!(I2C1->SR1 & I2C_SR1_TXE))  { if (--timeout == 0) return; }

    I2C1->CR1 |= I2C_CR1_STOP;
}

// I2C2 (PB10=SCL, PB11=SDA) — AS5600, 100 kHz standard mode (weak pull-ups)
void HAL_I2C2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    GPIOB->MODER  &= ~((3 << 20) | (3 << 22));
    GPIOB->MODER  |=  ((2 << 20) | (2 << 22));
    GPIOB->OTYPER |=  (1 << 10) | (1 << 11);
    GPIOB->OSPEEDR|=  (3 << 20) | (3 << 22);
    GPIOB->PUPDR  &= ~((3 << 20) | (3 << 22));
    GPIOB->PUPDR  |=  ((1 << 20) | (1 << 22));
    GPIOB->AFR[1] &= ~((15 << 8) | (15 << 12));
    GPIOB->AFR[1] |=   (4 << 8)  | (4 << 12);  // AF4 = I2C2

    I2C2->CR1 |= I2C_CR1_SWRST;
    I2C2->CR1 &= ~I2C_CR1_SWRST;
    I2C2->CR2   = 42;
    I2C2->CCR   = 210;   // 42 MHz / (2 × 100 kHz) = 210
    I2C2->TRISE = 43;    // 42 MHz × 1000 ns + 1
    I2C2->CR1  |= I2C_CR1_PE;
}

// Polling burst read from I2C2 device. Returns false on any timeout/NACK.
bool HAL_I2C2_Read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint16_t length) {
    uint32_t timeout;

    I2C2->CR1 |= I2C_CR1_START;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_SB))   { if (--timeout == 0) return false; }

    I2C2->DR = (dev_address << 1);
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0) return false; }
    (void)I2C2->SR2;

    I2C2->DR = reg_address;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_TXE))  { if (--timeout == 0) return false; }

    I2C2->CR1 |= I2C_CR1_START;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_SB))   { if (--timeout == 0) return false; }

    I2C2->DR = (dev_address << 1) | 1;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0) return false; }

    if (length == 1) {
        I2C2->CR1 &= ~I2C_CR1_ACK;
        (void)I2C2->SR2;
        I2C2->CR1 |= I2C_CR1_STOP;
        timeout = 10000;
        while (!(I2C2->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return false; }
        *buffer = I2C2->DR;
    } else {
        // RM0090: ACK before SR2 read (same rule as I2C1)
        I2C2->CR1 |= I2C_CR1_ACK;
        (void)I2C2->SR2;
        while (length > 0) {
            if (length == 1) {
                I2C2->CR1 &= ~I2C_CR1_ACK;
                I2C2->CR1 |= I2C_CR1_STOP;
            }
            timeout = 10000;
            while (!(I2C2->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return false; }
            *buffer++ = I2C2->DR;
            length--;
        }
    }
    return true;
}

// Single-register write to I2C2 device
void HAL_I2C2_Write(uint8_t dev_address, uint8_t reg_address, uint8_t data) {
    uint32_t timeout;

    I2C2->CR1 |= I2C_CR1_START;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_SB))   { if (--timeout == 0) return; }

    I2C2->DR = (dev_address << 1);
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0) return; }
    (void)I2C2->SR2;

    I2C2->DR = reg_address;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_TXE))  { if (--timeout == 0) return; }

    I2C2->DR = data;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_TXE))  { if (--timeout == 0) return; }

    I2C2->CR1 |= I2C_CR1_STOP;
}

// PA0 — Discovery blue button, pull-down input
void HAL_GPIO_Button_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3 << 0);   // input
    GPIOA->PUPDR &= ~(3 << 0);
    GPIOA->PUPDR |=  (2 << 0);   // pull-down
}

bool HAL_GPIO_Button_Read(void) {
    return (GPIOA->IDR & 1) != 0;
}
