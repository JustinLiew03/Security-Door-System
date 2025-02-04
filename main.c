#include "stm32f4xx.h"

void GPIO_Init_Custom(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA0, PA1, PA2 as inputs
    GPIOA->MODER |= (3U << (2 * 0)); // PA0 (Analog mode for LDR)
    GPIOA->MODER &= ~((3U << (2 * 1)) | (3U << (2 * 2))); // PA1, PA2 (Input mode)
    GPIOA->PUPDR &= ~((3U << (2 * 1)) | (3U << (2 * 2))); // No pull-up/pull-down for PA1, PA2

    // Configure PA8, PA9, PA10 as outputs
    GPIOA->MODER |= (1U << (2 * 8)) | (1U << (2 * 9)) | (1U << (2 * 10)); // Output mode
    GPIOA->OTYPER &= ~((1U << 8) | (1U << 9) | (1U << 10)); // Push-pull output
    GPIOA->OSPEEDR |= (3U << (2 * 8)) | (3U << (2 * 9)) | (3U << (2 * 10)); // High speed
    GPIOA->PUPDR &= ~((3U << (2 * 8)) | (3U << (2 * 9)) | (3U << (2 * 10))); // No pull-up/pull-down
}

void ADC_Init_Custom(void) {
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC1
    ADC1->CR1 &= ~ADC_CR1_RES; // 12-bit resolution
    ADC1->CR2 |= ADC_CR2_CONT; // Continuous conversion mode
    ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC1

    // Set sampling time for channel 0 (PA0)
    ADC1->SMPR2 |= (3U << (3 * 0)); // 144 cycles

    // Configure ADC channel sequence (only Channel 0, PA0)
    ADC1->SQR3 = 0;
}

void ADC_Start(void) {
    // Start ADC conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint16_t ADC_Read(void) {
    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

void delay_ms(uint32_t ms) {
    // Basic delay loop (assuming 16 MHz clock)
    for (uint32_t i = 0; i < ms * 1600; i++) {
        __NOP();
    }
}

int main(void) {
    // Initialize GPIO and ADC
    GPIO_Init_Custom();
    ADC_Init_Custom();
    ADC_Start();

    while (1) {
        // Read LDR value (PA0)
        uint16_t ldr_value = ADC_Read();

        // Check light intensity
        if (ldr_value < 200) {
            GPIOA->ODR |= (1U << 9); // Turn on Yellow LED (PA9)
        } else {
            GPIOA->ODR &= ~(1U << 9); // Turn off Yellow LED (PA9)
        }

        // Check IR1 (PA1, door open)
        if (
					(GPIOA->IDR & (1U << 1))) { // Logic 0: Door open
            GPIOA->ODR |= (1U << 10); // Turn on Buzzer (PA10)
        } else {
            GPIOA->ODR &= ~(1U << 10); // Turn off Buzzer (PA10)
        }

        // Check IR2 (PA2, approach detection)
        if (!(GPIOA->IDR & (1U << 2))) { // Logic 1: Someone approaches
            GPIOA->ODR |= (1U << 10); // Turn on Buzzer (PA10)

            // Blink Red LED (PA8)
            GPIOA->ODR |= (1U << 8); // Turn on Red LED
            delay_ms(200);
            GPIOA->ODR &= ~(1U << 8); // Turn off Red LED
            delay_ms(200);
        }
    }
}
