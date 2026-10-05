/*
 * utils.c
 *
 *  Created on: Dec 7, 2024
 *      Author: zakim
 */

#include <stdint.h>
#include "stm32f4xx_hal.h"

// Fungsi konversi linear dengan floating point yang lebih presisi
int map(int afterFrom, int afterTo, int beforeFrom, int beforeTo, int value)
{
    return (1.0 * (value - afterFrom)) / ((afterTo - afterFrom) * 1.0) * (beforeTo - beforeFrom) + beforeFrom;
}

// Fungsi pembatas nilai (constrain)
int constrain(int value, int minVal, int maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// Fungsi delay milliseconds yang lebih akurat
void delayMicroseconds(uint32_t microseconds)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < (microseconds / 1000));
}

// Fungsi untuk menghasilkan pulse dengan durasi tertentu
void generatePulse(GPIO_TypeDef* port, uint16_t pin, uint32_t durationUs)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    delayMicroseconds(durationUs);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

// Fungsi interpolasi sederhana
float lerp(float a, float b, float t)
{
    return a + t * (b - a);
}
