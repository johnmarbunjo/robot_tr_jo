/*
 * utils.h
 *
 *  Created on: Dec 7, 2024
 *      Author: zakim
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

int map(int afterFrom, int afterTo, int beforeFrom, int beforeTo, int value);
int constrain(int value, int minVal, int maxVal);
void delayMicroseconds(uint32_t microseconds);
void generatePulse(GPIO_TypeDef* port, uint16_t pin, uint32_t durationUs);
float lerp(float a, float b, float t);

#endif /* INC_UTILS_H_ */
