/**
 * @file	invicto_motor.h
 * @author	INVICTO TEAM
 * @brief	Header file of invicto_motor.c.
 * 			Versi BTS7960 murni untuk robot mecanum 4 roda.
 */

#ifndef INVICTO_MOTOR_H
#define INVICTO_MOTOR_H

#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * @brief Motor structure definition
 */

//BTS7960 Motor Driver
//Setiap motor menggunakan 2 channel PWM independen:
//forward_channel (RPWM) untuk arah maju, backward_channel (LPWM) untuk arah mundur.
typedef struct
{
    uint8_t forward_channel;
    TIM_HandleTypeDef *forward_timer;
    uint8_t backward_channel;
    TIM_HandleTypeDef *backward_timer;
} Motor_BTS;

/**
 * @brief function definition
 */
void motorDirection(uint8_t motor, uint8_t direction);
void setMotorSpeedBTS(Motor_BTS *motor, double speed);
void Inverse_Kinematics_Mecanum(double Vx, double Vy, double W, double z);
void Inverse_Kinematics_Mecanum_Decay(double Vx, double Vy, double W, double z);
void Inverse_Kinematics_Omni(double Vx, double Vy, double W);
#endif
