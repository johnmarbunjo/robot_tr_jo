/**
 * @file	invicto_motor.c
 * @author	INVICTO TEAM
 * @brief	Invicto motor library.
 * 			File ini menyediakan pemetaan motor BTS7960 dan perhitungan
 * 			inverse kinematics untuk robot mecanum 4 roda.
 */

#include "invicto_motor.h"
#include "utils.h"

extern TIM_HandleTypeDef htim2, htim4, htim12;
#define NUM_MOTORS_BTS 4

/**
 * @brief	Pemetaan 4 motor BTS7960 ke channel dan timer PWM masing-masing.
 *
 * Urutan array (index 0-3) merepresentasikan Motor 1 - Motor 4 sesuai
 * dokumen pin mapping:
 *
 *   Motor 1 : RPWM = PA1 (TIM2_CH2)  | LPWM = PA0  (TIM2_CH1)
 *   Motor 2 : RPWM = PB7 (TIM4_CH2)  | LPWM = PB6  (TIM4_CH1)
 *   Motor 3 : RPWM = PB10 (TIM2_CH3) | LPWM = PB11 (TIM2_CH4)
 *   Motor 4 : RPWM = PB15 (TIM12_CH2)| LPWM = PB14 (TIM12_CH1)
 */
Motor_BTS motors_bts[NUM_MOTORS_BTS] =
{
		{TIM_CHANNEL_2, &htim2,  TIM_CHANNEL_1, &htim2 },      //1
		{TIM_CHANNEL_2, &htim4,  TIM_CHANNEL_1, &htim4 },      //2
		{TIM_CHANNEL_3, &htim2,  TIM_CHANNEL_4, &htim2 },      //3
		{TIM_CHANNEL_2, &htim12, TIM_CHANNEL_1, &htim12},      //4
};

/**
 * @brief	Sets the speed (and direction) of a specified BTS7960 motor.
 *
 * BTS7960 tidak memakai pin arah GPIO terpisah seperti driver EMS/VNH.
 * Arah gerak motor ditentukan murni dari channel PWM mana yang diberi
 * duty cycle:
 *   - speed > 0  -> PWM maju (RPWM) diisi duty cycle, PWM mundur (LPWM) = 0
 *   - speed < 0  -> PWM mundur (LPWM) diisi duty cycle, PWM maju (RPWM) = 0
 *   - speed == 0 -> kedua channel PWM = 0 (motor berhenti / coast)
 *
 * @param	motor Pointer ke struct Motor_BTS yang ingin dikontrol.
 * @param	speed Kecepatan yang diinginkan (positif = maju, negatif = mundur).
 *
 * @return	None.
 */
void setMotorSpeedBTS(Motor_BTS *motor, double speed)
{
    if (speed > 0)
    {
        __HAL_TIM_SET_COMPARE(motor->forward_timer, motor->forward_channel, (uint32_t)speed);
        __HAL_TIM_SET_COMPARE(motor->backward_timer, motor->backward_channel, 0);
    }
    else if (speed < 0)
    {
        __HAL_TIM_SET_COMPARE(motor->forward_timer, motor->forward_channel, 0);
        __HAL_TIM_SET_COMPARE(motor->backward_timer, motor->backward_channel, (uint32_t)(-speed));
    }
    else
    {
        __HAL_TIM_SET_COMPARE(motor->forward_timer, motor->forward_channel, 0);
        __HAL_TIM_SET_COMPARE(motor->backward_timer, motor->backward_channel, 0);
    }
}

/**
 * @brief	Calculates and sets the speeds of the motors based on the desired robot velocities.
 *
 * Fungsi ini menghitung kecepatan tiap roda (V1x..V4x) dari 3 nilai
 * kecepatan robot secara keseluruhan (Vx, Vy, W) menggunakan rumus
 * inverse kinematics mecanum standar, lalu mengirim nilai tersebut
 * ke setMotorSpeedBTS() untuk masing-masing motor.
 *
 * @param	Vx The desired linear velocity in the x direction (maju/mundur).
 * @param	Vy The desired linear velocity in the y direction (geser kiri/kanan).
 * @param	W  The desired angular velocity (rotasi di tempat).
 * @param	z  Offset tambahan/koreksi (misal kompensasi drift), ditambahkan
 * 			   secara diagonal-berlawanan pada tiap roda.
 *
 * @return	None.
 */
void Inverse_Kinematics_Mecanum(double Vx, double Vy, double W, double z)
{
	float r = (7.6);
	double V1x = (((Vx - Vy - (25) * W ) / r)*1.555)+z;
	double V2x = (((Vx + Vy + (25) * W ) / r)*1.015)-z;		//0.942
	double V3x = (((Vx + Vy - (25) * W ) / r)*1.555)+z;		//0.982
	double V4x = (((Vx - Vy + (25) * W ) / r)*1.015)-z;		//1.017

	setMotorSpeedBTS(&motors_bts[0], (int)V1x);
	setMotorSpeedBTS(&motors_bts[1], (int)V2x);
	setMotorSpeedBTS(&motors_bts[2], (int)V3x);
	setMotorSpeedBTS(&motors_bts[3], (int)V4x);
}

void Inverse_Kinematics_Omni(double Vx, double Vy, double W)
{
	double R = 7.6;
//    double minThreshold = 1000;
//    double wheelMaxSpeed = 5500;
	const double pi_over_four = 0.78539816339744830962;
    double V1x = (-sin(1 * pi_over_four) * Vx + cos(1 * pi_over_four) * Vy - R * W)*1.047;//1.201
    double V2x = (-sin(3 * pi_over_four) * Vx + cos(3 * pi_over_four) * Vy - R * W)*0.878;//0.942
    double V3x = (-sin(5 * pi_over_four) * Vx + cos(5 * pi_over_four) * Vy - R * W)*0.972;//0.982
    double V4x = (-sin(7 * pi_over_four) * Vx + cos(7 * pi_over_four) * Vy - R * W)*0.829;

//    double maxM = fmax(fabs(M1), fmax(fabs(M2), fmax(fabs(M3), fabs(M4))));
//
//    if (maxM > wheelMaxSpeed) {
//        double scale = wheelMaxSpeed / maxM;
//        M1 *= scale;
//        M2 *= scale;
//        M3 *= scale;
//        M4 *= scale;
//    }
//
//    double V1 = (fabs(M1) > minThreshold) ? M1 : (M1 < 0) ? -minThreshold : ((M1 > 0) ? minThreshold : 0);
//    double V2 = (fabs(M2) > minThreshold) ? M2 : (M2 < 0) ? -minThreshold : ((M2 > 0) ? minThreshold : 0);
//    double V3 = (fabs(M3) > minThreshold) ? M3 : (M3 < 0) ? -minThreshold : ((M3 > 0) ? minThreshold : 0);
//    double V4 = (fabs(M4) > minThreshold) ? M4 : (M4 < 0) ? -minThreshold : ((M4 > 0) ? minThreshold : 0);

//    setMotorSpeedBTS(&motors_bts[0], (int)M1);
//    setMotorSpeedBTS(&motors_bts[1], (int)M2);
//    setMotorSpeedBTS(&motors_bts[2], (int)M3);
//    setMotorSpeedBTS(&motors_bts[3], (int)M4);

    setMotorSpeedBTS(&motors_bts[0], (int)V1x);
	setMotorSpeedBTS(&motors_bts[1], (int)V2x);
	setMotorSpeedBTS(&motors_bts[2], (int)V3x);
	setMotorSpeedBTS(&motors_bts[3], (int)V4x);
}
/**
 * @brief	Sama seperti Inverse_Kinematics_Mecanum(), dipakai khusus untuk
 * 			memberi pulsa singkat berlawanan arah (auto-brake) saat tombol
 * 			gerak dilepas. Dipisah dari fungsi utama agar pemanggilannya
 * 			jelas dibedakan konteksnya di main.c (lihat blok default: pada
 * 			switch-case perintah ESP32).
 *
 * @param	Vx, Vy, W, z	Sama seperti Inverse_Kinematics_Mecanum().
 *
 * @return	None.
 */
void Inverse_Kinematics_Mecanum_Decay(double Vx, double Vy, double W, double z)
{
	float r = (7.6);
	double V1x = (((Vx - Vy - (25) * W ) / r)*1.555)+z;
	double V2x = (((Vx + Vy + (25) * W ) / r)*1.015)-z;		//0.942
	double V3x = (((Vx + Vy - (25) * W ) / r)*1.555)+z;		//0.982
	double V4x = (((Vx - Vy + (25) * W ) / r)*1.015)-z;		//1.017

	setMotorSpeedBTS(&motors_bts[0], (int)V1x);
	setMotorSpeedBTS(&motors_bts[1], (int)V2x);
	setMotorSpeedBTS(&motors_bts[2], (int)V3x);
	setMotorSpeedBTS(&motors_bts[3], (int)V4x);
}
