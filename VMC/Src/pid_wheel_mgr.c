/*
 * pid_wheel_mgr.c
 *	�����̹�����
 *	����������ز����ļ��������
 *  Created on: Oct 30, 2024
 *      Author: Sab1e
 */

#include "global.h"

volatile Steering_Wheel_t Steering_Wheel = { .current_angle = 0,         // ��ǰ�Ƕ�
				.rotation_range = 900, .angular_velocity = 0.0f,    // ��ת�ٶȳ�ʼ��Ϊ0
				.angular_acceleration = 0.0f,      // ��ת���ٶȳ�ʼ��Ϊ0
				.encoder = { .pulse = 600, .pulse_count = 0,     // ���������������ʼ��Ϊ0
								.previous_state = 0,           // ��һ��״̬��ʼ��Ϊ0
								.clamped_pulse_count = 0,      // ���Ƶ����������ʼ��Ϊ0
								.max_pulses = 0 // ��������� (rotation_range*pluse*4/2)/360
								} };

volatile float_t previous_angle = 0;
///volatile float_t previous_angular_velocity = 0.0;
volatile uint32_t last_time = 0;

//ÿ150ms����һ��ƽ��ֵ
void Calculate_Steering_Wheel_Mechanical_Parameters(void) {

    // ���ٶȼ��㣨��λ����ÿ�룩
    Steering_Wheel.angular_velocity = (Steering_Wheel.current_angle - previous_angle)  / TIM4_PERIOD_SECONDS;

    // �Ǽ��ٶȼ��㣨��λ����ÿ��ƽ����
    //Steering_Wheel.angular_acceleration = (Steering_Wheel.angular_velocity - previous_angular_velocity) / TIM4_PERIOD_SECONDS;

    // ������ʷֵ
    //previous_angular_velocity = Steering_Wheel.angular_velocity;
    previous_angle = Steering_Wheel.current_angle;
}

