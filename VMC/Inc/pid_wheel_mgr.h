/*
 * pid_wheel_mgr.h
 *
 *  Created on: Oct 30, 2024
 *      Author: Sab1e
 */

#ifndef PID_WHEEL_MGR_H_
#define PID_WHEEL_MGR_H_

typedef struct {
	uint16_t pulse;		//������ÿ360�ȵ���������
	int32_t pulse_count;      // �������������
	uint8_t previous_state;
	int32_t clamped_pulse_count;
	int32_t max_pulses;
} Encoder_t;

typedef struct {
	float_t current_angle;	//��ǰ�Ƕ� �����������ֶ�����
	uint16_t rotation_range;              // ��ת�ܷ�Χ������������+������������
	float_t angular_velocity;            // ���ٶ�
	float_t angular_acceleration;        // �Ǽ��ٶ�
	Encoder_t encoder;
} Steering_Wheel_t;

extern volatile Steering_Wheel_t Steering_Wheel;
void Calculate_Instantaneous_Angular_Velocity(float_t period);
void Calculate_Steering_Wheel_Mechanical_Parameters(void);
#endif /* PID_WHEEL_MGR_H_ */
