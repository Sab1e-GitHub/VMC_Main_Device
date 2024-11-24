/*
 * pid_wheel_mgr.h
 *
 *  Created on: Oct 30, 2024
 *      Author: Sab1e
 */

#ifndef PID_WHEEL_MGR_H_
#define PID_WHEEL_MGR_H_

typedef struct {
	uint16_t pulse;		//编码器每360度的脉冲数量
	int32_t pulse_count;      // 编码器脉冲计数
	uint8_t previous_state;
	int32_t clamped_pulse_count;
	int32_t max_pulses;
} Encoder_t;

typedef struct {
	float_t current_angle;	//当前角度 重置它可以手动居中
	uint16_t rotation_range;              // 旋转总范围（正向最大度数+反向最大度数）
	float_t angular_velocity;            // 角速度
	float_t angular_acceleration;        // 角加速度
	Encoder_t encoder;
} Steering_Wheel_t;

extern volatile Steering_Wheel_t Steering_Wheel;
void Calculate_Instantaneous_Angular_Velocity(float_t period);
void Calculate_Steering_Wheel_Mechanical_Parameters(void);
#endif /* PID_WHEEL_MGR_H_ */
