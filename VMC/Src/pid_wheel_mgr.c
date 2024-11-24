/*
 * pid_wheel_mgr.c
 *	方向盘管理器
 *	负责方向盘相关参数的计算与控制
 *  Created on: Oct 30, 2024
 *      Author: Sab1e
 */

#include "global.h"

volatile Steering_Wheel_t Steering_Wheel = { .current_angle = 0,         // 当前角度
				.rotation_range = 900, .angular_velocity = 0.0f,    // 旋转速度初始化为0
				.angular_acceleration = 0.0f,      // 旋转加速度初始化为0
				.encoder = { .pulse = 600, .pulse_count = 0,     // 编码器脉冲计数初始化为0
								.previous_state = 0,           // 上一个状态初始化为0
								.clamped_pulse_count = 0,      // 限制的脉冲计数初始化为0
								.max_pulses = 0 // 最大脉冲数 (rotation_range*pluse*4/2)/360
								} };

volatile float_t previous_angle = 0;
///volatile float_t previous_angular_velocity = 0.0;
volatile uint32_t last_time = 0;

//每150ms计算一次平均值
void Calculate_Steering_Wheel_Mechanical_Parameters(void) {

    // 角速度计算（单位：度每秒）
    Steering_Wheel.angular_velocity = (Steering_Wheel.current_angle - previous_angle)  / TIM4_PERIOD_SECONDS;

    // 角加速度计算（单位：度每秒平方）
    //Steering_Wheel.angular_acceleration = (Steering_Wheel.angular_velocity - previous_angular_velocity) / TIM4_PERIOD_SECONDS;

    // 更新历史值
    //previous_angular_velocity = Steering_Wheel.angular_velocity;
    previous_angle = Steering_Wheel.current_angle;
}

