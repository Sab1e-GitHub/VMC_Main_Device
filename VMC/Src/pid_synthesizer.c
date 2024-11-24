/*
 * pid_synthesizer.c
 *
 *  Created on: Oct 28, 2024
 *      Author: Sab1e
 */
#include "global.h"

#define MAX_PWM ACTUATOR_TIM.Init.Period // 设定 PWM 的最大值
#define MIN_PWM 0

#define DEBUG_SAMPLE_RATE_MAX 1000

uint16_t Synthesizer_Global_Gain = 0;
uint32_t pid_last_time = 0; // 获取开始时间
uint32_t envelope_last_time = 0;

Effect_Gain_Controller_t Effect_Gain_Controller = { .spring_gain = 100,
				.damper_gain = 50, .friction_gain = 100, .inertia_gain = 10 };

Effect_Limiter_t Effect_Limiter = { .inertia_limiter = 20 };

PWM_Global_Parameters_t PWM_Global_Parameters = { .pwm_gain_multiple = 1.0f };

// PID控制器实例
PID_Controller spring_pid;

// 在合成效果之前初始化PID
float_t Spring_Kp = 4.0f;
float_t Spring_Ki = 0.1f;
float_t Spring_Kd = 1.0f;

void Spring_PID_Init(PID_Controller *pid, float_t Kp, float_t Ki, float_t Kd) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->previous_error = 0.0f;
	pid->integral = 0.0f;
}

//PID计算的是角度
float_t PID_Compute(PID_Controller *pid, float_t setpoint,
				float_t measured_value, float_t dt) {
	float_t error = setpoint - measured_value;
	pid->integral += error * dt;
	float_t derivative = error - pid->previous_error;
	pid->previous_error = error;

	return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

// 检查PWM是否正在运行
uint8_t Is_PWM_Running(void) {
	return (__HAL_TIM_GET_COMPARE(&ACTUATOR_TIM,
					POSITIVE_ACTUATOR_CHANNEL) > 0 ||
	__HAL_TIM_GET_COMPARE(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL) > 0) ?
					PWM_RUNNING : PWM_STOPPED;
}

// 清除PWM输出
void Clear_PWM(void) {
	__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, POSITIVE_ACTUATOR_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL, 0);
}

// 设置PWM值
void Set_PWM_Value(int16_t combined_value) {
	// 限制 combined_value 在 PWM 范围内
	if (combined_value > 0) {
		combined_value = (combined_value > MAX_PWM) ? MAX_PWM : combined_value;
		__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, POSITIVE_ACTUATOR_CHANNEL,
						combined_value
										* PWM_Global_Parameters.pwm_gain_multiple);
		__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL, 0);
	} else if (combined_value < 0) {
		combined_value =
						(combined_value < -MAX_PWM) ? -MAX_PWM : combined_value;
		__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL,
						-combined_value
										* PWM_Global_Parameters.pwm_gain_multiple);
		__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, POSITIVE_ACTUATOR_CHANNEL, 0);
	} else {
		Clear_PWM();
	}
}
void Update_Time_Setp(uint8_t pb_pool_index) {
	PB_pool[pb_pool_index].time_step++;
	// 如果达到一个完整的周期，重置时间步
	if (PB_pool[pb_pool_index].time_step
					>= PB_pool[pb_pool_index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period) {
		PB_pool[pb_pool_index].time_step = 0; // 重置时间步
	}
}
// 合成效果到PWM
void Synthesize_Effects_To_PWM() {
	int32_t total_force = 0; // 初始化总力
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if ((PB_pool[i].effect_state == EFFECT_RUNNING)
						&& (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type
										!= UNUSED_TYPE_SPECIFIC_EFFECT_TYPE)) { // 效果正在运行 并且 特殊效果不是空
			int32_t calculated_force = 0;
			int16_t angle_deviation = 0;
			int16_t angle_dead_band = 0;
			int16_t previous_angle = 0;
			uint32_t current_time = 0;
			float_t dt = 0;
			// 处理不同类型的效果
			switch (PB_pool[i].effect_parameter.effect_type) {
			case CONSTANT_FORCE:
				calculated_force +=
								-PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.constant_force_parameters.magnitude;
//				if (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].type
//								!= UNUSED_TYPE_SPECIFIC_EFFECT_TYPE) {
//					current_time = HAL_GetTick(); // 获取开始时间
//					dt = (current_time - envelope_last_time) / 1000.0f;
//					envelope_last_time=current_time;
//					float attack_increment =
//									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level
//													/ (float) PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_time;
//					float fade_increment =
//									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level
//													/ (float) PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_time;
//
//					// 攻击阶段
//					if (calculated_force
//									< PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level) {
//						calculated_force += attack_increment * dt;
//						if (calculated_force
//										> PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level) {
//							calculated_force =
//											PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level;
//						}
//					}
//					// 衰退阶段
//					else {
//						calculated_force -= fade_increment * dt;
//						if (calculated_force
//										< PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level) {
//							calculated_force =
//											PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level;
//						}
//					}
//
//				}
				break;
			case RAMP:
				// WARNING: 效果未经测试
				if (PB_pool[i].time_step
								>= PB_pool[i].effect_parameter.duration) {
					calculated_force +=
									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.ramp_force_parameters.ramp_end;
				}
				calculated_force +=
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.ramp_force_parameters.ramp_start
												+ ((PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.ramp_force_parameters.ramp_end
																- PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.ramp_force_parameters.ramp_start)
																* PB_pool[i].time_step)
																/ PB_pool[i].effect_parameter.duration;
				Update_Time_Setp(i);
				break;
			case SAWTOOTH_UP:
				//				uint32_t phase_time =
				//								PB_pool[i].time_step
				//												% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period; // 当前周期内的时间步长
				calculated_force +=
								(PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude
												* (PB_pool[i].time_step
																% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period))
												/ PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period;

				Update_Time_Setp(i);
				break;
			case SAWTOOTH_DOWN:
				//				uint32_t phase_time =
				//								(PB_pool[i].time_step
				//												% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period); // 当前周期内的时间步长
				calculated_force +=
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude
												- ((PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude
																* (PB_pool[i].time_step
																				% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period))
																/ PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period);

				Update_Time_Setp(i);
				break;
			case SINE:
				// WARNING: 效果未经测试
				// 计算与中心点的偏差 单位：度
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.offset
																* Steering_Wheel.rotation_range
																/ 20000);
				calculated_force +=
								(int16_t) angle_deviation
												+ (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude
																* sin(
																				2
																								* M_PI
																								* PB_pool[i].time_step
																								/ PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period));
				Update_Time_Setp(i);

				break;

			case SQUARE:
				// WARNING: 效果未经测试
				// 计算与中心点的偏差 单位：度
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.offset
																* Steering_Wheel.rotation_range
																/ 20000);

				uint16_t half_period =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period
												/ 2;

				// 计算当前时间步，考虑相位偏移
				uint16_t adjusted_time_step =
								(PB_pool[i].time_step
												+ PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.phase)
												% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period;

				// 方波逻辑
				if (adjusted_time_step < half_period) {
					calculated_force +=
									(PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude
													+ angle_deviation);
				} else {
					calculated_force +=
									(-PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude
													+ angle_deviation);
				}

				Update_Time_Setp(i);

				break;

			case TRIANGLE:
				// WARNING: 效果未经测试
				// 计算与中心点的偏差 单位：度
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.offset
																* Steering_Wheel.rotation_range
																/ 20000);

				uint16_t period =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period;
				int16_t triangle_value = ((2 * PB_pool[i].time_step)
								% (2 * period)) - period;
				calculated_force +=
								angle_deviation
												+ ((triangle_value
																* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude)
																/ period);
				Update_Time_Setp(i);
				break;

			case SPRING:
//				Steering_Wheel.current_angle =
//										(float_t) (Steering_Wheel.encoder.pulse_count
//														* 0.15f);
//				// 计算与中心点的偏差 单位：度
//				int16_t angle_deviation =
//								Steering_Wheel.current_angle
//												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset
//																* Steering_Wheel.rotation_range
//																/ 65534);
//
//				// 计算弹簧力
//				int32_t spring_force = 0;
//				//死区，单位：度
//				int16_t angle_dead_band =
//								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
//												* Steering_Wheel.rotation_range
//												/ 20000;
//				// 根据偏差和正负系数计算弹簧力
//				if (angle_deviation > angle_dead_band+10) { //当方向盘在正方向区，如果大于死区，就加反向力
//					spring_force =
//									(angle_deviation-10) *1000
//													* -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient/10000;
//				} else if (angle_deviation < -angle_dead_band-10) { //当方向盘在反方向区，如果小于死区，就加反向力
//					spring_force =
//									(angle_deviation+10) * 1000
//													* -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient/10000;
//				}
//				// 限制弹簧力在饱和范围内
//				if (spring_force
//								> PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation) {
//					spring_force =
//									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation;
//				} else if (spring_force
//								< -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation) {
//					spring_force =
//									-PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation;
//				}
//
//				// 添加到总力中
//				calculated_force += spring_force;

//				Steering_Wheel.current_angle =
//								(float_t) (Steering_Wheel.encoder.pulse_count
//												* 0.15f);
				current_time = HAL_GetTick(); // 获取开始时间
				dt = (current_time - pid_last_time) / 1000.0f;

				// 计算与中心点的偏差 单位：度
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset
																* Steering_Wheel.rotation_range
																/ 65534);

				// 计算死区
				angle_dead_band =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
												* Steering_Wheel.rotation_range
												/ 20000;

				// 根据偏差和正负系数计算弹簧力
				if (angle_deviation > angle_dead_band) { //当方向盘在正方向区，如果大于死区，就加反向力

					// 应用PID控制
					float_t pid_output = PID_Compute(&spring_pid,
									angle_dead_band, (float_t) angle_deviation,
									dt);
					//最终力=PID调制后的角度*弹性系数
					calculated_force +=
									(int32_t) (pid_output
													* Effect_Gain_Controller.spring_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient
													/ 10000);

				} else if (angle_deviation < -angle_dead_band) { //当方向盘在反方向区，如果小于死区，就加反向力

					// 应用PID控制
					float_t pid_output = PID_Compute(&spring_pid,
									-angle_dead_band, (float_t) angle_deviation,
									dt);
					//最终力=PID调制后的角度*弹性系数
					calculated_force +=
									(int32_t) (pid_output
													* Effect_Gain_Controller.spring_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient
													/ 10000);

				}

				pid_last_time = current_time;
				if (calculated_force
								> PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation) {
					calculated_force =
									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation;
				} else if (calculated_force
								< -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation) {
					calculated_force =
									-PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation;
				}
				break;

			case DAMPER:
				// 计算与中心点的偏差 单位：度
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset
																* Steering_Wheel.rotation_range
																/ 65534);

				// 计算死区
				angle_dead_band =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
												* Steering_Wheel.rotation_range
												/ 20000;

				// 根据偏差和正负系数计算阻尼
				if (angle_deviation > angle_dead_band) { //当方向盘在正方向区，如果大于死区，就加反向力
					//最终力=角速度*阻尼系数
					calculated_force +=
									(int32_t) (-Steering_Wheel.angular_velocity
													* Effect_Gain_Controller.damper_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient
													/ 10000);
				} else if (angle_deviation < -angle_dead_band) { //当方向盘在反方向区，如果小于死区，就加反向力
					//最终力=角速度*阻尼系数
					calculated_force +=
									(int32_t) (-Steering_Wheel.angular_velocity
													* Effect_Gain_Controller.damper_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient
													/ 10000);
				}

				if (calculated_force
								> PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation) {
					calculated_force =
									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation;
				} else if (calculated_force
								< -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation) {
					calculated_force =
									-PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation;
				}

				break;

			case FRICTION:
				int16_t angle_offset = Steering_Wheel.current_angle
								- previous_angle;
				previous_angle = Steering_Wheel.current_angle;
				// 计算与中心点的偏差 单位：度
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset
																* Steering_Wheel.rotation_range
																/ 65534);

				// 计算死区
				angle_dead_band =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
												* Steering_Wheel.rotation_range
												/ 20000;

				// 根据偏差和正负系数计算摩擦
				if (angle_deviation > angle_dead_band) { //当方向盘在正方向区，如果大于死区，就加反向力
					//最终力=角度偏移量*摩擦系数
					calculated_force +=
									(int32_t) (-angle_offset
													* Effect_Gain_Controller.friction_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient
													/ 10000);
				} else if (angle_deviation < -angle_dead_band) { //当方向盘在反方向区，如果小于死区，就加反向力
					//最终力=角度偏移量*摩擦系数
					calculated_force +=
									(int32_t) (-angle_offset
													* Effect_Gain_Controller.friction_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient
													/ 10000);
				}

				if (calculated_force
								> PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation) {
					calculated_force =
									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation;
				} else if (calculated_force
								< -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation) {
					calculated_force =
									-PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation;
				}

				break;

			case INERTIA:
				if (fabs(Steering_Wheel.angular_acceleration)
								> Effect_Limiter.inertia_limiter) {
					//必须大于限制器才能计算惯性，否则认为没有惯性
					// 计算死区
					angle_dead_band =
									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
													* Steering_Wheel.rotation_range
													/ 20000;

					// 根据偏差和正负系数计算惯性
					if (Steering_Wheel.current_angle > angle_dead_band) { //当方向盘在正方向区，如果大于死区，就加反向力
						//最终力=角加速度*惯性系数
						calculated_force +=
										(int32_t) (Steering_Wheel.angular_velocity
														* Effect_Gain_Controller.inertia_gain
														* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient
														/ 10000);
					} else if (Steering_Wheel.current_angle
									< -angle_dead_band) { //当方向盘在反方向区，如果小于死区，就加反向力
						//最终力=角加速度*惯性系数
						calculated_force +=
										(int32_t) (Steering_Wheel.angular_velocity
														* Effect_Gain_Controller.inertia_gain
														* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient
														/ 10000);
					}

					if (calculated_force
									> PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation) {
						calculated_force =
										PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation;
					} else if (calculated_force
									< -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation) {
						calculated_force =
										-PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation;
					}
				}
				break;

			default:
				break;

			}

			// 添加增益
			calculated_force = (calculated_force
							* PB_pool[i].effect_parameter.gain) / 10000;
			total_force += calculated_force;

			// 检查是否要暂停
			if (PB_pool[i].sample_period_timer != NULL) {
				PB_pool[i].effect_state = EFFECT_WAITING; // 等待下次可以运行的时刻
			}
		}
	}
	if (total_force != 0) {
		// 全局增益
		total_force = (total_force * Synthesizer_Global_Gain) / 10000;

		if (total_force > MAX_FORCE) {
			total_force =
			MAX_FORCE;
		} else if (total_force < MIN_FORCE) {
			total_force =
			MIN_FORCE;
		}

		// 转换为PWM值
		float_t pwm_value_float = ((float_t) total_force / (float_t) MAX_FORCE)
						* MAX_PWM;
		Set_PWM_Value((int16_t) pwm_value_float);
	} else if (Is_PWM_Running()) {
		Clear_PWM();
	}
}
