/*
 * pid_synthesizer.c
 *
 *  Created on: Oct 28, 2024
 *      Author: Sab1e
 */
#include "global.h"

#define MAX_PWM ACTUATOR_TIM.Init.Period // �趨 PWM �����ֵ
#define MIN_PWM 0

#define DEBUG_SAMPLE_RATE_MAX 1000

uint16_t Synthesizer_Global_Gain = 0;
uint32_t pid_last_time = 0; // ��ȡ��ʼʱ��
uint32_t envelope_last_time = 0;

Effect_Gain_Controller_t Effect_Gain_Controller = { .spring_gain = 100,
				.damper_gain = 50, .friction_gain = 100, .inertia_gain = 10 };

Effect_Limiter_t Effect_Limiter = { .inertia_limiter = 20 };

PWM_Global_Parameters_t PWM_Global_Parameters = { .pwm_gain_multiple = 1.0f };

// PID������ʵ��
PID_Controller spring_pid;

// �ںϳ�Ч��֮ǰ��ʼ��PID
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

//PID������ǽǶ�
float_t PID_Compute(PID_Controller *pid, float_t setpoint,
				float_t measured_value, float_t dt) {
	float_t error = setpoint - measured_value;
	pid->integral += error * dt;
	float_t derivative = error - pid->previous_error;
	pid->previous_error = error;

	return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

// ���PWM�Ƿ���������
uint8_t Is_PWM_Running(void) {
	return (__HAL_TIM_GET_COMPARE(&ACTUATOR_TIM,
					POSITIVE_ACTUATOR_CHANNEL) > 0 ||
	__HAL_TIM_GET_COMPARE(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL) > 0) ?
					PWM_RUNNING : PWM_STOPPED;
}

// ���PWM���
void Clear_PWM(void) {
	__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, POSITIVE_ACTUATOR_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL, 0);
}

// ����PWMֵ
void Set_PWM_Value(int16_t combined_value) {
	// ���� combined_value �� PWM ��Χ��
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
	// ����ﵽһ�����������ڣ�����ʱ�䲽
	if (PB_pool[pb_pool_index].time_step
					>= PB_pool[pb_pool_index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period) {
		PB_pool[pb_pool_index].time_step = 0; // ����ʱ�䲽
	}
}
// �ϳ�Ч����PWM
void Synthesize_Effects_To_PWM() {
	int32_t total_force = 0; // ��ʼ������
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if ((PB_pool[i].effect_state == EFFECT_RUNNING)
						&& (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type
										!= UNUSED_TYPE_SPECIFIC_EFFECT_TYPE)) { // Ч���������� ���� ����Ч�����ǿ�
			int32_t calculated_force = 0;
			int16_t angle_deviation = 0;
			int16_t angle_dead_band = 0;
			int16_t previous_angle = 0;
			uint32_t current_time = 0;
			float_t dt = 0;
			// ����ͬ���͵�Ч��
			switch (PB_pool[i].effect_parameter.effect_type) {
			case CONSTANT_FORCE:
				calculated_force +=
								-PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.constant_force_parameters.magnitude;
//				if (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].type
//								!= UNUSED_TYPE_SPECIFIC_EFFECT_TYPE) {
//					current_time = HAL_GetTick(); // ��ȡ��ʼʱ��
//					dt = (current_time - envelope_last_time) / 1000.0f;
//					envelope_last_time=current_time;
//					float attack_increment =
//									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level
//													/ (float) PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_time;
//					float fade_increment =
//									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level
//													/ (float) PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_time;
//
//					// �����׶�
//					if (calculated_force
//									< PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level) {
//						calculated_force += attack_increment * dt;
//						if (calculated_force
//										> PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level) {
//							calculated_force =
//											PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level;
//						}
//					}
//					// ˥�˽׶�
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
				// WARNING: Ч��δ������
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
				//												% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period; // ��ǰ�����ڵ�ʱ�䲽��
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
				//												% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period); // ��ǰ�����ڵ�ʱ�䲽��
				calculated_force +=
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude
												- ((PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude
																* (PB_pool[i].time_step
																				% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period))
																/ PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period);

				Update_Time_Setp(i);
				break;
			case SINE:
				// WARNING: Ч��δ������
				// ���������ĵ��ƫ�� ��λ����
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
				// WARNING: Ч��δ������
				// ���������ĵ��ƫ�� ��λ����
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.offset
																* Steering_Wheel.rotation_range
																/ 20000);

				uint16_t half_period =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period
												/ 2;

				// ���㵱ǰʱ�䲽��������λƫ��
				uint16_t adjusted_time_step =
								(PB_pool[i].time_step
												+ PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.phase)
												% PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period;

				// �����߼�
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
				// WARNING: Ч��δ������
				// ���������ĵ��ƫ�� ��λ����
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
//				// ���������ĵ��ƫ�� ��λ����
//				int16_t angle_deviation =
//								Steering_Wheel.current_angle
//												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset
//																* Steering_Wheel.rotation_range
//																/ 65534);
//
//				// ���㵯����
//				int32_t spring_force = 0;
//				//��������λ����
//				int16_t angle_dead_band =
//								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
//												* Steering_Wheel.rotation_range
//												/ 20000;
//				// ����ƫ�������ϵ�����㵯����
//				if (angle_deviation > angle_dead_band+10) { //����������������������������������ͼӷ�����
//					spring_force =
//									(angle_deviation-10) *1000
//													* -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient/10000;
//				} else if (angle_deviation < -angle_dead_band-10) { //���������ڷ������������С���������ͼӷ�����
//					spring_force =
//									(angle_deviation+10) * 1000
//													* -PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient/10000;
//				}
//				// ���Ƶ������ڱ��ͷ�Χ��
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
//				// ��ӵ�������
//				calculated_force += spring_force;

//				Steering_Wheel.current_angle =
//								(float_t) (Steering_Wheel.encoder.pulse_count
//												* 0.15f);
				current_time = HAL_GetTick(); // ��ȡ��ʼʱ��
				dt = (current_time - pid_last_time) / 1000.0f;

				// ���������ĵ��ƫ�� ��λ����
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset
																* Steering_Wheel.rotation_range
																/ 65534);

				// ��������
				angle_dead_band =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
												* Steering_Wheel.rotation_range
												/ 20000;

				// ����ƫ�������ϵ�����㵯����
				if (angle_deviation > angle_dead_band) { //����������������������������������ͼӷ�����

					// Ӧ��PID����
					float_t pid_output = PID_Compute(&spring_pid,
									angle_dead_band, (float_t) angle_deviation,
									dt);
					//������=PID���ƺ�ĽǶ�*����ϵ��
					calculated_force +=
									(int32_t) (pid_output
													* Effect_Gain_Controller.spring_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient
													/ 10000);

				} else if (angle_deviation < -angle_dead_band) { //���������ڷ������������С���������ͼӷ�����

					// Ӧ��PID����
					float_t pid_output = PID_Compute(&spring_pid,
									-angle_dead_band, (float_t) angle_deviation,
									dt);
					//������=PID���ƺ�ĽǶ�*����ϵ��
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
				// ���������ĵ��ƫ�� ��λ����
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset
																* Steering_Wheel.rotation_range
																/ 65534);

				// ��������
				angle_dead_band =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
												* Steering_Wheel.rotation_range
												/ 20000;

				// ����ƫ�������ϵ����������
				if (angle_deviation > angle_dead_band) { //����������������������������������ͼӷ�����
					//������=���ٶ�*����ϵ��
					calculated_force +=
									(int32_t) (-Steering_Wheel.angular_velocity
													* Effect_Gain_Controller.damper_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient
													/ 10000);
				} else if (angle_deviation < -angle_dead_band) { //���������ڷ������������С���������ͼӷ�����
					//������=���ٶ�*����ϵ��
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
				// ���������ĵ��ƫ�� ��λ����
				angle_deviation =
								Steering_Wheel.current_angle
												- (PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset
																* Steering_Wheel.rotation_range
																/ 65534);

				// ��������
				angle_dead_band =
								PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
												* Steering_Wheel.rotation_range
												/ 20000;

				// ����ƫ�������ϵ������Ħ��
				if (angle_deviation > angle_dead_band) { //����������������������������������ͼӷ�����
					//������=�Ƕ�ƫ����*Ħ��ϵ��
					calculated_force +=
									(int32_t) (-angle_offset
													* Effect_Gain_Controller.friction_gain
													* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient
													/ 10000);
				} else if (angle_deviation < -angle_dead_band) { //���������ڷ������������С���������ͼӷ�����
					//������=�Ƕ�ƫ����*Ħ��ϵ��
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
					//����������������ܼ�����ԣ�������Ϊû�й���
					// ��������
					angle_dead_band =
									PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band
													* Steering_Wheel.rotation_range
													/ 20000;

					// ����ƫ�������ϵ���������
					if (Steering_Wheel.current_angle > angle_dead_band) { //����������������������������������ͼӷ�����
						//������=�Ǽ��ٶ�*����ϵ��
						calculated_force +=
										(int32_t) (Steering_Wheel.angular_velocity
														* Effect_Gain_Controller.inertia_gain
														* PB_pool[i].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient
														/ 10000);
					} else if (Steering_Wheel.current_angle
									< -angle_dead_band) { //���������ڷ������������С���������ͼӷ�����
						//������=�Ǽ��ٶ�*����ϵ��
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

			// �������
			calculated_force = (calculated_force
							* PB_pool[i].effect_parameter.gain) / 10000;
			total_force += calculated_force;

			// ����Ƿ�Ҫ��ͣ
			if (PB_pool[i].sample_period_timer != NULL) {
				PB_pool[i].effect_state = EFFECT_WAITING; // �ȴ��´ο������е�ʱ��
			}
		}
	}
	if (total_force != 0) {
		// ȫ������
		total_force = (total_force * Synthesizer_Global_Gain) / 10000;

		if (total_force > MAX_FORCE) {
			total_force =
			MAX_FORCE;
		} else if (total_force < MIN_FORCE) {
			total_force =
			MIN_FORCE;
		}

		// ת��ΪPWMֵ
		float_t pwm_value_float = ((float_t) total_force / (float_t) MAX_FORCE)
						* MAX_PWM;
		Set_PWM_Value((int16_t) pwm_value_float);
	} else if (Is_PWM_Running()) {
		Clear_PWM();
	}
}
