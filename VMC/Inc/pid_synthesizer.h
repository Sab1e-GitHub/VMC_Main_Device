/*
 * pid_synthesizer.h
 *
 *  Created on: Oct 28, 2024
 *      Author: Sab1e
 */

#ifndef PID_SYNTHESIZER_H_
#define PID_SYNTHESIZER_H_

#define PWM_RUNNING 0x01U
#define PWM_STOPPED 0x00U

#define MAX_FORCE 10000
#define MIN_FORCE -10000

typedef enum{
	SYNTHESZIER_PAUSE,SYNTHESIZER_RUNNING
}Synthesizer_StatusTypeDef;

typedef struct {
	float_t Kp;         // 比例增益
	float_t Ki;         // 积分增益
	float_t Kd;         // 微分增益
	float_t previous_error; // 上一次误差
	float_t integral;   // 积分值
} PID_Controller;

typedef struct{
	uint16_t spring_gain;
	uint16_t damper_gain;
	uint16_t friction_gain;
	uint16_t inertia_gain;
}Effect_Gain_Controller_t;

typedef struct{
	float_t inertia_limiter;
}Effect_Limiter_t;

typedef struct {
	float_t pwm_gain_multiple;			//PWM增益倍数
} PWM_Global_Parameters_t;

extern uint16_t Synthesizer_Global_Gain;
extern float_t Spring_Kp;
extern float_t Spring_Ki;
extern float_t Spring_Kd;
extern PID_Controller spring_pid;

extern Effect_Gain_Controller_t Effect_Gain_Controller;
extern Effect_Limiter_t Effect_Limiter;
extern PWM_Global_Parameters_t PWM_Global_Parameters;

uint8_t Is_PWM_Running(void);
void Clear_PWM(void);
void Synthesize_Effects_To_PWM(void);
void Set_PWM_Value(int16_t combined_value) ;
void Spring_PID_Init(PID_Controller *pid, float_t Kp, float_t Ki, float_t Kd);
#endif /* PID_SYNTHESIZER_H_ */
