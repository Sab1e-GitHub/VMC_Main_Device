/*
 * pid_dc.h
 *
 *  Created on: Oct 26, 2024
 *      Author: Sab1e
 */

#ifndef PID_ACTUATOR_H_
#define PID_ACTUATOR_H_

#define DC_SUCCESS 1
#define DC_FAIL 0
#define ACTUATOR_TIM htim2
#define POSITIVE_ACTUATOR_CHANNEL TIM_CHANNEL_1
#define NEGATIVE_ACTUATOR_CHANNEL TIM_CHANNEL_3
#define DC_EFFECT_PAUSED 1


typedef enum {
	ACTUATOR_STOPPED, ACTUATOR_RUNNING, ACTUATOR_PAUSED
} Actuator_StatusTypeDef;

extern TIM_HandleTypeDef htim2;
extern uint8_t Actuator_Status;
extern uint8_t DC_Effect_Status;
void DC_Enable_Actuators(void);
void DC_Disable_Actuators(void);
uint8_t DC_Effect_Stop_All(void);
uint8_t DC_Reset(void);
uint8_t DC_Effect_Pause(void);
uint8_t DC_Effect_Continue(void);
#endif /* PID_ACTUATOR_H_ */
