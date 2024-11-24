/*
 * pid_dc.c
 *
 *  Created on: Oct 26, 2024
 *      Author: Sab1e
 *
 *      定义执行器及其相关功能的函数实现
 */

#include "global.h"

#define INIT_PULSE_VALUE 0

uint8_t Actuator_Status = ACTUATOR_STOPPED;
uint8_t DC_Effect_Status = 0;
/**
 * @brief 启用执行器
 * @param 无参数
 * @return 无返回
 */
void DC_Enable_Actuators(void) {
	Actuator_Status = ACTUATOR_RUNNING;

	HAL_GPIO_WritePin(GPIO_PWM1, GPIO_PIN_PWM1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PWM2, GPIO_PIN_PWM2, GPIO_PIN_SET);

	HAL_TIM_PWM_Start(&ACTUATOR_TIM, POSITIVE_ACTUATOR_CHANNEL);
	HAL_TIM_PWM_Start(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL);

	__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, POSITIVE_ACTUATOR_CHANNEL,
					INIT_PULSE_VALUE);
	__HAL_TIM_SET_COMPARE(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL,
					INIT_PULSE_VALUE);

}
/**
 * @brief 禁用执行器
 * @param 无参数
 * @return 无返回
 */
void DC_Disable_Actuators(void) {
	Actuator_Status = ACTUATOR_STOPPED;

	HAL_GPIO_WritePin(GPIO_PWM1, GPIO_PIN_PWM1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_PWM2, GPIO_PIN_PWM2, GPIO_PIN_RESET);

	HAL_TIM_PWM_Stop(&ACTUATOR_TIM, POSITIVE_ACTUATOR_CHANNEL);
	HAL_TIM_PWM_Stop(&ACTUATOR_TIM, NEGATIVE_ACTUATOR_CHANNEL);
}

uint8_t DC_Effect_Stop_All(void) {
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		PB_pool[i].effect_state = EFFECT_STOPPED;		//全部暂停
	}
	if (Delete_All_Timers() == TM_OK) {
		return DC_SUCCESS;
	} else {
		return DC_FAIL;
	}
}
uint8_t DC_Reset(void) {
	//删除定时器
	if (Delete_All_Timers() != TM_OK) {
		return DC_FAIL;
	}
	//清空PB_Pool
	memset(PB_pool, 0, sizeof(PB_pool));  // 可能需要修改代码，让其能够实现完全初始化

	DC_Enable_Actuators();
	return DC_SUCCESS;
}

//暂停所有效果
uint8_t DC_Effect_Pause(void) {
	DC_Effect_Status = DC_EFFECT_PAUSED;
	if (Pause_All_Timers() == TM_OK) {
		return DC_SUCCESS;
	} else {
		return DC_FAIL;
	}

}
//恢复所有效果
uint8_t DC_Effect_Continue(void) {
	DC_Effect_Status = 0;
	if (Resume_All_Timers() == TM_OK) {
		return DC_SUCCESS;
	} else {
		return DC_FAIL;
	}
}
