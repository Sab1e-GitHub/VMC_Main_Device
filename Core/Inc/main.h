/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {
	cmd_unknown_command,
	cmd_steering_wheel_set_center,							// 设置当前角度为中心
	cmd_steering_wheel_set_rotation_range,		// 设置方向盘最大旋转角度，正向+反向
	cmd_steering_wheel_encoder_set_pulse,			// 设置编码器脉冲
	cmd_accelerator_pedal_set_maximum,			// 设置当前值为加速踏板的最大值
	cmd_accelerator_pedal_set_minimum,				// 设置当前值为加速踏板的最小值
	cmd_brake_pedal_set_maximum,						// 设置当前值为刹车踏板的最大值
	cmd_brake_pedal_set_minimum,						// 设置当前值为刹车踏板的最小值
	cmd_clutch_pedal_set_maximum,						// 设置当前值为离合器踏板的最大值
	cmd_clutch_pedal_set_minimum,						// 设置当前值为离合器踏板的最小值
	cmd_effect_gain_controller_set_spring_gain,			// 设置力反馈效果 - Spring效果的增益
	cmd_effect_gain_controller_set_damper_gain,		// 设置力反馈效果 - Damper效果的增益
	cmd_effect_gain_controller_set_friction_gain,		// 设置力反馈效果 - Friction弹簧效果的增益
	cmd_effect_gain_controller_set_inertia_gain,			// 设置力反馈效果 - Inertia效果的增益
	cmd_effect_limiter_set_inertia_limiter,						// 设置力反馈效果 - Inertia效果的限位
	cmd_effect_set_spring_kp,		// 设置力反馈效果 - Spring PID的Kp
	cmd_effect_set_spring_ki,		// 设置力反馈效果 - Spring PID的Ki
	cmd_effect_set_spring_kd,		// 设置力反馈效果 - Spring PID的Kd
	cmd_steering_wheel_software_limiter_set_kp,	// 设置方向盘软限位 PID 的Kp
	cmd_steering_wheel_software_limiter_set_ki,	// 设置方向盘软限位 PID 的Ki
	cmd_steering_wheel_software_limiter_set_kd,	// 设置方向盘软限位 PID 的Kd
	cmd_pwm_global_parameters_set_pwm_gain_multiple,		// 设置力反馈输出PWM的增益倍数
	cmd_steering_wheel_software_limiter_set_vibration_feedback_enable,		// 设置方向盘软限位的震动反馈使能
	cmd_steering_wheel_software_limiter_set_vibration_feedback_delay			// 设置方向盘软限位的震动反馈
} VMC_CommandTypeDef;
typedef enum {
	resp_unknown,
	resp_accelerator_pedal_maximum,
	resp_accelerator_pedal_minimum,
	resp_brake_pedal_maximum,
	resp_brake_pedal_minimum,
	resp_clutch_pedal_maximum,
	resp_clutch_pedal_minimum
} VMC_RespondingTypeDef;

typedef enum {
	SL_Vibration_Feedback_OFF,
	SL_Vibration_Feedback_CONSTANT,
} SL_Vibration_FeedbackTypeDef;

typedef struct {
	uint16_t accelerator_pedal_maximum;
	uint16_t accelerator_pedal_minimum;
	uint16_t brake_pedal_set_maximum;
	uint16_t brake_pedal_set_minimum;
	uint16_t clutch_pedal_set_maximum;
	uint16_t clutch_pedal_set_minimum;
} Pedal_Limiter_t;




extern Pedal_Limiter_t Pedal_Limiter;
extern uint32_t adc_values[3];
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DECODER_A_Pin GPIO_PIN_0
#define DECODER_A_GPIO_Port GPIOC
#define DECODER_A_EXTI_IRQn EXTI0_IRQn
#define DECODER_B_Pin GPIO_PIN_1
#define DECODER_B_GPIO_Port GPIOC
#define DECODER_B_EXTI_IRQn EXTI1_IRQn
#define PMM1_EN_Pin GPIO_PIN_2
#define PMM1_EN_GPIO_Port GPIOC
#define PWM2_EN_Pin GPIO_PIN_3
#define PWM2_EN_GPIO_Port GPIOC
#define ADC_ACCELERATOR_Pin GPIO_PIN_0
#define ADC_ACCELERATOR_GPIO_Port GPIOA
#define ADC_BRAKE_Pin GPIO_PIN_1
#define ADC_BRAKE_GPIO_Port GPIOA
#define ADC_CLUTCH_Pin GPIO_PIN_2
#define ADC_CLUTCH_GPIO_Port GPIOA
#define BTN_WPIERS_PULL_Pin GPIO_PIN_3
#define BTN_WPIERS_PULL_GPIO_Port GPIOA
#define SW_BLINK_LEFT_Pin GPIO_PIN_4
#define SW_BLINK_LEFT_GPIO_Port GPIOA
#define SW_BLINK_RIGHT_Pin GPIO_PIN_5
#define SW_BLINK_RIGHT_GPIO_Port GPIOA
#define SW_LIGHT_1_Pin GPIO_PIN_6
#define SW_LIGHT_1_GPIO_Port GPIOA
#define SW_LIGHT_2_Pin GPIO_PIN_7
#define SW_LIGHT_2_GPIO_Port GPIOA
#define SW_LIGHT_3_Pin GPIO_PIN_4
#define SW_LIGHT_3_GPIO_Port GPIOC
#define SW_LIGHT_4_Pin GPIO_PIN_5
#define SW_LIGHT_4_GPIO_Port GPIOC
#define SW_WIPERS_1_Pin GPIO_PIN_0
#define SW_WIPERS_1_GPIO_Port GPIOB
#define SW_WIPERS_2_Pin GPIO_PIN_1
#define SW_WIPERS_2_GPIO_Port GPIOB
#define SW_WIPERS_3_Pin GPIO_PIN_2
#define SW_WIPERS_3_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_10
#define PWM2_GPIO_Port GPIOB
#define SW_H_SHIFTER_1_Pin GPIO_PIN_11
#define SW_H_SHIFTER_1_GPIO_Port GPIOB
#define SW_H_SHIFTER_2_Pin GPIO_PIN_12
#define SW_H_SHIFTER_2_GPIO_Port GPIOB
#define SW_H_SHIFTER_3_Pin GPIO_PIN_13
#define SW_H_SHIFTER_3_GPIO_Port GPIOB
#define SW_H_SHIFTER_4_Pin GPIO_PIN_14
#define SW_H_SHIFTER_4_GPIO_Port GPIOB
#define SW_H_SHIFTER_5_Pin GPIO_PIN_15
#define SW_H_SHIFTER_5_GPIO_Port GPIOB
#define SW_H_SHIFTER_6_Pin GPIO_PIN_6
#define SW_H_SHIFTER_6_GPIO_Port GPIOC
#define SW_H_SHIFTER_R_Pin GPIO_PIN_7
#define SW_H_SHIFTER_R_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_15
#define PWM1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
