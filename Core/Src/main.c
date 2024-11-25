/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "global.h"

extern uint8_t USB_Out_Flag;
extern uint8_t USB_Out_Buffer[USB_OUT_BUFFER_SIZE];

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//发送报告
#define SendReport(report) \
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report))

#define TIM_PERIOD (50 - 1)

#define TOGGLE_LED 		HAL_GPIO_TogglePin(GPIO_LED, GPIO_PIN_LED);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

typedef struct {
	float_t Kp;         // 比例增益
	float_t Ki;         // 积分增益
	float_t Kd;         // 微分增益
	float_t previous_error; // 上一次误差
	float_t integral;   // 积分值
} SL_PID_Controller_t;

typedef struct {
	uint8_t mode;		// 震动模式
	uint16_t delay;		// 震动延迟
	uint8_t coefficient;		// Ramp模式下的震动系数 Ramp模式下震动延迟=系数*角度
} SL_Vibration_Feedback_t;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

osThreadId LEDTaskHandle;
osThreadId USBOutputTaskHandle;
osThreadId SynthesizerTaskHandle;
osThreadId USBInputTaskHandle;
/* USER CODE BEGIN PV */

uint32_t adc_values[3];

uint8_t Current_Report_ID = INPUT_REPORT_ID;

// PID控制器实例
SL_PID_Controller_t sl_pid;
// 在合成效果之前初始化PID
float_t SL_KP = 200.0f;
float_t SL_KI = 1.0f;
float_t SL_KD = 1.0f;
uint32_t sl_pid_last_time = 0; // 获取开始时间

SL_Vibration_Feedback_t SL_Vibration_Feedback = { .mode =
				SL_Vibration_Feedback_CONSTANT, .delay = 200, };

float_t angle_per_pulse = 0;
// 初始化踏板限位
Pedal_Limiter_t Pedal_Limiter = { .accelerator_pedal_maximum = ADC_MAX_VALUE,
				.accelerator_pedal_minimum = 0, .brake_pedal_set_maximum =
				ADC_MAX_VALUE, .brake_pedal_set_minimum = 0,
				.clutch_pedal_set_maximum = ADC_MAX_VALUE,
				.clutch_pedal_set_minimum = 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
void StartLEDTask(void const *argument);
void StartUSBOutputTask(void const *argument);
void StartSynthesizerTask(void const *argument);
void StartUSBInputTask(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// SL = Software  Limiter
void SL_PID_Init(SL_PID_Controller_t *pid, float_t Kp, float_t Ki, float_t Kd) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->previous_error = 0.0f;
	pid->integral = 0.0f;
}

//PID计算的是角度
float_t SL_PID_Compute(SL_PID_Controller_t *pid, float_t setpoint,
				float_t measured_value, float_t dt) {
	float_t error = setpoint - measured_value;
	pid->integral += error * dt;
	float_t derivative = error - pid->previous_error;
	pid->previous_error = error;

	return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		uint16_t raw_accelerator = adc_values[0];
		uint16_t raw_brake = adc_values[1];
		uint16_t raw_clutch = adc_values[2];
		// 处理加速踏板
		if (raw_accelerator < Pedal_Limiter.accelerator_pedal_minimum) {
			raw_accelerator = Pedal_Limiter.accelerator_pedal_minimum; // 限位到最小值
		} else if (raw_accelerator > Pedal_Limiter.accelerator_pedal_maximum) {
			raw_accelerator = Pedal_Limiter.accelerator_pedal_maximum; // 限位到最大值
		}
		// 将限位后的值映射到0~4095
		Input_Report.accelerator =
						(uint16_t) ((float) (raw_accelerator
										- Pedal_Limiter.accelerator_pedal_minimum)
										/ (Pedal_Limiter.accelerator_pedal_maximum
														- Pedal_Limiter.accelerator_pedal_minimum)
										* 4095);

		// 处理刹车踏板
		if (raw_brake < Pedal_Limiter.brake_pedal_set_minimum) {
			raw_brake = Pedal_Limiter.brake_pedal_set_minimum; // 限位到最小值
		} else if (raw_brake > Pedal_Limiter.brake_pedal_set_maximum) {
			raw_brake = Pedal_Limiter.brake_pedal_set_maximum; // 限位到最大值
		}
		// 将限位后的值映射到0~4095
		Input_Report.brake = (uint16_t) ((float) (raw_brake
						- Pedal_Limiter.brake_pedal_set_minimum)
						/ (Pedal_Limiter.brake_pedal_set_maximum
										- Pedal_Limiter.brake_pedal_set_minimum)
						* 4095);

		// 处理离合器踏板
		if (raw_clutch < Pedal_Limiter.clutch_pedal_set_minimum) {
			raw_clutch = Pedal_Limiter.clutch_pedal_set_minimum; // 限位到最小值
		} else if (raw_clutch > Pedal_Limiter.clutch_pedal_set_maximum) {
			raw_clutch = Pedal_Limiter.clutch_pedal_set_maximum; // 限位到最大值
		}
		// 将限位后的值映射到0~4095
		Input_Report.clutch =
						(uint16_t) ((float) (raw_clutch
										- Pedal_Limiter.clutch_pedal_set_minimum)
										/ (Pedal_Limiter.clutch_pedal_set_maximum
														- Pedal_Limiter.clutch_pedal_set_minimum)
										* 4095);
	}
}

uint32_t Read_Keys(void) {
	uint32_t key_state = 0;
	//按下是0，没按下是1
	// 检测 雨刮喷水
	key_state |= (GPIOA->IDR & GPIO_PIN_3) ? 0 : (1 << 0);  // PA3
	// 检测 左转向灯
	key_state |= (GPIOA->IDR & GPIO_PIN_4) ? 0 : (1 << 1);  // PA4
	// 检测 右转向灯
	key_state |= (GPIOA->IDR & GPIO_PIN_5) ? 0 : (1 << 2);  // PA5
	/*=================灯光组合==================*/
	// 发送灯光关闭信号
	key_state |= (GPIOA->IDR & GPIO_PIN_6) && (GPIOA->IDR & GPIO_PIN_7)
					&& (GPIOC->IDR & GPIO_PIN_4) && (GPIOC->IDR & GPIO_PIN_5) ?
					(1 << 3) : 0;
	// 发送示宽灯信号（条件：示宽灯开启，且未开启近光灯）
	key_state |= (!(GPIOC->IDR & GPIO_PIN_4) && (GPIOA->IDR & GPIO_PIN_6)
					&& (GPIOA->IDR & GPIO_PIN_7)) ? (1 << 4) : 0;

	// 发送近光灯信号（条件：近光灯开启）
	key_state |= (!(GPIOA->IDR & GPIO_PIN_6)) ? (1 << 5) : 0;

	// 发送远光灯信号（条件：远光灯开启）
	key_state |= (!(GPIOC->IDR & GPIO_PIN_5)) ? (1 << 6) : 0;
	/*==================雨刮器组合==================*/
	// 雨刮器关闭
	key_state |= (!(GPIOB->IDR & GPIO_PIN_0)) && (GPIOB->IDR & GPIO_PIN_1)
					&& (GPIOB->IDR & GPIO_PIN_2) ? (1 << 7) : 0;  // PB0
	// 雨刮器一档
	key_state |= (!(GPIOB->IDR & GPIO_PIN_1)) ? (1 << 8) : 0;  // PB1
	// 雨刮器二挡
	key_state |= (GPIOB->IDR & GPIO_PIN_0) && (GPIOB->IDR & GPIO_PIN_1)
					&& (!(GPIOB->IDR & GPIO_PIN_2)) ? (1 << 9) : 0;  // PB2
	// 雨刮器三挡
	key_state |= (GPIOB->IDR & GPIO_PIN_2) && (GPIOB->IDR & GPIO_PIN_0)
					&& (GPIOB->IDR & GPIO_PIN_1) ? (1 << 10) : 0;  // PB2
	/*==================H档位组合==================*/
	// 检测 PB11
	key_state |= (GPIOB->IDR & GPIO_PIN_11) ? 0 : (1 << 11);  // PB11
	// 检测 PB12
	key_state |= (GPIOB->IDR & GPIO_PIN_12) ? 0 : (1 << 12);  // PB12
	// 检测 PB13
	key_state |= (GPIOB->IDR & GPIO_PIN_13) ? 0 : (1 << 13);  // PB13
	// 检测 PB14
	key_state |= (GPIOB->IDR & GPIO_PIN_14) ? 0 : (1 << 14);  // PB14
	// 检测 PB15
	key_state |= (GPIOB->IDR & GPIO_PIN_15) ? 0 : (1 << 15);  // PB15

	// 检测 PC6
	key_state |= (GPIOC->IDR & GPIO_PIN_6) ? 0 : (1 << 16);  // PC6
	// 检测 PC7
	key_state |= (GPIOC->IDR & GPIO_PIN_7) ? 0 : (1 << 17);  // PC7

	return key_state;
}

/**
 * @brief 处理编码器的引脚状态发生变化时的回调
 * @param GPIO_Pin 触发回调的引脚
 * @return 无返回
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint8_t current_state;

	if (GPIO_Pin == GPIO_PIN_DECODER_A || GPIO_Pin == GPIO_PIN_DECODER_B) {
		current_state = ((GPIO_DECODER_A->IDR & GPIO_PIN_DECODER_A) ? 1 : 0)
						<< 1
						| (GPIO_DECODER_B->IDR & GPIO_PIN_DECODER_B ? 1 : 0);

		// 根据状态变化更新脉冲计数
		switch (Steering_Wheel.encoder.previous_state) {
		case 0b00:
			if (current_state == 0b01)
				Steering_Wheel.encoder.pulse_count++;  // A上升
			else if (current_state == 0b10)
				Steering_Wheel.encoder.pulse_count--; // B上升
			break;
		case 0b01:
			if (current_state == 0b11)
				Steering_Wheel.encoder.pulse_count++;  // A保持，B上升
			else if (current_state == 0b00)
				Steering_Wheel.encoder.pulse_count--; // B下降
			break;
		case 0b11:
			if (current_state == 0b10)
				Steering_Wheel.encoder.pulse_count++;  // B下降
			else if (current_state == 0b01)
				Steering_Wheel.encoder.pulse_count--; // A下降
			break;
		case 0b10:
			if (current_state == 0b00)
				Steering_Wheel.encoder.pulse_count++;  // B保持，A下降
			else if (current_state == 0b11)
				Steering_Wheel.encoder.pulse_count--; // A上升
			break;
		}

		Steering_Wheel.encoder.previous_state = current_state;  // 更新状态
	}
}

// 通用的函数：设置 uint16 类型并限制范围
void CMD_Set_uint16(uint16_t *param, uint16_t min, uint16_t max,
				uint16_t *target) {
	*param = (*param < min) ? min : (*param > max) ? max : *param;
	*target = *param;
}

// 通用的函数：设置 float 类型并限制范围
void CMD_Set_float_from_buffer(uint8_t *USB_Out_Buffer, size_t start_idx,
				float_t min, float_t max, float_t *target) {
	// 从 USB_Out_Buffer 中读取 4 个字节到浮点数
	float_t parameter;
	memcpy(&parameter, &USB_Out_Buffer[start_idx], sizeof(parameter));

	// 限制 parameter 在 min 和 max 之间
	*target = (parameter < min) ? min : (parameter > max) ? max : parameter;
}

// 用于从字节数组获取参数的函数
uint64_t Get_uint64_from_buffer(uint8_t *USB_Out_Buffer, size_t start_idx) {
	return USB_Out_Buffer[start_idx] | (USB_Out_Buffer[start_idx + 1] << 8)
					| (USB_Out_Buffer[start_idx + 2] << 16)
					| (USB_Out_Buffer[start_idx + 3] << 24);
}
/*==================软限位使用的PID==================*/

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIO_PWM1, GPIO_PIN_PWM1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_PWM2, GPIO_PIN_PWM2, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim4); // 启动定时器并使能中断
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of LEDTask */
	osThreadDef(LEDTask, StartLEDTask, osPriorityNormal, 0, 128);
	LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

	/* definition and creation of USBOutputTask */
	osThreadDef(USBOutputTask, StartUSBOutputTask, osPriorityNormal, 0, 512);
	USBOutputTaskHandle = osThreadCreate(osThread(USBOutputTask), NULL);

	/* definition and creation of SynthesizerTask */
	osThreadDef(SynthesizerTask, StartSynthesizerTask, osPriorityNormal, 0,
					256);
	SynthesizerTaskHandle = osThreadCreate(osThread(SynthesizerTask), NULL);

	/* definition and creation of USBInputTask */
	osThreadDef(USBInputTask, StartUSBInputTask, osPriorityNormal, 0, 512);
	USBInputTaskHandle = osThreadCreate(osThread(USBInputTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
//  Start_ADC_DMA();
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
					| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 72 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
					!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
					!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
					!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 3000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
					!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
					!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
					!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 7200 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1500 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
					!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, PMM1_EN_Pin | PWM2_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : DECODER_A_Pin DECODER_B_Pin */
	GPIO_InitStruct.Pin = DECODER_A_Pin | DECODER_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PMM1_EN_Pin PWM2_EN_Pin */
	GPIO_InitStruct.Pin = PMM1_EN_Pin | PWM2_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_WPIERS_PULL_Pin SW_BLINK_LEFT_Pin SW_BLINK_RIGHT_Pin SW_LIGHT_1_Pin
	 SW_LIGHT_2_Pin */
	GPIO_InitStruct.Pin = BTN_WPIERS_PULL_Pin | SW_BLINK_LEFT_Pin
					| SW_BLINK_RIGHT_Pin | SW_LIGHT_1_Pin | SW_LIGHT_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_LIGHT_3_Pin SW_LIGHT_4_Pin SW_H_SHIFTER_6_Pin SW_H_SHIFTER_R_Pin */
	GPIO_InitStruct.Pin = SW_LIGHT_3_Pin | SW_LIGHT_4_Pin | SW_H_SHIFTER_6_Pin
					| SW_H_SHIFTER_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_WIPERS_1_Pin SW_WIPERS_2_Pin SW_WIPERS_3_Pin SW_H_SHIFTER_1_Pin
	 SW_H_SHIFTER_2_Pin SW_H_SHIFTER_3_Pin SW_H_SHIFTER_4_Pin SW_H_SHIFTER_5_Pin */
	GPIO_InitStruct.Pin = SW_WIPERS_1_Pin | SW_WIPERS_2_Pin | SW_WIPERS_3_Pin
					| SW_H_SHIFTER_1_Pin | SW_H_SHIFTER_2_Pin
					| SW_H_SHIFTER_3_Pin | SW_H_SHIFTER_4_Pin
					| SW_H_SHIFTER_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLEDTask */
/**
 * @brief  Function implementing the LEDTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void const *argument) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	uint32_t duty_cycle = 0;   // 当前占空比值
	int32_t step = 10;         // 每次调整的步长
	/* Infinite loop */
	for (;;) {
		osDelay(15);
		// 设置当前占空比
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);

		// 调整占空比
		duty_cycle += step;

		// 检查是否达到上下限，改变方向
		if (duty_cycle >= htim1.Init.Period || duty_cycle <= 0) {
			step = -step; // 反向调整
		}
//		Calculate_Instantaneous_Angular_Velocity(0.001f);
//		SendReport(Effect_Parameter_Block_Load_Report);

//		SendReport(Input_Report);
//		Input_Report.buttons+=1;
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUSBOutputTask */
/**
 * @brief Function implementing the USBOutputTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUSBOutputTask */
void StartUSBOutputTask(void const *argument) {
	/* USER CODE BEGIN StartUSBOutputTask */
	/* Infinite loop */
	for (;;) {
		if (USB_Out_Flag) {

			switch (USB_Out_Buffer[0]) {
			case SET_EFFECT_REPORT_ID:
				// 设置Effect Types报告
				Set_Effect_Parameter((uint8_t) USB_Out_Buffer[1],
								(Effect_TypeDef) USB_Out_Buffer[2],
								(uint16_t) (fabs(
												(USB_Out_Buffer[3]
																| (USB_Out_Buffer[4]
																				<< 8)))
												>= 30001 ?
												DURATION_INFINITE :
												(USB_Out_Buffer[3]
																| (USB_Out_Buffer[4]
																				<< 8))),
								(uint8_t) USB_Out_Buffer[5],
								(uint8_t) USB_Out_Buffer[6],
								(uint16_t) (USB_Out_Buffer[7]
												| (USB_Out_Buffer[8] << 8)),
								(uint8_t) USB_Out_Buffer[9],
								(uint8_t) USB_Out_Buffer[10],
								(uint8_t) USB_Out_Buffer[11],
								(uint8_t) USB_Out_Buffer[12]);

				break;

			case ENVELOPE_REPORT_ID:
				// 设置Envelope报告

				Add_Envelope_PB((uint8_t) USB_Out_Buffer[1],
								(uint8_t) USB_Out_Buffer[2],
								(uint8_t) USB_Out_Buffer[3],
								(uint8_t) USB_Out_Buffer[4],
								(uint8_t) USB_Out_Buffer[5]);
				break;

			case CONDITION_REPORT_ID:
				// 设置Condition报告
				Add_Condition_PB((uint8_t) USB_Out_Buffer[1],
								(int16_t) (USB_Out_Buffer[2]
												| (USB_Out_Buffer[3] << 8)),
								(int16_t) (USB_Out_Buffer[4]
												| (USB_Out_Buffer[5] << 8)),
								(int16_t) (USB_Out_Buffer[6]
												| (USB_Out_Buffer[7] << 8)),
								(uint16_t) (USB_Out_Buffer[8]
												| (USB_Out_Buffer[9] << 8)),
								(uint16_t) (USB_Out_Buffer[10]
												| (USB_Out_Buffer[11] << 8)),
								(uint16_t) (USB_Out_Buffer[12]
												| (USB_Out_Buffer[13] << 8)));
				break;

			case PERIODIC_REPORT_ID:
				// 设置Periodic报告
				Add_Periodic_PB((uint8_t) USB_Out_Buffer[1],
								(uint8_t) USB_Out_Buffer[2],
								(uint16_t) (USB_Out_Buffer[3]
												| (USB_Out_Buffer[4] << 8)),
								(int16_t) (USB_Out_Buffer[5]
												| (USB_Out_Buffer[6] << 8)),
								(uint16_t) (USB_Out_Buffer[7]
												| (USB_Out_Buffer[8] << 8)));
				break;

			case CONSTANT_FORCE_REPORT_ID:
				// 设置Constant-Force报告
				Add_Constant_Force_PB((uint8_t) USB_Out_Buffer[1],
								(int16_t) (USB_Out_Buffer[2]
												| (USB_Out_Buffer[3] << 8)));
				break;
			case RAMP_FROCE_REPORT_ID:
				Add_Ramp_Force_PB((uint8_t) USB_Out_Buffer[1],
								(uint16_t) (USB_Out_Buffer[2]
												| (USB_Out_Buffer[3] << 8)),
								(uint16_t) (USB_Out_Buffer[4]
												| (USB_Out_Buffer[5] << 8)));
				break;
			case EFFECT_OPERATION_REPORT_ID:
				//设置Effect的操作报告
				if (USB_Out_Buffer[2] == 1) {
					//Op Effect Begin
					OP_Effect_Start(USB_Out_Buffer[1], USB_Out_Buffer[3]);
				} else if (USB_Out_Buffer[2] == 2) {
					//Op Effect Begin Solo
					OP_Effect_Start_Solo(USB_Out_Buffer[1], USB_Out_Buffer[3]);
				} else if (USB_Out_Buffer[2] == 3) {
					//Op Effect Stop
					OP_Effect_Stop(USB_Out_Buffer[1]);
				}
				break;
			case PID_DEVICE_CONTROL_REPORT_ID:
				//设置PID Device Control

				if ((USB_Out_Buffer[1] >> 0) & 1) {
					//启动所有设备执行器
					DC_Enable_Actuators();
				} else if ((USB_Out_Buffer[1] >> 1) & 1) {
					//关闭所有设备执行器
					DC_Disable_Actuators();
				} else if ((USB_Out_Buffer[1] >> 2) & 1) {
					//停止所有正在进行的Effect
					DC_Effect_Stop_All();
				} else if ((USB_Out_Buffer[1] >> 3) & 1) {
					//清除任何设备暂停状态，启用所有执行器并清除内存中的所有效果。
					DC_Reset();
				} else if ((USB_Out_Buffer[1] >> 4) & 1) {
					//设备上的所有效果都会在当前帧暂停。
					DC_Effect_Pause();
				} else if ((USB_Out_Buffer[1] >> 5) & 1) {
					//设备暂停时运行的所有效果都会从上一个帧重新启动。
					DC_Effect_Continue();
				}

				break;
				//			case CREATE_NEW_EFFECT_PARAMETER_BLOCK_ID:
				//				// 配置Create New Effect Parameter Block Report
				//				//在usbd_custom_hid_if.c中配置
				//				break;
			case EFFECT_PARAMETER_BLOCK_FREE_REPORT_ID:
				//配置Effect Parameter Block Free Report
				Free_Effect_PB(USB_Out_Buffer[1]);
				break;
//			case EFFECT_PARAMETER_BLOCK_LOAD_REPORT_ID:
//				//配置Effect Parameter Block Load Report
//				//在usbd_custom_hid_if.c中配置
//				break;
			case DEVICE_GAIN_REPORT_ID:
				//配置Device Gain Report
				Synthesizer_Global_Gain = (USB_Out_Buffer[2] << 8)
								| USB_Out_Buffer[1];
				break;
			case VMC_REPORT_ID:
				// VMC参数配置相关
				uint64_t Command = USB_Out_Buffer[1];
				uint64_t Parameter = Get_uint64_from_buffer(USB_Out_Buffer, 2); // 获取从 USB 收到的 4 字节参数

				switch (Command) {
				case cmd_steering_wheel_set_center:
					Steering_Wheel.current_angle = 0;
					Steering_Wheel.encoder.pulse_count = 0;
					break;

				case cmd_steering_wheel_set_rotation_range:
					// 处理最大最小值
					Parameter = (uint16_t) Parameter < 30 ? 30U :
								(uint16_t) Parameter > UINT16_MAX ?
												UINT16_MAX :
												(uint16_t) Parameter;
					Steering_Wheel.rotation_range = (uint16_t) Parameter;
					Steering_Wheel.encoder.max_pulses =
									(Steering_Wheel.rotation_range
													* Steering_Wheel.encoder.pulse)
													/ 180;
					break;

				case cmd_steering_wheel_encoder_set_pulse:
					// 处理最大最小值
					Parameter = (uint16_t) Parameter <= 10 ? 10U :
								(uint16_t) Parameter > UINT16_MAX ?
												UINT16_MAX :
												(uint16_t) Parameter;
					Steering_Wheel.encoder.pulse = (uint16_t) Parameter;
					angle_per_pulse = (float_t) (360.0
									/ (Steering_Wheel.encoder.pulse * 4.0));
					break;

				case cmd_accelerator_pedal_set_maximum:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Pedal_Limiter.accelerator_pedal_maximum);
					break;

				case cmd_accelerator_pedal_set_minimum:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Pedal_Limiter.accelerator_pedal_minimum);
					break;

				case cmd_brake_pedal_set_maximum:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Pedal_Limiter.brake_pedal_set_maximum);
					break;

				case cmd_brake_pedal_set_minimum:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Pedal_Limiter.brake_pedal_set_minimum);
					break;

				case cmd_clutch_pedal_set_maximum:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Pedal_Limiter.clutch_pedal_set_maximum);
					break;

				case cmd_clutch_pedal_set_minimum:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Pedal_Limiter.clutch_pedal_set_minimum);
					break;

				case cmd_effect_gain_controller_set_spring_gain:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Effect_Gain_Controller.spring_gain);
					break;

				case cmd_effect_gain_controller_set_damper_gain:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Effect_Gain_Controller.damper_gain);
					break;

				case cmd_effect_gain_controller_set_friction_gain:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Effect_Gain_Controller.friction_gain);
					break;

				case cmd_effect_gain_controller_set_inertia_gain:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
									&Effect_Gain_Controller.inertia_gain);
					break;

				case cmd_effect_limiter_set_inertia_limiter:
					CMD_Set_float_from_buffer(USB_Out_Buffer, 2, 0.0f,
					FLT_MAX, &Effect_Limiter.inertia_limiter);
					break;

				case cmd_effect_set_spring_kp:
					CMD_Set_float_from_buffer(USB_Out_Buffer, 2, 0.0f,
					FLT_MAX, &Spring_Kp);
					Spring_PID_Init(&spring_pid, Spring_Kp, Spring_Ki,
									Spring_Kd);
					break;

				case cmd_effect_set_spring_ki:
					CMD_Set_float_from_buffer(USB_Out_Buffer, 2, 0.0f,
					FLT_MAX, &Spring_Ki);
					Spring_PID_Init(&spring_pid, Spring_Kp, Spring_Ki,
									Spring_Kd);
					break;

				case cmd_effect_set_spring_kd:
					CMD_Set_float_from_buffer(USB_Out_Buffer, 2, 0.0f,
					FLT_MAX, &Spring_Kd);
					Spring_PID_Init(&spring_pid, Spring_Kp, Spring_Ki,
									Spring_Kd);
					break;

				case cmd_steering_wheel_software_limiter_set_kp:
					CMD_Set_float_from_buffer(USB_Out_Buffer, 2, 0.0f,
					FLT_MAX, &SL_KP);
					SL_PID_Init(&sl_pid, SL_KP, SL_KI, SL_KD);
					break;

				case cmd_steering_wheel_software_limiter_set_ki:
					CMD_Set_float_from_buffer(USB_Out_Buffer, 2, 0.0f,
					FLT_MAX, &SL_KI);
					SL_PID_Init(&sl_pid, SL_KP, SL_KI, SL_KD);
					break;

				case cmd_steering_wheel_software_limiter_set_kd:
					CMD_Set_float_from_buffer(USB_Out_Buffer, 2, 0.0f,
					FLT_MAX, &SL_KD);
					SL_PID_Init(&sl_pid, SL_KP, SL_KI, SL_KD);
					break;

				case cmd_pwm_global_parameters_set_pwm_gain_multiple:
					CMD_Set_float_from_buffer(USB_Out_Buffer, 2, 0.0f,
					FLT_MAX, &PWM_Global_Parameters.pwm_gain_multiple);
					break;
				case cmd_steering_wheel_software_limiter_set_vibration_feedback_enable:
					SL_Vibration_Feedback.mode = Parameter;
					break;

				case cmd_steering_wheel_software_limiter_set_vibration_feedback_delay:
					CMD_Set_uint16((uint16_t*) &Parameter, 0U, UINT16_MAX,
															&SL_Vibration_Feedback.delay);
					break;
				default:
					// 其他命令
					break;
				}
				break;
			default:
				// Unknown report ID, handle error or ignore
				break;
			}	//End switch

			USB_Out_Flag = 0;
		}
//		osDelay(1);
	}
	/* USER CODE END StartUSBOutputTask */
}

/* USER CODE BEGIN Header_StartSynthesizerTask */
/**
 * @brief Function implementing the SynthesizerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSynthesizerTask */
void StartSynthesizerTask(void const *argument) {
	/* USER CODE BEGIN StartSynthesizerTask */
	uint8_t effect_running_flag = 0;
	angle_per_pulse = (float_t) (360.0
					/ ((float_t) Steering_Wheel.encoder.pulse * 4.0));
	SL_PID_Init(&sl_pid, SL_KP, SL_KI, SL_KD);
	Spring_PID_Init(&spring_pid, Spring_Kp, Spring_Ki, Spring_Kd); // 调整参数以适合你的需求
	/* Infinite loop */
	for (;;) {
		Steering_Wheel.current_angle =
						(float_t) (Steering_Wheel.encoder.pulse_count
										* angle_per_pulse);
		if (Actuator_Status == ACTUATOR_RUNNING) {		//当执行器运行时才能开始合成效果
			if ((Steering_Wheel.current_angle
							> (Steering_Wheel.rotation_range / 2))
							|| ((Steering_Wheel.current_angle
											< -(Steering_Wheel.rotation_range
															/ 2)))) {
				uint32_t current_time = HAL_GetTick(); // 获取开始时间
				float_t dt = (current_time - sl_pid_last_time) / 1000.0f;
				// 应用PID控制
				float_t pid_output = SL_PID_Compute(&sl_pid,
								Steering_Wheel.current_angle > 0 ?
												(Steering_Wheel.rotation_range
																/ 2) :
												-(Steering_Wheel.rotation_range
																/ 2),
								(float_t) Steering_Wheel.current_angle, dt);
				sl_pid_last_time = current_time;
				if (pid_output > MAX_FORCE) {
					pid_output =
					MAX_FORCE;
				} else if (pid_output < -MAX_FORCE) {
					pid_output = -MAX_FORCE;
				}
				Set_PWM_Value(pid_output);

				if (SL_Vibration_Feedback.mode
								== SL_Vibration_Feedback_CONSTANT) {
					osDelay(SL_Vibration_Feedback.delay);
					Set_PWM_Value(-pid_output);
				}
			} else {
				//sl_pid_last_time = 0;

				if (DC_Effect_Status != DC_EFFECT_PAUSED) {	//只要效果没有暂停就一直运行
					for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
						if (PB_pool[i].effect_state == EFFECT_RUNNING) {//必须有还在运行的效果才能合成
							effect_running_flag = 1;
							break;
						} else {
							effect_running_flag = 0;
						}
					}
					if (effect_running_flag) {

						Synthesize_Effects_To_PWM();
					} else {
						if (Is_PWM_Running()) {
							Clear_PWM();
						}
					}

				}
			}
		}
		osDelay(1);
	}
	/* USER CODE END StartSynthesizerTask */
}

/* USER CODE BEGIN Header_StartUSBInputTask */
/**
 * @brief Function implementing the USBInputTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUSBInputTask */
void StartUSBInputTask(void const *argument) {
	/* USER CODE BEGIN StartUSBInputTask */
	Steering_Wheel.encoder.max_pulses = (Steering_Wheel.rotation_range
					* Steering_Wheel.encoder.pulse) / 180;		// 180=360/2
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_values, 3);
	/* Infinite loop */
	for (;;) {
		switch (Current_Report_ID) {
		case INPUT_REPORT_ID:
			Input_Report.buttons = Read_Keys();

//		Input_Report.steering = (int16_t) ((Steering_Wheel.encoder.pulse_count
//						* STEERING_WHEEL_AXES_MAX_VALUE)
//						/ Steering_Wheel.encoder.max_pulses);

			if (Steering_Wheel.encoder.pulse_count
							> Steering_Wheel.encoder.max_pulses) {
				Steering_Wheel.encoder.clamped_pulse_count =
								Steering_Wheel.encoder.max_pulses;
			} else if (Steering_Wheel.encoder.pulse_count
							< -Steering_Wheel.encoder.max_pulses) {
				Steering_Wheel.encoder.clamped_pulse_count =
								-Steering_Wheel.encoder.max_pulses;
			} else {
				Steering_Wheel.encoder.clamped_pulse_count =
								Steering_Wheel.encoder.pulse_count;
			}

			Input_Report.steering =
							(int16_t) ((Steering_Wheel.encoder.clamped_pulse_count
											* STEERING_WHEEL_AXES_MAX_VALUE)
											/ Steering_Wheel.encoder.max_pulses);

			SendReport(Input_Report);
			break;
		case VMC_RESPONDING_REPORT_ID:
			for (uint8_t i = 0; i < 10; i++) {
				SendReport(Responding_Report);
			}
			// 发送完返回，继续发送InputReport
			Current_Report_ID = INPUT_REPORT_ID;
			break;

		}
//		osDelay(1);
	}
	/* USER CODE END StartUSBInputTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM3) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
