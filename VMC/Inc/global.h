/*
 * global.h
 *
 *  Created on: Oct 27, 2024
 *      Author: Sab1e
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdarg.h>
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "usbd_custom_hid_if.h"
#include "pid_struct.h"
#include "pid_dc.h"
#include "debug.h"
#include "usbd_customhid.h"
#include "pid_pool_mgr.h"
#include "pid_op.h"
#include <math.h>
#include "pid_synthesizer.h"
#include "rtos_timer_mgr.h"
#include "pid_wheel_mgr.h"
#include <float.h>
extern USBD_HandleTypeDef hUsbDeviceFS;

#define SendReport(report) \
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report))

#define DURATION_INFINITE 0x7531U
#define TRIGGER_REPEAT_INTERVAL_INFINITE 0xFFU
#define LOOP_INFINITE 0xFFU
#define DEFAULT_SAMPLE_RATE 0

#define STEERING_WHEEL_AXES_MAX_VALUE 32767		//������������ֵ��Ҳ����x������ֵ

#define GPIO_PWM1	GPIOC
#define GPIO_PWM2	GPIOC
#define GPIO_LED GPIOA
#define GPIO_DECODER_A GPIOC
#define GPIO_DECODER_B GPIOC

#define GPIO_PIN_PWM1	GPIO_PIN_2
#define GPIO_PIN_PWM2	GPIO_PIN_3
#define GPIO_PIN_LED GPIO_PIN_8
#define GPIO_PIN_DECODER_A GPIO_PIN_0
#define GPIO_PIN_DECODER_B GPIO_PIN_1

#define TIM4_PERIOD_SECONDS 0.15f //��λ ��

#define ADC_MAX_VALUE 4095  // ����ADC���ֵ

#define PID_DEVICE_MANAGE_MODE 1	// 0Ϊ���������Ч����أ�1Ϊ�豸���й���Ĳ������

#endif /* GLOBAL_H_ */
