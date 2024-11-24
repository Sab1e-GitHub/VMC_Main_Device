/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_custom_hid_if.c
 * @version        : v2.0_Cube
 * @brief          : USB Device Custom HID interface file.
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
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
#include "global.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t USB_Out_Flag;
uint8_t USB_Out_Buffer[USB_OUT_BUFFER_SIZE];

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @brief Usb device.
 * @{
 */

/** @addtogroup USBD_CUSTOM_HID
 * @{
 */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
 * @brief Private types.
 * @{
 */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
 * @}
 */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
 * @brief Private defines.
 * @{
 */

/* USER CODE BEGIN PRIVATE_DEFINES */
#define DESC_SIZE (uint16_t)sizeof(CUSTOM_HID_ReportDesc_FS)
/* USER CODE END PRIVATE_DEFINES */

/**
 * @}
 */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
 * @brief Private macros.
 * @{
 */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
 * @brief Private variables.
 * @{
 */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END
= {
/* USER CODE BEGIN 0 */
0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
				0x09, 0x04,        // Usage (Joystick)
				0xA1, 0x01,        // Collection (Application)
				/*==================JoyStick Input Begin==================*/
				/*==================Report ID=3 Begin==================*/
				/*==================Input X==================*/
				0x85, 0x03,        //   Report ID (3)
				0x05, 0x01, //   Usage Page (Generic Desktop Ctrls)
				0xA1, 0x00,        //   Collection (Physical)
				0x09, 0x30,        //     Usage (X)
				0x16, 0x00, 0x80, //     Logical Minimum (-32768)
				0x26, 0xFF, 0x7F, //     Logical Maximum (32767)
				0x75, 0x10,        //     Report Size (16)
				0x95, 0x01,     //     Report Count (1)		2B
				0x81, 0x02, //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
				/*==================Input Rx,Ry,Rz==================*/
				0x09, 0x33,        //     Usage (Rx)
				0x09, 0x34,        //     Usage (Ry)
				0x09, 0x35,        //     Usage (Rz)
				0x15, 0x00,        //     Logical Minimum (0)
				0x26, 0xFF, 0x0F,  //     Logical Maximum (4095)
				0x35, 0x00,        //     Physical Minimum (0)
				0x46, 0xFF, 0x0F, //     Physical Maximum (4095)
				0x75, 0x10,        //     Report Size (16)
				0x95, 0x03,     //     Report Count (3)		6B
				0x81, 0x02, //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
				0xC0,              //   End Physical Collection
				/*==================Input Button1~32==================*/
				0x05, 0x09,        //   Usage Page (Button)
				0x19, 0x01,        //   Usage Minimum (0x01)
				0x29, 0x20,        //   Usage Maximum (0x20)
				0x15, 0x00,        //   Logical Minimum (0)
				0x25, 0x01,        //   Logical Maximum (1)
				0x75, 0x01,        //   Report Size (1)
				0x95, 0x20,      //   Report Count (32)		4B
				0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
				/*==================Report ID=3 End==================*/
				/*==================PID Begin==================*/
				/*==================Report ID=1 Begin==================*/
				/**
				 * 下文就是主机发送数据给本设备，本设备需要接收的结构，必须把这个结构实现。
				 * 本结构的所有Report ID均为1。
				 */
				/*==================Effect Types==================*/
				0x05, 0x0F,        //   Usage Page (PID Page)
				0x09, 0x21, //   Usage (0x21)			Set Effect Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x01,        //     Report ID (1)
				0x09, 0x22, //     Usage (0x22)		Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				/*==================Effect Type==================*/
				0x09, 0x25, //     Usage (0x25)		Effect Type
				0xA1, 0x02,        //     Collection (Logical)
				0x09, 0x26, //       Usage (0x26)		ET Constant-Force
				0x09, 0x31,   //       Usage (0x31)		ET Sine
				0x09, 0x40, //       Usage (0x40)		ET Spring
				0x09, 0x41, //       Usage (0x41)		ET Damper
				0x09, 0x43, //       Usage (0x43)		ET Friction
				0x09, 0x42, //       Usage (0x42)		ET Inertia
				0x09, 0x30, //       Usage (0x30)		ET Square
				0x09, 0x32, //       Usage (0x32)		ET Triangle
				0x15, 0x01,        //       Logical Minimum (1)
				0x25, 0x08,        //       Logical Maximum (8)
				0x35, 0x01,        //       Physical Minimum (1)
				0x45, 0x08,        //       Physical Maximum (8)
				0x75, 0x08,        //       Report Size (8)
				0x95, 0x01,        //       Report Count (1)
				0x91, 0x00, //       Output (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0xC0,              //     End Collection
				//1B
				/*==================Duration==================*/
				0x09, 0x50,     //     Usage (0x50)		Duration
				0x15, 0x00,        //     Logical Minimum (0)
				0x26, 0x31, 0x75, //     Logical Maximum (30001)
				0x35, 0x00,        //     Physical Minimum (0)
				0x46, 0x98, 0x3A, //     Physical Maximum (15000)
				0x66, 0x03, 0x10, //     Unit (System: English Linear, Time: Seconds)
				0x55, 0xFD,       //     Unit (10^-3)Seconds=1ms
				0x75, 0x10,        //     Report Size (16)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				/*==================Sample Period==================*/
				0x09, 0x54, //     Usage (0x54)		Trigger Repeat Interval
				0x09, 0x51, //     Usage (0x51)		Sample Period
				0x26, 0xFF, 0x00,  //     Logical Maximum (255)
				0x46, 0xFF, 0x7F, //     Physical Maximum (32767)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x02,        //     Report Count (2)
				// From Duration    Unit (System: English Linear, Time: Seconds)
				//     							  Unit Exponent
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//Clear Global Unit
				0x55, 0x00,        //     Unit Exponent (0)
				0x66, 0x00, 0x00,  //     Unit (None)
				//2B
				/*==================Gain==================*/
				0x09, 0x52,        //     Usage (0x52)		Gain
				0x15, 0x00,        //     Logical Minimum (0)
				0x26, 0x10, 0x27, //     Logical Maximum (10000)
				0x35, 0x00,        //     Physical Minimum (0)
				0x46, 0x10, 0x27, //     Physical Maximum (10000)
				0x75, 0x10,        //     Report Size (16)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				/*==================Trigger Button==================*/
				0x09, 0x53, //     Usage (0x53)		Trigger Button
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x08,        //     Logical Maximum (8)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x08,        //     Physical Maximum (8)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				/*==================Axes Enable==================*/
				0x09, 0x55, //     Usage (0x55)		Axes Enable
				0xA1, 0x02,        //     Collection (Logical)
				0x05, 0x01, //       Usage Page (Generic Desktop Ctrls)			Usage Page Changed!!!
				0xA1, 0x00,       //       Collection (Physical)
				0x09, 0x30,        //         Usage (X)
				0x15, 0x00,       //         Logical Minimum (0)
				0x25, 0x01,       //         Logical Maximum (1)
				0x75, 0x01,        //         Report Size (1)
				0x95, 0x01,        //         Report Count (1)
				0x91, 0x02, //         Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0xC0,              //       End Collection
				0xC0,              //     End Collection
				/*==================Direction Enable==================*/
				0x05, 0x0F,        //     Usage Page (PID Page)
				0x09, 0x56, //     Usage (0x56)		Direction Enable
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0x95, 0x06,        //     Report Count (6)
				0x91, 0x03, //     Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				/*==================Direction==================*/
				//Usage Page (PID_Page)
				0x09, 0x57, //     Usage (0x57)		Direction
				0xA1, 0x02,        //     Collection (Logical)
				0x0B, 0x01, 0x00, 0x0A, 0x00, //       Usage (Ordinals: Instance 1)
				0x0B, 0x02, 0x00, 0x0A, 0x00, //       Usage (Ordinals: Instance 2)
				0x65, 0x14, //       Unit (System: English Rotation, Length: Centimeter)
				0x55, 0xFE,        //       Unit Exponent
				0x15, 0x00,        //       Logical Minimum (0)
				0x26, 0xFF, 0x00, //       Logical Maximum (255)
				0x35, 0x00,        //       Physical Minimum (0)
				0x47, 0xA0, 0x8C, 0x00, 0x00, //       Physical Maximum (35999)
				0x65, 0x00,        //       Unit (None)
				0x75, 0x08,        //       Report Size (8)
				0x95, 0x02,        //       Report Count (2)
				0x91, 0x02, //       Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0x55, 0x00,        //       Unit Exponent (0)
				0x65, 0x00,        //       Unit (None)
				0xC0,              //     End Collection
				0xC0,              //   End Collection
				//2B
				/*==================Report ID=1 End==================*/

				/*==================Report ID=2 Begin==================*/
				/*==================Set Envelope Report==================*/
				0x09, 0x5A, //   Usage (0x5A)		Set Envelope Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x02,        //     Report ID (2)
				0x09, 0x22, //     Usage (0x22)		Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0x09, 0x5B, //     Usage (0x5B)		Attack Leve
				0x09, 0x5D, //     Usage (0x5D)		Fade Level
				0x26, 0xFF, 0x00,  //     Logical Maximum (255)
				0x46, 0x10, 0x27, //     Physical Maximum (10000)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x02,        //     Report Count (2)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				0x09, 0x5C, //     Usage (0x5C)		Attack Time
				0x09, 0x5E, //     Usage (0x5E)		Fade Time
				0x66, 0x03, 0x10, //     Unit (System: English Linear, Time: Seconds)
				0x55, 0xFD,        //     Unit Exponent
				0x75, 0x08,        //     Report Size (8)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0x65, 0x00,        //     Unit (None)
				0x55, 0x00,        //     Unit Exponent (0)
				0xC0,              //   End Collection
				//2B
				/*==================Report ID=2 End==================*/

				/*==================Report ID=3 Begin==================*/
				/*==================Set Condition Report==================*/
				0x09, 0x5F, //   Usage (0x5F)			Set Condition Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x03,        //     Report ID (3)
				0x09, 0x22, //     Usage (0x22)		Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0x09, 0x60, //     Usage (0x60)		Center-Point Offset
				0x36, 0x01, 0x80, //     Physical Minimum (-32767)
				0x46, 0xFF, 0x7F, //     Physical Maximum (32767)
				0x16, 0x01, 0x80, //     Logical Minimum (-32767)
				0x26, 0xFF, 0x7F, //     Logical Maximum (32767)
				0x75, 0x10,        //     Report Size (16)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				0x09, 0x61, //     Usage (0x61)		Positive Coefficient
				0x09, 0x62, //     Usage (0x62)		Negative Coefficient
				0x36, 0xF0, 0xD8, //     Physical Minimum (-10000)
				0x46, 0x10, 0x27, //     Physical Maximum (10000)
				0x16, 0xF0, 0xD8, //     Logical Minimum (-10000)
				0x26, 0x10, 0x27, //     Logical Maximum (10000)
				0x95, 0x02,        //     Report Count (2)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//4B
				0x09, 0x63, //     Usage (0x63)		Positive Saturation
				0x09, 0x64, //     Usage (0x64)		Negative Saturation
				0x35, 0x00,        //     Physical Minimum (0)
				0x15, 0x00,        //     Logical Minimum (0)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//4B
				0x09, 0x65, //     Usage (0x65)		Dead Band
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0xC0,              //   End Collection
				//2B
				/*==================Report ID=3 End==================*/

				/*==================Report ID=4 Begin==================*/
				/*==================Set Periodic Report==================*/
				0x09, 0x6E, //   Usage (0x6E)			Set Periodic Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x04,        //     Report ID (4)
				0x09, 0x22, //     Usage (0x22)		Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0x09, 0x71,     //     Usage (0x71)		Phase
				0x26, 0xFF, 0x00,  //     Logical Maximum (255)
				0x46, 0x10, 0x27, //     Physical Maximum (10000)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0x09, 0x70, //     Usage (0x70)		Magnitude
				0x26, 0x10, 0x27, //     Logical Maximum (10000)
				0x75, 0x10,        //     Report Size (16)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				0x09, 0x6F,     //     Usage (0x6F)		Offset
				0x16, 0xF0, 0xD8, //     Logical Minimum (-10000)
				0x36, 0xF0, 0xD8, //     Physical Minimum (-10000)
				0x75, 0x10,        //     Report Size (16)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				0x09, 0x72,     //     Usage (0x72)		Period
				0x15, 0x00,        //     Logical Minimum (0)
				0x35, 0x00,        //     Physical Minimum (0)
				0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65534)
				0x47, 0xFF, 0xFF, 0x00, 0x00, //     Physical Maximum (65534)
				0x55, 0xFB,        //     Unit (10^-5)s = 1us
				0x66, 0x03, 0x10, //     Unit (System: English Linear, Time: Seconds)
				0x75, 0x10,        //     Report Size (16)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				0x65, 0x00,        //     Unit (None)
				0x55, 0x00,        //     Unit Exponent (0)
				0xC0,              //   End Collection
				/*==================Report ID=4 End==================*/

				/*==================Report ID=5 Begin==================*/
				/*==================Set Constant-Force Report==================*/
				0x09, 0x73, //   Usage (0x73)			Set Constant-Force Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x05,        //     Report ID (5)
				0x09, 0x22, //     Usage (0x22)		Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0x09, 0x70, //     Usage (0x70)		Magnitude
				0x16, 0xF0, 0xD8, //     Logical Minimum (-10000)
				0x26, 0x10, 0x27, //     Logical Maximum (10000)
				0x36, 0xF0, 0xD8, //     Physical Minimum (-10000)
				0x46, 0x10, 0x27, //     Physical Maximum (10000)
				0x75, 0x10,        //     Report Size (16)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				0xC0,//   End Collection
				/*==================Report ID=5 End==================*/

				/*==================Report ID=10 Begin==================*/
				/*==================Effect Operation Report==================*/
				0x09, 0x77, //   Usage (0x77)			Effect Operation Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x0A,        //     Report ID (10)
				0x09, 0x22, //     Usage (0x22)		Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0x09, 0x78, //     Usage (0x78)		Effect Operation
				0xA1, 0x02,        //     Collection (Logical)
				0x09, 0x79, //       Usage (0x79)		Op Effect Begin
				0x09, 0x7A, //       Usage (0x7A)	Op Effect Begin Solo
				0x09, 0x7B, //       Usage (0x7B)	Op Effect Stop
				0x15, 0x01,        //       Logical Minimum (1)
				0x25, 0x03,        //       Logical Maximum (3)
				0x91, 0x00, //       Output (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0xC0,              //     End Collection
				//1B
				0x09, 0x7C, //     Usage (0x7C)		Loop Count
				0x15, 0x00,        //     Logical Minimum (0)
				0x26, 0xFF, 0x00,  //     Logical Maximum (255)
				0x46, 0xFF, 0x00,  //     Physical Maximum (255)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0xC0,//   End Collection
				/*==================Report ID=10 End==================*/

				/*==================Report ID=67 Begin==================*/
				/*==================Parameter Block Pools Report==================*/
				0x09, 0x7F, //   Usage (0x7F)						//Parameter Block Pools Report, also named PID Pool Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x43,        //     Report ID (1)
				0x09, 0x80, //     Usage (0x80)		RAM Pool Size
				0x15, 0x00,        //     Logical Minimum (0)
				0x26, 0xFF, 0xFF, //     Logical Maximum (65535)
				0x35, 0x00,        //     Physical Minimum (0)
				0x46, 0xFF, 0xFF, //     Physical Maximum (65535)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0xB1, 0x02, //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0x09, 0xA8, //     Usage (0xA8)			Parameter Block Size
				0xA1, 0x02,        //     Collection (Logical)
				0x09, 0x21, //       Usage (0x21)			Set Effect Report
				0x09, 0x5A, //       Usage (0x5A)		Set Envelope Report
				0x09, 0x5F, //       Usage (0x5F)			Set Condition Report
				0x09, 0x6E, //       Usage (0x6E)			Set Periodic Report
				0x09, 0x73, //       Usage (0x73)			Set Constant-Force Report
				0x75, 0x08,        //       Report Size (8)
				0x95, 0x05,        //       Report Count (5)
				0xB1, 0x02, //       Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//5B
				0xC0,//     End Collection
				/*==================占位符==================*/
				0x25, 0x01,        //     Logical Maximum (1)
				0x75, 0x07,        //     Report Size (7)
				0x95, 0x01,        //     Report Count (1)
				0xB1, 0x03, //     Feature (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				/*==================占位符==================*/
				//高位[0xA9]000 0000低位
				0x09, 0xA9, //     Usage (0xA9)		Device-Managed Pool
				0x75, 0x01,        //     Report Size (1)
				0xB1, 0x02, //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0xC0,//   End Collection
				/*==================Report ID=67 End==================*/

				/*==================Report ID=2 Begin==================*/
				/*==================PID State Report==================*/
				0x09, 0x92,        //   Usage (0x92)
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x02,        //     Report ID (2)
				0x09, 0x22, //     Usage (0x22)		Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x81, 0x02, //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
				//1B
				0x09, 0x94, //     Usage (0x94)		Effect Playing
				0x09, 0xA0, //     Usage (0xA0)		Actuators Enabled
				0x09, 0xA4, //     Usage (0xA4)		Safety Switch
				0x09, 0xA6, //     Usage (0xA6)		Actuator Power
				0x25, 0x01,        //     Logical Maximum (1)
				0x45, 0x01,        //     Physical Maximum (1)
				0x75, 0x01,        //     Report Size (1)
				0x95, 0x04,        //     Report Count (4)
				0x81, 0x02, //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
				0x81, 0x03, //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
				//1B
				0xC0,//   End Collection
				/*==================Report ID=2 End==================*/

				/*==================Report ID=11 Begin==================*/
				/*==================PID Device Control==================*/
				0x09, 0x96,        //   Usage (0x96)
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x0B,        //     Report ID (11)
				0x09, 0x97, //     Usage (0x97)		DC Enable Actuator
				0x09, 0x98, //     Usage (0x98)		DC Disable Actuator
				0x09, 0x99, //     Usage (0x99)		DC Stop All Effects
				0x09, 0x9A,     //     Usage (0x9A)		DC Reset
				0x09, 0x9B,     //     Usage (0x9B)		DC Pause
				0x09, 0x9C, //     Usage (0x9C)		DC Continue
				0x15, 0x00,        //     Logical Minimum (0)
				0x25, 0x01,        //     Logical Maximum (1)
				0x75, 0x01,        //     Report Size (1)
				0x95, 0x06,        //     Report Count (6)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0x95, 0x02,        //     Report Count (2)
				0x91, 0x03, //     Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0xC0,//   End Collection
				/*==================Report ID=11 End==================*/

				/*==================Report ID=68 Begin==================*/
				/*==================Create New Effect Parameter Block Report==================*/
				0x09, 0xAB, //   Usage (0xAB)		Create New Effect Parameter Block Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x44,        //     Report ID (68)
				0x09, 0x25, //     Usage (0x25)		Effect Type
				0xA1, 0x02,        //     Collection (Logical)
				0x09, 0x26, //       Usage (0x26)		ET Constant-Force
				0x09, 0x31,   //       Usage (0x31)		ET Sine
				0x09, 0x40, //       Usage (0x40)		ET Spring
				0x09, 0x41, //       Usage (0x41)		ET Damper
				0x09, 0x43, //       Usage (0x43)		ET Friction
				0x09, 0x42, //       Usage (0x42)		ET Inertia
				0x09, 0x30, //       Usage (0x30)		ET Square
				0x09, 0x32, //       Usage (0x32)		ET Triangle
				0x15, 0x01,        //       Logical Minimum (1)
				0x25, 0x08,        //       Logical Maximum (8)
				0x35, 0x01,        //       Physical Minimum (1)
				0x45, 0x08,        //       Physical Maximum (8)
				0x75, 0x08,        //       Report Size (8)
				0x95, 0x01,        //       Report Count (1)
				0xB1, 0x00, //       Feature (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0xC0,//     End Collection

				0x05, 0x01, //     Usage Page (Generic Desktop Ctrls)
				0x09, 0x3B,        //     Usage (Byte Count)
				0x15, 0x00,        //     Logical Minimum (0)
				0x26, 0xFF, 0x00,  //     Logical Maximum (255)
				0x35, 0x00,        //     Physical Minimum (0)
				0x46, 0xFF, 0x00,  //     Physical Maximum (255)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0xB1, 0x02, //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0xC0,//   End Collection
				/*==================Report ID=68 End==================*/

				/*==================Report ID=69 Begin==================*/
				/*==================Effect Parameter Block Free Report==================*/
				0x05, 0x0F,        //   Usage Page (PID Page)
				0x09, 0x90, //   Usage (0x90)				Effect Parameter Block Free Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x45,        //     Report ID (69)
				0x09, 0x22, //     Usage (0x22)			Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0xC0,//   End Collection
				/*==================Report ID=69 End==================*/

				/*==================Report ID=51 Begin==================*/
				/*==================Effect Parameter Block Load Report==================*/
				0x09, 0x89, //   Usage (0x89)		Effect Parameter Block Load Report
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x33,        //     Report ID (51)
				0x09, 0x22, //     Usage (0x22)		Effect Parameter Block Index
				0x15, 0x01,        //     Logical Minimum (1)
				0x25, 0x09,        //     Logical Maximum (9)
				0x35, 0x01,        //     Physical Minimum (1)
				0x45, 0x09,        //     Physical Maximum (9)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0xB1, 0x02, //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0x09, 0x8B, //     Usage (0x8B)		Effect Parameter Block Load Status
				0xA1, 0x02,        //     Collection (Logical)
				0x09, 0x8C, //       Usage (0x8C)		Block Load Success
				0x09, 0x8D, //       Usage (0x8D)	Block Load Full
				0x09, 0x8E, //       Usage (0x8E)		Block Load Error
				0x15, 0x01,        //       Logical Minimum (1)
				0x25, 0x03,        //       Logical Maximum (3)
				0x35, 0x01,        //       Physical Minimum (1)
				0x45, 0x03,        //       Physical Maximum (3)
				0x75, 0x08,        //       Report Size (8)
				0x95, 0x01,        //       Report Count (1)
				0xB1, 0x00, //       Feature (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0xC0,//     End Collection

				0x09, 0xAC, //     Usage (0xAC)		RAM Pool Available
				0x15, 0x00,        //     Logical Minimum (0)
				0x26, 0xFF, 0xFF, //     Logical Maximum (65535)
				0x35, 0x00,        //     Physical Minimum (0)
				0x46, 0xFF, 0xFF, //     Physical Maximum (65535)
				0x75, 0x08,        //     Report Size (8)
				0x95, 0x01,        //     Report Count (1)
				0xB1, 0x02, //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//1B
				0xC0,//   End Collection
				/*==================Report ID=51 End==================*/

				/*==================Report ID=64 Begin==================*/
				/*==================Device Gain Report==================*/
				0x09, 0x7D,        //   Usage (0x7D)
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x40,        //     Report ID (64)
				0x09, 0x7E, //     Usage (0x7E)		Device Gain
				0x26, 0x10, 0x27, //     Logical Maximum (10000)
				0x46, 0x10, 0x27, //     Physical Maximum (10000)
				0x75, 0x10,        //     Report Size (16)
				0x95, 0x01,        //     Report Count (1)
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				//2B
				0xC0,//   End Collection
				/*==================Report ID=64 End==================*/

				/*==================PID End==================*/

				0x06, 0x00, 0xFF, // Usage Page (Vendor Defined 0xFF00)
				0x09, 0x01,       //   Usage (Vendor Usage 0x01)
				0xA1, 0x02,        //   Collection (Logical)
				0x85, 0x64,        //     Report ID (100)
				0x09, 0x01,            // Usage (0x01)
				0x15, 0x00,            // Logical Minimum (0)
				0x26, 0xFF, 0x00,      // Logical Maximum (255)
				0x75, 0x08,            // Report Size (8 bits)
				0x95, 0x05, // Report Count (5 bytes)  Bit0 = Command Bit1~4 = Parameters
				0x91, 0x02, //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

				0x85, 0x65,        //     Report ID (101)
				0x09, 0x01,        //     Usage (0x01)
				0x95, 0x05, // Report Count (5 bytes)  Bit0 = Command Bit1~4 = Parameters
				0xB1, 0x02, //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
				0xC0,              //   End Collection

				/* USER CODE END 0 */
				0xC0 /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
 * @brief Public variables.
 * @{
 */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
static int8_t CustomHID_SetFeature(uint8_t event_idx, uint8_t *buffer);
static int8_t CustomHID_GetFeature(uint8_t event_idx, uint8_t *buffer,
				uint16_t *length);
/* USER CODE END EXPORTED_VARIABLES */
/**
 * @}
 */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
 * @}
 */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS = { CUSTOM_HID_ReportDesc_FS,
				CUSTOM_HID_Init_FS, CUSTOM_HID_DeInit_FS,
				CUSTOM_HID_OutEvent_FS, CustomHID_SetFeature,
				CustomHID_GetFeature };

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
 * @brief Private functions.
 * @{
 */

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initializes the CUSTOM HID media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CUSTOM_HID_Init_FS(void) {
	/* USER CODE BEGIN 4 */
	return (USBD_OK);
	/* USER CODE END 4 */
}

/**
 * @brief  DeInitializes the CUSTOM HID media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CUSTOM_HID_DeInit_FS(void) {
	/* USER CODE BEGIN 5 */
	return (USBD_OK);
	/* USER CODE END 5 */
}

/**
 * @brief  Manage the CUSTOM HID class events
 * @param  event_idx: Event index
 * @param  state: Event state
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state) {
	/* USER CODE BEGIN 6 */
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
	        USBD_CUSTOM_HID_HandleTypeDef *hhid =
	                    (USBD_CUSTOM_HID_HandleTypeDef*) hUsbDeviceFS.pClassData;
	        memcpy(USB_Out_Buffer, hhid->Report_buf, USB_OUT_BUFFER_SIZE);
	        memset(hhid->Report_buf, 0, sizeof(hhid->Report_buf));
	        USB_Out_Flag = 1;

	        // 释放互斥锁
	        xSemaphoreGive(xMutex);
	    }
	return (USBD_OK);
	/* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */

uint8_t vmc_last_resp = resp_unknown;

static int8_t CustomHID_SetFeature(uint8_t event_idx, uint8_t *buffer) {
	uint8_t Feature_Buffer[USB_OUT_BUFFER_SIZE];
	USBD_CUSTOM_HID_HandleTypeDef *hhid =
					(USBD_CUSTOM_HID_HandleTypeDef*) hUsbDeviceFS.pClassData;
	memcpy(Feature_Buffer, hhid->Report_buf, USB_OUT_BUFFER_SIZE);

	switch (event_idx) {

	case CREATE_NEW_EFFECT_PARAMETER_BLOCK_ID:

		if (Feature_Buffer[1] == 0x01) {
			Create_New_Effect_PB(CONSTANT_FORCE);
		} else if (Feature_Buffer[1] == 0x02) {
			Create_New_Effect_PB(SINE);
		} else if (Feature_Buffer[1] == 0x03) {
			Create_New_Effect_PB(SPRING);
		} else if (Feature_Buffer[1] == 0x04) {
			Create_New_Effect_PB(DAMPER);
		} else if (Feature_Buffer[1] == 0x05) {
			Create_New_Effect_PB(FRICTION);
		} else if (Feature_Buffer[1] == 0x06) {
			Create_New_Effect_PB(INERTIA);
		} else if (Feature_Buffer[1] == 0x07) {
			Create_New_Effect_PB(SQUARE);
		} else if (Feature_Buffer[1] == 0x08) {
			Create_New_Effect_PB(TRIANGLE);
		}

		break;
	case VMC_RESPONDING_REPORT_ID:
		switch (Feature_Buffer[1]) {
		case resp_accelerator_pedal_maximum:
			Pedal_Limiter.accelerator_pedal_maximum = adc_values[0];
			vmc_last_resp = resp_accelerator_pedal_maximum;
			break;
		case resp_accelerator_pedal_minimum:
			Pedal_Limiter.accelerator_pedal_minimum = adc_values[0];
			vmc_last_resp = resp_accelerator_pedal_minimum;
			break;
		case resp_brake_pedal_maximum:
			Pedal_Limiter.brake_pedal_set_maximum = adc_values[1];
			vmc_last_resp = resp_brake_pedal_maximum;
			break;
		case resp_brake_pedal_minimum:
			Pedal_Limiter.brake_pedal_set_minimum = adc_values[1];
			vmc_last_resp = resp_brake_pedal_minimum;
			break;
		case resp_clutch_pedal_maximum:
			Pedal_Limiter.clutch_pedal_set_maximum = adc_values[2];
			vmc_last_resp = resp_clutch_pedal_maximum;
			break;
		case resp_clutch_pedal_minimum:
			Pedal_Limiter.clutch_pedal_set_minimum = adc_values[2];
			vmc_last_resp = resp_clutch_pedal_minimum;
			break;
		}

		break;
	default: /* Report does not exist */
		break;
	}

	return (USBD_OK);
}

static int8_t CustomHID_GetFeature(uint8_t event_idx, uint8_t *buffer,
				uint16_t *length) {

	uint8_t *data;
	uint16_t len;
	//想传输的数据先放在data里面，第一个元素是ReportID
	data = &buffer[1];

	switch (event_idx) {
	case VMC_RESPONDING_REPORT_ID:
		switch (vmc_last_resp) {
		case resp_accelerator_pedal_maximum:
			data[0] = resp_accelerator_pedal_maximum;
			data[1] = Pedal_Limiter.accelerator_pedal_maximum
							& 0xFF;
			data[2] = Pedal_Limiter.accelerator_pedal_maximum
							>> 8;

			vmc_last_resp = resp_unknown;
			break;
		case resp_accelerator_pedal_minimum:
			data[0] = resp_accelerator_pedal_minimum;
			data[1] = Pedal_Limiter.accelerator_pedal_minimum
							& 0xFF;
			data[2] = Pedal_Limiter.accelerator_pedal_minimum
							>> 8;
			vmc_last_resp = resp_unknown;
			break;
		case resp_brake_pedal_maximum:
			data[0] = resp_brake_pedal_maximum;
			data[1] = Pedal_Limiter.brake_pedal_set_maximum
							& 0xFF;
			data[2] = Pedal_Limiter.brake_pedal_set_maximum
							>> 8;
			vmc_last_resp = resp_unknown;
			break;
		case resp_brake_pedal_minimum:
			data[0] = resp_brake_pedal_minimum;
			data[1] = Pedal_Limiter.brake_pedal_set_minimum
							& 0xFF;
			data[2] = Pedal_Limiter.brake_pedal_set_minimum
							>> 8;
			vmc_last_resp = resp_unknown;
			break;
		case resp_clutch_pedal_maximum:
			data[0] = resp_clutch_pedal_maximum;
			data[1] = Pedal_Limiter.clutch_pedal_set_maximum
							& 0xFF;
			data[2] = Pedal_Limiter.clutch_pedal_set_maximum
							>> 8;
			vmc_last_resp = resp_unknown;
			break;
		case resp_clutch_pedal_minimum:
			data[0] = resp_clutch_pedal_minimum;
			data[1] = Pedal_Limiter.clutch_pedal_set_minimum
							& 0xFF;
			data[2] = Pedal_Limiter.clutch_pedal_set_minimum
							>> 8;
			vmc_last_resp = resp_unknown;
			break;

		}
		len = 3;
		break;
	case EFFECT_PARAMETER_BLOCK_LOAD_REPORT_ID:
		//配置Effect Parameter Block Load Report
		data[0] = PM_Create_New_Effect_PB_Results.index;
		data[1] = PM_Create_New_Effect_PB_Results.result;
		uint16_t ram_size = Get_Available_RAM_Size();
		data[2] = ram_size & 0xFF;		//发送低位
		data[3] = ram_size >> 8;			//发送高位
		len = 4;
		//Clear Results
		PM_Create_New_Effect_PB_Results.index = 0;
		PM_Create_New_Effect_PB_Results.result = BLOCK_LOAD_ERROR;
		break;

	default: /* Report does not exist */
		return (USBD_FAIL);
		break;
	}

	buffer[0] = event_idx;  // ReportID must reside inside the first byte
	*length = len + 1;      // +1 -> ReportID must also be considered

	return (USBD_OK);
}

/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

