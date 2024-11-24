/*
 * pid_struct.h
 *
 *  Created on: Oct 24, 2024
 *      Author: Sab1e
 */

#ifndef PID_STRUCT_H_
#define PID_STRUCT_H_

#define SET_EFFECT_REPORT_ID                   0x01
#define ENVELOPE_REPORT_ID                     0x02
#define CONDITION_REPORT_ID                    0x03
#define PERIODIC_REPORT_ID                     0x04
#define CONSTANT_FORCE_REPORT_ID               0x05
#define RAMP_FROCE_REPORT_ID 							0x06
#define EFFECT_OPERATION_REPORT_ID             0x0A
#define PARAMETER_BLOCK_POOLS_REPORT_ID        0x43
#define PID_STATE_REPORT_ID                    0x02
#define PID_DEVICE_CONTROL_REPORT_ID           0x0B
#define CREATE_NEW_EFFECT_PARAMETER_BLOCK_ID   0x44
#define EFFECT_PARAMETER_BLOCK_FREE_REPORT_ID  0x45
#define EFFECT_PARAMETER_BLOCK_LOAD_REPORT_ID  0x33
#define DEVICE_GAIN_REPORT_ID                  0x40
#define INPUT_REPORT_ID                        0x03
#define VMC_REPORT_ID							0x64
#define VMC_RESPONDING_REPORT_ID				0x65

typedef struct __attribute__((packed)) {
	uint8_t id;		//ID=3
	int16_t steering;
	uint16_t accelerator;
	uint16_t brake;
	uint16_t clutch;
	uint32_t buttons;
} Input_Report_t;
typedef struct __attribute__((packed)) {
	uint8_t id;
	uint8_t byte_0;
	uint8_t byte_1;
	uint8_t byte_2;
	uint8_t byte_3;
	uint8_t byte_4;
} Responding_Report_t;
//
//typedef struct __attribute__((packed)) {
//	uint8_t id;		//ID=1
//	uint8_t effect_parameter_block_index;
//	uint8_t effect_type;
//	uint16_t duration;
//	uint8_t trigger_repeat_interval;
//	uint8_t sample_period;
//	uint16_t gain;
//	uint8_t trigger_button;
//	uint8_t axes_direction_enable;
//	uint8_t x_axis_direction;		//maybe
//	uint8_t y_axis_direction;		//maybe
//} Set_Effect_Report_t;
//
//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=2
//	uint8_t effect_parameter_block_index;
//	uint8_t attack_level;
//	uint8_t fade_level;
//	uint8_t attack_time;
//	uint8_t fade_time;
//} Envelope_Report_t;
//
//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=3
//	uint8_t effect_parameter_block_index;
//	int16_t center_point_offset;
//	int16_t positive_coefficient;
//	int16_t negative_coefficient;
//	uint16_t positive_saturation;
//	uint16_t negative_saturation;
//	uint16_t dead_band;
//} Condition_Report_t;
//
//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=4
//	uint8_t effect_parameter_block_index;
//	uint8_t phase;
//	uint16_t magnitude;
//	int16_t offset;
//	uint16_t period;		//1us
//} Periodic_Report_t;
//
//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=5
//	uint8_t effect_parameter_block_index;
//	int16_t magnitude;
//} Constant_Force_Report_t;
//
//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=10
//	uint8_t effect_parameter_block_index;
//	uint8_t effect_operation;
//	uint8_t loop_count;
//} Effect_Operation_Report_t;

//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=67
//	uint8_t ram_pool_size;
//	uint8_t effect_parameter_block_size;
//	uint8_t envelope_parameter_block_size;
//	uint8_t condition_parameter_block_size;
//	uint8_t periodic_parameter_block_size;
//	uint8_t constant_force_parameter_block_size;
//	uint8_t device_managed_pool;
//} Parameter_Block_Pools_Report_t;
//
//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=2
//	uint8_t effect_parameter_block_index;
//	uint8_t pid_state;
//} PID_State_Report_t;

//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=11
//	uint8_t pid_device_control;
//} PID_Device_Control_Report_t;

//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=68
//	uint8_t effect_type;
//	uint8_t byte_count;
//} Create_New_Effect_Parameter_Block_Report_t;

//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=69
//	uint8_t effect_parameter_block_index;
//} Effect_Parameter_Block_Free_Report_t;

//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=51
//	uint8_t effect_parameter_block_index;
//	uint8_t effect_parameter_block_load_status;
//	uint8_t ram_pool_available_size;
//} Effect_Parameter_Block_Load_Report_t;

//typedef struct __attribute__((packed)) {
//	uint8_t id;			//ID=64
//	uint16_t device_gain;
//} Device_Gain_Report_t;

extern Input_Report_t Input_Report;
extern Responding_Report_t Responding_Report;
//extern Set_Effect_Report_t Set_Effect_Report;
//extern Envelope_Report_t Envelope_Report;
//extern Condition_Report_t Condition_Report;
//extern Periodic_Report_t Periodic_Report;
//extern Constant_Force_Report_t Constant_Force_Report;
//extern Effect_Operation_Report_t Effect_Operation_Report;
//extern Parameter_Block_Pools_Report_t Parameter_Block_Pools_Report;
//extern PID_State_Report_t PID_State_Report;
//extern PID_Device_Control_Report_t PID_Device_Control_Report;
//extern Create_New_Effect_Parameter_Block_Report_t Create_New_Effect_Parameter_Block_Report;
//extern Effect_Parameter_Block_Free_Report_t Effect_Parameter_Block_Free_Report;
//extern Effect_Parameter_Block_Load_Report_t Effect_Parameter_Block_Load_Report;
//extern Device_Gain_Report_t Device_Gain_Report;

#endif /* PID_STRUCT_H_ */
