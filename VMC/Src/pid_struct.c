/*
 * pid_struct.h
 *
 *  Created on: Oct 27, 2024
 *      Author: Sab1e
 */

#include "global.h"
/*==================Init Struct==================*/

// 结构体实例初始化
Input_Report_t Input_Report = { .id = INPUT_REPORT_ID, .steering = 0,
				.accelerator = 0, .brake = 0, .clutch = 0, .buttons = 0 };
Responding_Report_t Responding_Report =
				{ .id = VMC_RESPONDING_REPORT_ID, .byte_0 = 0, .byte_1 = 0,
								.byte_2 = 0, .byte_3 = 0, .byte_4 = 0, };
//
//Set_Effect_Report_t Set_Effect_Report = { .id = SET_EFFECT_REPORT_ID,
//				.effect_parameter_block_index = 0, .effect_type = 0, .duration =
//								0, .trigger_repeat_interval = 0,
//				.sample_period = 0, .gain = 0, .trigger_button = 0,
//				.axes_direction_enable = 0, .x_axis_direction = 0,
//				.y_axis_direction = 0 };
//
//Envelope_Report_t Envelope_Report = { .id = ENVELOPE_REPORT_ID,
//				.effect_parameter_block_index = 0, .attack_level = 0,
//				.fade_level = 0, .attack_time = 0, .fade_time = 0 };
//
//Condition_Report_t Condition_Report = { .id = CONDITION_REPORT_ID,
//				.effect_parameter_block_index = 0, .center_point_offset = 0,
//				.positive_coefficient = 0, .negative_coefficient = 0,
//				.positive_saturation = 0, .negative_saturation = 0, .dead_band =
//								0 };
//
//Periodic_Report_t Periodic_Report = { .id = PERIODIC_REPORT_ID,
//				.effect_parameter_block_index = 0, .phase = 0, .magnitude = 0,
//				.offset = 0, .period = 0 };
//
//Constant_Force_Report_t Constant_Force_Report = {
//				.id = CONSTANT_FORCE_REPORT_ID, .effect_parameter_block_index =
//								0, .magnitude = 0 };
//
//Effect_Operation_Report_t Effect_Operation_Report = { .id =
//				EFFECT_OPERATION_REPORT_ID, .effect_parameter_block_index = 0,
//				.effect_operation = 0, .loop_count = 0 };

//Parameter_Block_Pools_Report_t Parameter_Block_Pools_Report = { .id =
//				PARAMETER_BLOCK_POOLS_REPORT_ID, .ram_pool_size = 0,
//				.effect_parameter_block_size = 0,
//				.envelope_parameter_block_size = 0,
//				.condition_parameter_block_size = 0,
//				.periodic_parameter_block_size = 0,
//				.constant_force_parameter_block_size = 0, .device_managed_pool =
//								0 };
//
//PID_State_Report_t PID_State_Report = { .id = PID_STATE_REPORT_ID,
//				.effect_parameter_block_index = 0, .pid_state = 0 };

//PID_Device_Control_Report_t PID_Device_Control_Report = { .id =
//				PID_DEVICE_CONTROL_REPORT_ID, .pid_device_control = 0 };
//
//Create_New_Effect_Parameter_Block_Report_t Create_New_Effect_Parameter_Block_Report =
//				{ .id = CREATE_NEW_EFFECT_PARAMETER_BLOCK_ID, .effect_type = 0,
//								.byte_count = 0 };

//Effect_Parameter_Block_Free_Report_t Effect_Parameter_Block_Free_Report = {
//				.id = EFFECT_PARAMETER_BLOCK_FREE_REPORT_ID,
//				.effect_parameter_block_index = 0 };
//
//Effect_Parameter_Block_Load_Report_t Effect_Parameter_Block_Load_Report = {
//				.id = EFFECT_PARAMETER_BLOCK_LOAD_REPORT_ID,
//				.effect_parameter_block_index = 0,
//				.effect_parameter_block_load_status = 0,
//				.ram_pool_available_size = 0 };
//
//Device_Gain_Report_t Device_Gain_Report = { .id = DEVICE_GAIN_REPORT_ID,
//				.device_gain = 0 };
