/*
 * pid_pool_mgr.c
 *
 *  Created on: Oct 27, 2024
 *      Author: Sab1e
 */

/*
 * PB 	Parameter Block
 * PM Pool Manager
 */

#include "global.h"

static uint16_t used_bytes = 0; // 当前使用的字节数

Effect PB_pool[PB_POOL_ARRAY_SIZE];

PM_Create_New_Effect_PB_Results_t PM_Create_New_Effect_PB_Results = {
				.index = 0, .result = PM_FAIL };

uint16_t Get_Available_RAM_Size() {
	return sizeof(PB_pool) - used_bytes; // 计算剩余字节数
}

void Create_New_Effect_PB(Effect_TypeDef effect_type) {
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		// 检查是否未使用
		if (PB_pool[i].effect_parameter.effect_PB_index == UNUSED_PB) { // 0表示未使用
			// 初始化效果参数块
			PB_pool[i].effect_parameter.effect_PB_index = i + 1; // 从1开始
			PB_pool[i].effect_parameter.effect_type = effect_type;
			PB_pool[i].effect_state = EFFECT_STOPPED;
			PB_pool[i].time_step = 0;
			PB_pool[i].loop_count = 0;
			PB_pool[i].duration_timer_remaining_ticks = 0;
			PB_pool[i].sample_period_timer_remaining_ticks = 0;
			PB_pool[i].duration_timer = NULL;
			PB_pool[i].sample_period_timer = NULL;
			PB_pool[i].effect_parameter.duration = 0;
			PB_pool[i].effect_parameter.trigger_repeat_interval = 0;
			PB_pool[i].effect_parameter.sample_period = 0;
			PB_pool[i].effect_parameter.gain = 0;
			PB_pool[i].effect_parameter.trigger_button = 0;
			PB_pool[i].effect_parameter.axes_direction_enable = 0;
			PB_pool[i].effect_parameter.direction_instance1 = 0;
			PB_pool[i].effect_parameter.direction_instance2 = 0;

			// 使用 memset 将 parameters_array 清零
			memset(PB_pool[i].parameters_array, 0,
							sizeof(PB_pool[i].parameters_array));

			for (uint8_t j = 0; j < MAX_TYPE_SPECIFIC_BLOCKS; j++) {
				PB_pool[i].parameters_array[j].type =
								UNUSED_TYPE_SPECIFIC_EFFECT_TYPE; // 设置无效类型
			}
			used_bytes += PB_SIZE;
			// 返回创建成功的结果
			PM_Create_New_Effect_PB_Results.index =
							PB_pool[i].effect_parameter.effect_PB_index;
			PM_Create_New_Effect_PB_Results.result = BLOCK_LOAD_SUCCESS;
			return;
		}
	}

	// 如果没有可用的参数块
	PM_Create_New_Effect_PB_Results.index = 0;
	PM_Create_New_Effect_PB_Results.result = BLOCK_LOAD_FULL;
	return;
}

uint8_t Set_Effect_Parameter(uint8_t index, Effect_TypeDef effect_type,
				uint16_t duration,
				uint8_t trigger_repeat_interval,			//NOUSED
				uint8_t sample_period, uint16_t gain,
				uint8_t trigger_button,							//NOUSED
				uint8_t axes_direction_enable, uint8_t direction_instance1,
				uint8_t direction_instance2) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return PM_FAIL; // 无效索引
	}
	index--;
	PB_pool[index].effect_parameter.effect_type = effect_type; // 默认类型
	if(duration>DURATION_INFINITE){
		duration=DURATION_INFINITE;
	}
	PB_pool[index].effect_parameter.duration = duration;
	PB_pool[index].effect_parameter.trigger_repeat_interval =
					trigger_repeat_interval;
	PB_pool[index].effect_parameter.sample_period = sample_period;
	PB_pool[index].effect_parameter.gain = gain;
	PB_pool[index].effect_parameter.trigger_button = trigger_button;
	PB_pool[index].effect_parameter.axes_direction_enable =
					axes_direction_enable;
	PB_pool[index].effect_parameter.direction_instance1 =
					direction_instance1;
	PB_pool[index].effect_parameter.direction_instance2 =
					direction_instance2;

	return PM_SUCCESS;
}
uint8_t Add_Constant_Force_PB(uint8_t index, int16_t magnitude) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return PM_FAIL; // 无效索引
	}
	index--;
	if (PB_pool[index].effect_parameter.effect_PB_index
					== UNUSED_EFFECT_TYPE) {
		return PM_FAIL; // 无效索引
	}
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
					Constant_Force;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.constant_force_parameters.magnitude =
					magnitude;
	return PM_SUCCESS;
}
uint8_t Add_Ramp_Force_PB(uint8_t index, uint16_t ramp_start,uint16_t ramp_end) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return PM_FAIL; // 无效索引
	}
	index--;
	if (PB_pool[index].effect_parameter.effect_PB_index
					== UNUSED_EFFECT_TYPE) {
		return PM_FAIL; // 无效索引
	}
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
					Ramp_Force;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.ramp_force_parameters.ramp_start =
					ramp_start;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.ramp_force_parameters.ramp_end =
					ramp_end;
	return PM_SUCCESS;
}
uint8_t Add_Envelope_PB(uint8_t index, uint8_t attack_level, uint8_t fade_level,
				uint8_t attack_time, uint8_t fade_time) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return PM_FAIL; // 无效索引
	}
	index--;
	if (PB_pool[index].effect_parameter.effect_PB_index
					== UNUSED_EFFECT_TYPE) {
		return PM_FAIL; // 无效索引
	}
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].type =
					Envelope;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level =
					attack_level;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level =
					fade_level;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_time =
					attack_time;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_time =
					fade_time;
	return PM_SUCCESS;
}
uint8_t Add_Periodic_PB(uint8_t index, uint8_t phase, uint16_t magnitude,
				int16_t offset, uint16_t period) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return PM_FAIL; // 无效索引
	}
	index--;
	if (PB_pool[index].effect_parameter.effect_PB_index
					== UNUSED_EFFECT_TYPE) {
		return PM_FAIL; // 无效索引
	}
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
					Periodic;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.phase =
					phase;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude =
					magnitude;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.offset =
					offset;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period =
					period;
	return PM_SUCCESS;
}
uint8_t Add_Condition_PB(uint8_t index, int16_t center_point_offset,
				int16_t positive_coefficient, int16_t negative_coefficient,
				uint16_t positive_saturation, uint16_t negative_saturation,
				uint16_t dead_band) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return PM_FAIL; // 无效索引
	}
	index--;
	if (PB_pool[index].effect_parameter.effect_PB_index
					== UNUSED_EFFECT_TYPE) {
		return PM_FAIL; // 无效索引
	}
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
					Condition;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset =
					center_point_offset;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient =
					positive_coefficient;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient =
					negative_coefficient;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation =
					positive_saturation;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation =
					negative_saturation;
	PB_pool[index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band =
					dead_band;
	return PM_SUCCESS;
}
//uint8_t Add__PB(uint8_t index,) {
//	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
//		return PM_FAIL; // 无效索引
//	}
//
//	return PM_SUCCESS;
//}
//uint8_t Add_Type_Specific_PB(uint8_t index,
//				Type_Sepcific_Effect_TypeDef effect_type) {
//	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
//		return PM_FAIL; // 无效索引
//	}
//
//	uint8_t PB_Pool_Index = index - 1;
//	if (PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index != 0) {
//
//		switch (PB_pool[PB_Pool_Index].effect_parameter.effect_type) {
//		/*==================CONSTANT_FORCE==================*/
//		case CONSTANT_FORCE:
//			switch (effect_type) {
//			case Constant_Force:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
//								Constant_Force;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.constant_force_parameters.magnitude =
//								Constant_Force_Report.magnitude;
//				break;
//			case Envelope:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].type =
//								Envelope;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level =
//								Envelope_Report.attack_level;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level =
//								Envelope_Report.fade_level;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_time =
//								Envelope_Report.attack_time;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_time =
//								Envelope_Report.fade_time;
//				break;
//			default:
//				return PM_FAIL;
//				break;
//			}
//			/*==================SINE==================*/
//		case SINE:
//			switch (effect_type) {
//			case Periodic:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
//								Periodic;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.phase =
//								Periodic_Report.phase;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude =
//								Periodic_Report.magnitude;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.offset =
//								Periodic_Report.offset;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period =
//								Periodic_Report.period;
//				break;
//			case Envelope:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].type =
//								Envelope;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level =
//								Envelope_Report.attack_level;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level =
//								Envelope_Report.fade_level;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_time =
//								Envelope_Report.attack_time;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_time =
//								Envelope_Report.fade_time;
//				break;
//			default:
//				return PM_FAIL;
//				break;
//			}
//			/*==================SPRING==================*/
//		case SPRING:
//			switch (effect_type) {
//			case Condition:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
//								Condition;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset =
//								Condition_Report.center_point_offset;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient =
//								Condition_Report.positive_coefficient;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient =
//								Condition_Report.negative_coefficient;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation =
//								Condition_Report.positive_saturation;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation =
//								Condition_Report.negative_saturation;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band =
//								Condition_Report.dead_band;
//				break;
//			default:
//				return PM_FAIL;
//				break;
//			}
//			/*==================DAMPER==================*/
//		case DAMPER:
//			switch (effect_type) {
//			case Condition:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
//								Condition;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset =
//								Condition_Report.center_point_offset;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient =
//								Condition_Report.positive_coefficient;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient =
//								Condition_Report.negative_coefficient;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation =
//								Condition_Report.positive_saturation;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation =
//								Condition_Report.negative_saturation;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band =
//								Condition_Report.dead_band;
//				break;
//			default:
//				return PM_FAIL;
//				break;
//			}
//			/*==================FRICTION==================*/
//		case FRICTION:
//			switch (effect_type) {
//			case Condition:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
//								Condition;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset =
//								Condition_Report.center_point_offset;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient =
//								Condition_Report.positive_coefficient;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient =
//								Condition_Report.negative_coefficient;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation =
//								Condition_Report.positive_saturation;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation =
//								Condition_Report.negative_saturation;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band =
//								Condition_Report.dead_band;
//				break;
//			default:
//				return PM_FAIL;
//				break;
//			}
//			/*==================INERTIA==================*/
//		case INERTIA:
//			switch (effect_type) {
//			case Condition:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
//								Condition;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.center_point_offset =
//								Condition_Report.center_point_offset;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_coefficient =
//								Condition_Report.positive_coefficient;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_coefficient =
//								Condition_Report.negative_coefficient;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.positive_saturation =
//								Condition_Report.positive_saturation;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.negative_saturation =
//								Condition_Report.negative_saturation;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.condition_parameters.dead_band =
//								Condition_Report.dead_band;
//				break;
//			default:
//				return PM_FAIL;
//				break;
//			}
//			/*==================SQUARE==================*/
//		case SQUARE:
//			switch (effect_type) {
//			case Periodic:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
//								Periodic;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.phase =
//								Periodic_Report.phase;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude =
//								Periodic_Report.magnitude;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.offset =
//								Periodic_Report.offset;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period =
//								Periodic_Report.period;
//				break;
//			case Envelope:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].type =
//								Envelope;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level =
//								Envelope_Report.attack_level;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level =
//								Envelope_Report.fade_level;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_time =
//								Envelope_Report.attack_time;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_time =
//								Envelope_Report.fade_time;
//				break;
//			default:
//				return PM_FAIL;
//				break;
//			}
//			/*==================TRIANGLE==================*/
//		case TRIANGLE:
//			switch (effect_type) {
//			case Periodic:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].type =
//								Periodic;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.phase =
//								Periodic_Report.phase;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.magnitude =
//								Periodic_Report.magnitude;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.offset =
//								Periodic_Report.offset;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_1].Parameters.periodic_parameters.period =
//								Periodic_Report.period;
//				break;
//			case Envelope:
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].type =
//								Envelope;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_level =
//								Envelope_Report.attack_level;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_level =
//								Envelope_Report.fade_level;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.attack_time =
//								Envelope_Report.attack_time;
//				PB_pool[PB_Pool_Index].parameters_array[TYPE_SPECIFIC_PB_INDEX_2].Parameters.envelope_parameters.fade_time =
//								Envelope_Report.fade_time;
//				break;
//			default:
//				return PM_FAIL;
//				break;
//			}
//		default:
//			return PM_FAIL;
//			break;
//		}
//		/*==================SWITCH END==================*/
//		return PM_SUCCESS;
//	}
//	return PM_FAIL;
//}

/**
 * @brief 释放参数块
 * @param index 参数块索引
 * @return PM_FAIL 释放失败 PM_SUCCESS 释放成功
 */
uint8_t Free_Effect_PB(uint8_t index) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return PM_FAIL; // 无效索引
	}
	uint8_t PB_Pool_Index = index - 1;
	PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index = UNUSED_PB; // 标记为未使用
	PB_pool[PB_Pool_Index].effect_parameter.effect_type = UNUSED_EFFECT_TYPE;
	PB_pool[PB_Pool_Index].effect_state = EFFECT_STOPPED;
	PB_pool[PB_Pool_Index].time_step = 0;
	PB_pool[PB_Pool_Index].loop_count = 0;
	PB_pool[PB_Pool_Index].duration_timer_remaining_ticks = 0;
	PB_pool[PB_Pool_Index].sample_period_timer_remaining_ticks = 0;
	PB_pool[PB_Pool_Index].duration_timer = NULL;
	PB_pool[PB_Pool_Index].sample_period_timer = NULL;
	PB_pool[PB_Pool_Index].effect_parameter.duration = 0;
	PB_pool[PB_Pool_Index].effect_parameter.trigger_repeat_interval = 0;
	PB_pool[PB_Pool_Index].effect_parameter.sample_period = 0;
	PB_pool[PB_Pool_Index].effect_parameter.gain = 0;
	PB_pool[PB_Pool_Index].effect_parameter.trigger_button = 0;
	PB_pool[PB_Pool_Index].effect_parameter.axes_direction_enable = 0;
	PB_pool[PB_Pool_Index].effect_parameter.direction_instance1 = 0;
	PB_pool[PB_Pool_Index].effect_parameter.direction_instance2 = 0;

	// 使用 memset 将 parameters_array 清零
	memset(PB_pool[PB_Pool_Index].parameters_array, 0,
					sizeof(PB_pool[PB_Pool_Index].parameters_array));

	// 设置所有 lifetime 字段为 LIFETIME_GONE
	for (uint8_t j = 0; j < MAX_TYPE_SPECIFIC_BLOCKS; j++) {
		PB_pool[PB_Pool_Index].parameters_array[j].type =
						UNUSED_TYPE_SPECIFIC_EFFECT_TYPE; // 设置无效类型
	}
	used_bytes -= PB_SIZE; // 减少已使用的字节数
	//释放特定类型参数块的通用内存区域
	return PM_SUCCESS;

}

