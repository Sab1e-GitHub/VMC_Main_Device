/*
 * pid_op.c
 *
 *  Created on: Oct 28, 2024
 *      Author: Sab1e
 */

#include "global.h"

// 定义启动效果的函数
uint8_t OP_Effect_Start(uint8_t index, uint8_t loop_count) {
	if (index == UNUSED_PB || index >= PB_POOL_ARRAY_SIZE ) {  // 修正了 index 范围检查
		return OP_FAIL; // 无效索引
	}
	if(PB_pool[index - 1].effect_parameter.effect_PB_index
					== UNUSED_EFFECT_TYPE){
		return OP_FAIL; // 无效索引
	}
	uint8_t PB_Pool_Index = index - 1;
	if (PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index != 0) {
		PB_pool[PB_Pool_Index].effect_state = EFFECT_RUNNING;
		PB_pool[PB_Pool_Index].loop_count = loop_count;

		Start_Timers(PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index);

		return OP_SUCCESS;
	}
	return OP_FAIL;

}

uint8_t OP_Effect_Start_Solo(uint8_t index, uint8_t loop_count) {
	if (index == UNUSED_PB || index >= PB_POOL_ARRAY_SIZE ) {  // 修正了 index 范围检查
			return OP_FAIL; // 无效索引
		}
		if(PB_pool[index - 1].effect_parameter.effect_PB_index
						== UNUSED_EFFECT_TYPE){
			return OP_FAIL; // 无效索引
		}
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].effect_parameter.effect_PB_index == index) { // 只运行所给索引的效果
			PB_pool[i].effect_state = EFFECT_RUNNING;
			PB_pool[i].loop_count = loop_count;
			Start_Timers(PB_pool[i].effect_parameter.effect_PB_index);
			//启动效果
		} else {	//其他的关闭
			PB_pool[i].effect_state = EFFECT_STOPPED;
			Delete_Timer(PB_pool[i].effect_parameter.effect_PB_index);
		}
	}
	return OP_SUCCESS;
}

uint8_t OP_Effect_Stop(uint8_t index) {
	uint8_t PB_Pool_Index = index - 1;
	if (PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index != 0) {
		Delete_Timer(PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index);
		PB_pool[PB_Pool_Index].effect_state = EFFECT_STOPPED;
		PB_pool[PB_Pool_Index].loop_count=0;
		return OP_SUCCESS;
	}
	return OP_FAIL;
}
