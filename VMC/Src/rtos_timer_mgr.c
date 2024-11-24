/*
 * rtos_timer_mgr.c
 *
 *  Created on: Oct 29, 2024
 *      Author: Sab1e
 */

#include "global.h"
/**
 * 所有删除定时器的操作都会清空Loop Count
 */



//效果定时器的回调函数 Duration Timer
void Effect_Timer_Callback(TimerHandle_t xTimer) {
	uint32_t index = (uint32_t) pvTimerGetTimerID(xTimer);	//获取参数块索引值
	// 当Effect执行完毕后就停止
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return; // 无效索引
	}
	uint8_t PB_Pool_Index = index - 1;
	//执行完毕看是否需要继续 Loop Count>0
	//如果>0就重新启动计时器，开启新一轮的运行
	if (PB_pool[PB_Pool_Index].loop_count == LOOP_INFINITE) {	//无限循环
		PB_pool[PB_Pool_Index].effect_state = EFFECT_RUNNING;
		xTimerStart(PB_pool[PB_Pool_Index].duration_timer, 0);
	} else if (PB_pool[PB_Pool_Index].loop_count > 0) {
		PB_pool[PB_Pool_Index].effect_state = EFFECT_RUNNING;
		xTimerStart(PB_pool[PB_Pool_Index].duration_timer, 0);
		PB_pool[PB_Pool_Index].loop_count--;
	} else {
		PB_pool[PB_Pool_Index].effect_state = EFFECT_STOPPED;
		Delete_Duration_Timer(PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index);
		//所有删除定时器的函数都会清除loop_count，因此不必在此处清除loop_count
	}

	return;
}
// 间隔指定时间后，执行一次效果
void Effect_Sample_Period_Callback(TimerHandle_t xTimer) {
	uint32_t index = (uint32_t) pvTimerGetTimerID(xTimer) - PB_POOL_ARRAY_SIZE;	//获取参数块索引值 去掉PB_POOL_ARRAY_SIZE大小的偏移才是索引值
	// 当Effect执行完毕后就停止
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return; // 无效索引
	}
	uint8_t PB_Pool_Index = index - 1;
	PB_pool[PB_Pool_Index].effect_state = EFFECT_RUNNING; //开始运行，并且在运行完一次后自动恢复EFFECT_WAITING
	return;
}

uint8_t Start_Timers(uint8_t index){
	uint8_t PB_Pool_Index = index - 1;
	// 启动 Duration 定时器
		if (PB_pool[PB_Pool_Index].effect_parameter.duration != DURATION_INFINITE) {
			if (PB_pool[PB_Pool_Index].duration_timer != NULL) {
				xTimerDelete(PB_pool[PB_Pool_Index].duration_timer, 0); // 删除旧的定时器句柄，避免内存泄漏
			}

			PB_pool[PB_Pool_Index].duration_timer =
							xTimerCreate("EffectTimer",
											pdMS_TO_TICKS(
															PB_pool[PB_Pool_Index].effect_parameter.duration),
											pdFALSE,
											(void*) (uintptr_t) PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index, //直接使用index作为索引
											Effect_Timer_Callback);

			if (PB_pool[PB_Pool_Index].duration_timer != NULL) {
				xTimerStart(PB_pool[PB_Pool_Index].duration_timer, 0);
			} else {
				return TM_FAIL;  // 定时器创建失败
			}
		}

		/**
		 * 如果 Sample Period 不为默认的采样率
		 * 就单独给它启动 Sample Period 定时器
		 * 然后根据这个定时器对应的采样率来更新它的Effect
		 */
		if (PB_pool[PB_Pool_Index].effect_parameter.sample_period
						!= DEFAULT_SAMPLE_RATE) {
			if (PB_pool[PB_Pool_Index].sample_period_timer != NULL) {
				xTimerDelete(PB_pool[PB_Pool_Index].sample_period_timer, 0); // 删除旧的定时器句柄，避免内存泄漏
			}

			PB_pool[PB_Pool_Index].sample_period_timer =
							xTimerCreate("EffectSamplePeriodTimer",
											pdMS_TO_TICKS(
															PB_pool[PB_Pool_Index].effect_parameter.sample_period),
											pdTRUE,  // 自动重载
											(void*) (uintptr_t) PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index
															+ PB_POOL_ARRAY_SIZE, //由于ID不能重复，添加PB_POOL_ARRAY_SIZE大小的偏移作为ID
											Effect_Sample_Period_Callback);

			if (PB_pool[PB_Pool_Index].sample_period_timer != NULL) {
				xTimerStart(PB_pool[PB_Pool_Index].sample_period_timer, 0);
			} else {
				return TM_FAIL;  // 定时器创建失败
			}
		}
		return TM_OK;
}


uint8_t Pause_All_Timers(void) {
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].sample_period_timer != NULL) {
			PB_pool[i].sample_period_timer_remaining_ticks =
							xTimerGetExpiryTime(PB_pool[i].sample_period_timer)
											- xTaskGetTickCount();
			xTimerStop(PB_pool[i].sample_period_timer, 0);  // 停止定时器
		}
	}
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].duration_timer != NULL) {
			PB_pool[i].duration_timer_remaining_ticks = xTimerGetExpiryTime(
							PB_pool[i].duration_timer) - xTaskGetTickCount();
			xTimerStop(PB_pool[i].duration_timer, 0);  // 停止定时器
		}
	}
	return TM_OK;
}
//// 暂停Duration定时器
//void Pause_Duration_Timer(uint8_t index) {
//	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
//		return; // 无效索引
//	}
//	uint8_t PB_Pool_Index = index - 1;
//	PB_pool[PB_Pool_Index].duration_timer_remaining_ticks = xTimerGetExpiryTime(
//					PB_pool[PB_Pool_Index].duration_timer) - xTaskGetTickCount();
//	xTimerStop(PB_pool[PB_Pool_Index].duration_timer, 0);  // 停止定时器
//	return;
//
//}
//
//void Pause_Sample_Period_Timer(uint8_t index) {
//	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
//		return; // 无效索引
//	}
//	uint8_t PB_Pool_Index = index - 1;
//	PB_pool[PB_Pool_Index].sample_period_timer_remaining_ticks =
//					xTimerGetExpiryTime(xTimer) - xTaskGetTickCount();
//	xTimerStop(xTimer, 0);  // 停止定时器
//	return;
//
//}

// 恢复定时器
uint8_t Resume_All_Timers(void) {
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].duration_timer_remaining_ticks != 0) {
			PB_pool[i].duration_timer_remaining_ticks = 0;
			xTimerChangePeriod(PB_pool[i].duration_timer,
							PB_pool[i].duration_timer_remaining_ticks, 0); // 恢复定时器的周期
			xTimerStart(PB_pool[i].duration_timer, 0);  // 启动定时器
		}
	}
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].sample_period_timer_remaining_ticks != 0) {
			PB_pool[i].sample_period_timer_remaining_ticks = 0;
			xTimerChangePeriod(PB_pool[i].sample_period_timer,
							PB_pool[i].sample_period_timer_remaining_ticks, 0); // 恢复定时器的周期
			xTimerStart(PB_pool[i].sample_period_timer, 0);  // 启动定时器
		}
	}
	return TM_OK;
}
/**
 * @brief 删除所有PB_Pool内的所有定时器
 * @param 无参数
 * @return TM_OK指删除成功 TM_FAIL指删除失败
 */
uint8_t Delete_All_Timers(void) {
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].duration_timer != NULL) {
			PB_pool[i].duration_timer_remaining_ticks = 0;
			for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
				if (xTimerDelete(PB_pool[i].duration_timer,
								portMAX_DELAY) == pdPASS) {
					PB_pool[i].duration_timer = NULL; // 删除成功后将句柄重置为 NULL
					PB_pool[i].loop_count = 0;
					break; // 删除成功，退出循环
				}
				// 如果在重试3次后仍然失败，则返回 TM_FAIL
				if (retry_count == 2) {
					return TM_FAIL;
				}
			}
		}
	}

	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].sample_period_timer != NULL) {
			PB_pool[i].sample_period_timer_remaining_ticks = 0;
			for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
				if (xTimerDelete(PB_pool[i].sample_period_timer,
								portMAX_DELAY) == pdPASS) {
					PB_pool[i].sample_period_timer = NULL; // 删除成功后将句柄重置为 NULL
					PB_pool[i].loop_count = 0;
					break; // 删除成功，退出循环
				}
				// 如果在重试3次后仍然失败，则返回 TM_FAIL
				if (retry_count == 2) {
					return TM_FAIL;
				}
			}
		}
	}

	return TM_OK;
}
uint8_t Delete_Timer(uint8_t index) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return TM_FAIL; // 无效索引
	}
	uint8_t PB_Pool_Index = index - 1;
	if (PB_pool[PB_Pool_Index].duration_timer != NULL) {
		PB_pool[PB_Pool_Index].duration_timer_remaining_ticks = 0;
		for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
			if (xTimerDelete(PB_pool[PB_Pool_Index].duration_timer,
							portMAX_DELAY) == pdPASS) {
				PB_pool[PB_Pool_Index].duration_timer = NULL; // 删除成功后将句柄重置为 NULL
				break; // 删除成功，退出循环
			}
			// 如果在重试3次后仍然失败，则返回 TM_FAIL
			if (retry_count == 2) {
				return TM_FAIL;
			}
		}
	}
	if (PB_pool[PB_Pool_Index].sample_period_timer != NULL) {
		PB_pool[PB_Pool_Index].sample_period_timer_remaining_ticks = 0;
		for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
			if (xTimerDelete(PB_pool[PB_Pool_Index].sample_period_timer,
							portMAX_DELAY) == pdPASS) {
				PB_pool[PB_Pool_Index].sample_period_timer = NULL; // 删除成功后将句柄重置为 NULL
				break; // 删除成功，退出循环
			}
			// 如果在重试3次后仍然失败，则返回 TM_FAIL
			if (retry_count == 2) {
				return TM_FAIL;
			}
		}
	}
	PB_pool[PB_Pool_Index].loop_count = 0;
	return TM_OK;
}


uint8_t Delete_Duration_Timer(uint8_t index) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return TM_FAIL; // 无效索引
	}
	uint8_t PB_Pool_Index = index - 1;
	PB_pool[PB_Pool_Index].loop_count = 0;
	if (PB_pool[PB_Pool_Index].duration_timer != NULL) {
		PB_pool[PB_Pool_Index].duration_timer_remaining_ticks = 0;
		for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
			if (xTimerDelete(PB_pool[PB_Pool_Index].duration_timer,
							portMAX_DELAY) == pdPASS) {
				PB_pool[PB_Pool_Index].duration_timer = NULL; // 删除成功后将句柄重置为 NULL
				break; // 删除成功，退出循环
			}
			// 如果在重试3次后仍然失败，则返回 TM_FAIL
			if (retry_count == 2) {
				return TM_FAIL;
			}
		}
	}
	return TM_OK;
}

uint8_t Delete_Sample_Period_Timer(uint8_t index) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return TM_FAIL; // 无效索引
	}
	uint8_t PB_Pool_Index = index - 1;
	PB_pool[PB_Pool_Index].loop_count = 0;
	if (PB_pool[PB_Pool_Index].sample_period_timer != NULL) {
		PB_pool[PB_Pool_Index].sample_period_timer_remaining_ticks = 0;
		for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
			if (xTimerDelete(PB_pool[PB_Pool_Index].sample_period_timer,
							portMAX_DELAY) == pdPASS) {
				PB_pool[PB_Pool_Index].sample_period_timer = NULL; // 删除成功后将句柄重置为 NULL
				break; // 删除成功，退出循环
			}
			// 如果在重试3次后仍然失败，则返回 TM_FAIL
			if (retry_count == 2) {
				return TM_FAIL;
			}
		}
	}
	return TM_OK;
}
