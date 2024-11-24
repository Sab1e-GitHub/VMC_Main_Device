/*
 * rtos_timer_mgr.c
 *
 *  Created on: Oct 29, 2024
 *      Author: Sab1e
 */

#include "global.h"
/**
 * ����ɾ����ʱ���Ĳ����������Loop Count
 */



//Ч����ʱ���Ļص����� Duration Timer
void Effect_Timer_Callback(TimerHandle_t xTimer) {
	uint32_t index = (uint32_t) pvTimerGetTimerID(xTimer);	//��ȡ����������ֵ
	// ��Effectִ����Ϻ��ֹͣ
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return; // ��Ч����
	}
	uint8_t PB_Pool_Index = index - 1;
	//ִ����Ͽ��Ƿ���Ҫ���� Loop Count>0
	//���>0������������ʱ����������һ�ֵ�����
	if (PB_pool[PB_Pool_Index].loop_count == LOOP_INFINITE) {	//����ѭ��
		PB_pool[PB_Pool_Index].effect_state = EFFECT_RUNNING;
		xTimerStart(PB_pool[PB_Pool_Index].duration_timer, 0);
	} else if (PB_pool[PB_Pool_Index].loop_count > 0) {
		PB_pool[PB_Pool_Index].effect_state = EFFECT_RUNNING;
		xTimerStart(PB_pool[PB_Pool_Index].duration_timer, 0);
		PB_pool[PB_Pool_Index].loop_count--;
	} else {
		PB_pool[PB_Pool_Index].effect_state = EFFECT_STOPPED;
		Delete_Duration_Timer(PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index);
		//����ɾ����ʱ���ĺ����������loop_count����˲����ڴ˴����loop_count
	}

	return;
}
// ���ָ��ʱ���ִ��һ��Ч��
void Effect_Sample_Period_Callback(TimerHandle_t xTimer) {
	uint32_t index = (uint32_t) pvTimerGetTimerID(xTimer) - PB_POOL_ARRAY_SIZE;	//��ȡ����������ֵ ȥ��PB_POOL_ARRAY_SIZE��С��ƫ�Ʋ�������ֵ
	// ��Effectִ����Ϻ��ֹͣ
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return; // ��Ч����
	}
	uint8_t PB_Pool_Index = index - 1;
	PB_pool[PB_Pool_Index].effect_state = EFFECT_RUNNING; //��ʼ���У�������������һ�κ��Զ��ָ�EFFECT_WAITING
	return;
}

uint8_t Start_Timers(uint8_t index){
	uint8_t PB_Pool_Index = index - 1;
	// ���� Duration ��ʱ��
		if (PB_pool[PB_Pool_Index].effect_parameter.duration != DURATION_INFINITE) {
			if (PB_pool[PB_Pool_Index].duration_timer != NULL) {
				xTimerDelete(PB_pool[PB_Pool_Index].duration_timer, 0); // ɾ���ɵĶ�ʱ������������ڴ�й©
			}

			PB_pool[PB_Pool_Index].duration_timer =
							xTimerCreate("EffectTimer",
											pdMS_TO_TICKS(
															PB_pool[PB_Pool_Index].effect_parameter.duration),
											pdFALSE,
											(void*) (uintptr_t) PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index, //ֱ��ʹ��index��Ϊ����
											Effect_Timer_Callback);

			if (PB_pool[PB_Pool_Index].duration_timer != NULL) {
				xTimerStart(PB_pool[PB_Pool_Index].duration_timer, 0);
			} else {
				return TM_FAIL;  // ��ʱ������ʧ��
			}
		}

		/**
		 * ��� Sample Period ��ΪĬ�ϵĲ�����
		 * �͵����������� Sample Period ��ʱ��
		 * Ȼ����������ʱ����Ӧ�Ĳ���������������Effect
		 */
		if (PB_pool[PB_Pool_Index].effect_parameter.sample_period
						!= DEFAULT_SAMPLE_RATE) {
			if (PB_pool[PB_Pool_Index].sample_period_timer != NULL) {
				xTimerDelete(PB_pool[PB_Pool_Index].sample_period_timer, 0); // ɾ���ɵĶ�ʱ������������ڴ�й©
			}

			PB_pool[PB_Pool_Index].sample_period_timer =
							xTimerCreate("EffectSamplePeriodTimer",
											pdMS_TO_TICKS(
															PB_pool[PB_Pool_Index].effect_parameter.sample_period),
											pdTRUE,  // �Զ�����
											(void*) (uintptr_t) PB_pool[PB_Pool_Index].effect_parameter.effect_PB_index
															+ PB_POOL_ARRAY_SIZE, //����ID�����ظ������PB_POOL_ARRAY_SIZE��С��ƫ����ΪID
											Effect_Sample_Period_Callback);

			if (PB_pool[PB_Pool_Index].sample_period_timer != NULL) {
				xTimerStart(PB_pool[PB_Pool_Index].sample_period_timer, 0);
			} else {
				return TM_FAIL;  // ��ʱ������ʧ��
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
			xTimerStop(PB_pool[i].sample_period_timer, 0);  // ֹͣ��ʱ��
		}
	}
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].duration_timer != NULL) {
			PB_pool[i].duration_timer_remaining_ticks = xTimerGetExpiryTime(
							PB_pool[i].duration_timer) - xTaskGetTickCount();
			xTimerStop(PB_pool[i].duration_timer, 0);  // ֹͣ��ʱ��
		}
	}
	return TM_OK;
}
//// ��ͣDuration��ʱ��
//void Pause_Duration_Timer(uint8_t index) {
//	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
//		return; // ��Ч����
//	}
//	uint8_t PB_Pool_Index = index - 1;
//	PB_pool[PB_Pool_Index].duration_timer_remaining_ticks = xTimerGetExpiryTime(
//					PB_pool[PB_Pool_Index].duration_timer) - xTaskGetTickCount();
//	xTimerStop(PB_pool[PB_Pool_Index].duration_timer, 0);  // ֹͣ��ʱ��
//	return;
//
//}
//
//void Pause_Sample_Period_Timer(uint8_t index) {
//	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
//		return; // ��Ч����
//	}
//	uint8_t PB_Pool_Index = index - 1;
//	PB_pool[PB_Pool_Index].sample_period_timer_remaining_ticks =
//					xTimerGetExpiryTime(xTimer) - xTaskGetTickCount();
//	xTimerStop(xTimer, 0);  // ֹͣ��ʱ��
//	return;
//
//}

// �ָ���ʱ��
uint8_t Resume_All_Timers(void) {
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].duration_timer_remaining_ticks != 0) {
			PB_pool[i].duration_timer_remaining_ticks = 0;
			xTimerChangePeriod(PB_pool[i].duration_timer,
							PB_pool[i].duration_timer_remaining_ticks, 0); // �ָ���ʱ��������
			xTimerStart(PB_pool[i].duration_timer, 0);  // ������ʱ��
		}
	}
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].sample_period_timer_remaining_ticks != 0) {
			PB_pool[i].sample_period_timer_remaining_ticks = 0;
			xTimerChangePeriod(PB_pool[i].sample_period_timer,
							PB_pool[i].sample_period_timer_remaining_ticks, 0); // �ָ���ʱ��������
			xTimerStart(PB_pool[i].sample_period_timer, 0);  // ������ʱ��
		}
	}
	return TM_OK;
}
/**
 * @brief ɾ������PB_Pool�ڵ����ж�ʱ��
 * @param �޲���
 * @return TM_OKָɾ���ɹ� TM_FAILָɾ��ʧ��
 */
uint8_t Delete_All_Timers(void) {
	for (uint8_t i = 0; i < PB_POOL_ARRAY_SIZE; i++) {
		if (PB_pool[i].duration_timer != NULL) {
			PB_pool[i].duration_timer_remaining_ticks = 0;
			for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
				if (xTimerDelete(PB_pool[i].duration_timer,
								portMAX_DELAY) == pdPASS) {
					PB_pool[i].duration_timer = NULL; // ɾ���ɹ��󽫾������Ϊ NULL
					PB_pool[i].loop_count = 0;
					break; // ɾ���ɹ����˳�ѭ��
				}
				// ���������3�κ���Ȼʧ�ܣ��򷵻� TM_FAIL
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
					PB_pool[i].sample_period_timer = NULL; // ɾ���ɹ��󽫾������Ϊ NULL
					PB_pool[i].loop_count = 0;
					break; // ɾ���ɹ����˳�ѭ��
				}
				// ���������3�κ���Ȼʧ�ܣ��򷵻� TM_FAIL
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
		return TM_FAIL; // ��Ч����
	}
	uint8_t PB_Pool_Index = index - 1;
	if (PB_pool[PB_Pool_Index].duration_timer != NULL) {
		PB_pool[PB_Pool_Index].duration_timer_remaining_ticks = 0;
		for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
			if (xTimerDelete(PB_pool[PB_Pool_Index].duration_timer,
							portMAX_DELAY) == pdPASS) {
				PB_pool[PB_Pool_Index].duration_timer = NULL; // ɾ���ɹ��󽫾������Ϊ NULL
				break; // ɾ���ɹ����˳�ѭ��
			}
			// ���������3�κ���Ȼʧ�ܣ��򷵻� TM_FAIL
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
				PB_pool[PB_Pool_Index].sample_period_timer = NULL; // ɾ���ɹ��󽫾������Ϊ NULL
				break; // ɾ���ɹ����˳�ѭ��
			}
			// ���������3�κ���Ȼʧ�ܣ��򷵻� TM_FAIL
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
		return TM_FAIL; // ��Ч����
	}
	uint8_t PB_Pool_Index = index - 1;
	PB_pool[PB_Pool_Index].loop_count = 0;
	if (PB_pool[PB_Pool_Index].duration_timer != NULL) {
		PB_pool[PB_Pool_Index].duration_timer_remaining_ticks = 0;
		for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
			if (xTimerDelete(PB_pool[PB_Pool_Index].duration_timer,
							portMAX_DELAY) == pdPASS) {
				PB_pool[PB_Pool_Index].duration_timer = NULL; // ɾ���ɹ��󽫾������Ϊ NULL
				break; // ɾ���ɹ����˳�ѭ��
			}
			// ���������3�κ���Ȼʧ�ܣ��򷵻� TM_FAIL
			if (retry_count == 2) {
				return TM_FAIL;
			}
		}
	}
	return TM_OK;
}

uint8_t Delete_Sample_Period_Timer(uint8_t index) {
	if (index == UNUSED_PB || index > PB_POOL_ARRAY_SIZE) {
		return TM_FAIL; // ��Ч����
	}
	uint8_t PB_Pool_Index = index - 1;
	PB_pool[PB_Pool_Index].loop_count = 0;
	if (PB_pool[PB_Pool_Index].sample_period_timer != NULL) {
		PB_pool[PB_Pool_Index].sample_period_timer_remaining_ticks = 0;
		for (uint8_t retry_count = 0; retry_count < 3; retry_count++) {
			if (xTimerDelete(PB_pool[PB_Pool_Index].sample_period_timer,
							portMAX_DELAY) == pdPASS) {
				PB_pool[PB_Pool_Index].sample_period_timer = NULL; // ɾ���ɹ��󽫾������Ϊ NULL
				break; // ɾ���ɹ����˳�ѭ��
			}
			// ���������3�κ���Ȼʧ�ܣ��򷵻� TM_FAIL
			if (retry_count == 2) {
				return TM_FAIL;
			}
		}
	}
	return TM_OK;
}
