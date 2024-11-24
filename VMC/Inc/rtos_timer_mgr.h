/*
 * rtos_timer_mgr.h
 *
 *  Created on: Oct 29, 2024
 *      Author: Sab1e
 */
// TM Timer Manager
#ifndef RTOS_TIMER_MGR_H_
#define RTOS_TIMER_MGR_H_



typedef enum{
	TM_OK,TM_FAIL
}Timers_StatesTypeDef;

uint8_t Pause_All_Timers(void);
uint8_t Resume_All_Timers(void);
uint8_t Delete_All_Timers(void);
uint8_t Delete_Timer(uint8_t index);
uint8_t Delete_Duration_Timer(uint8_t index);
uint8_t Delete_Sample_Period_Timer(uint8_t index);
uint8_t Start_Timers(uint8_t index);
#endif /* RTOS_TIMER_MGR_H_ */
