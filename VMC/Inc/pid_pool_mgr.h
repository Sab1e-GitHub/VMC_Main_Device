/*
 * pid_pool_mgr.h
 *
 *  Created on: Oct 27, 2024
 *      Author: Sab1e
 */

#ifndef PID_POOL_MGR_H_
#define PID_POOL_MGR_H_

//Ч�������ع���
#define PB_POOL_ARRAY_SIZE 9
#define PB_SIZE sizeof(Effect) // ����ÿ��������Ĵ�СΪEffect�ṹ��Ĵ�С

#define MAX_TYPE_SPECIFIC_BLOCKS 2

#define PM_FAIL 0
#define PM_SUCCESS 1

#define UNUSED_PB 0

#define BLOCK_LOAD_SUCCESS 0x01
#define BLOCK_LOAD_FULL 0x02
#define BLOCK_LOAD_ERROR 0x03

#define TYPE_SPECIFIC_PB_INDEX_1 0
#define TYPE_SPECIFIC_PB_INDEX_2 1

typedef enum {
	UNUSED_EFFECT_TYPE,
	CONSTANT_FORCE,
	SINE,
	SPRING,
	DAMPER,
	FRICTION,
	INERTIA,
	SQUARE,
	TRIANGLE,
	RAMP,
	SAWTOOTH_UP,
	SAWTOOTH_DOWN
} Effect_TypeDef;

typedef enum {
	UNUSED_TYPE_SPECIFIC_EFFECT_TYPE,
	Envelope,
	Condition,
	Constant_Force,
	Periodic,
	Ramp_Force
} Type_Sepcific_Effect_TypeDef;

typedef enum {
	EFFECT_STOPPED, EFFECT_RUNNING, EFFECT_PAUSED, EFFECT_WAITING
} Effect_StateTypeDef;

typedef struct {
	uint8_t effect_PB_index;
	Effect_TypeDef effect_type;
	uint16_t duration;
	uint8_t trigger_repeat_interval;			//NOUSED
	uint8_t sample_period;
	uint16_t gain;
	uint8_t trigger_button;							//NOUSED
	uint8_t axes_direction_enable;
	uint8_t direction_instance1;
	uint8_t direction_instance2;
} Effect_PB;

typedef struct {
	uint8_t attack_level;           // ��������
	uint8_t fade_level;             // ˥�˼���
	uint8_t attack_time;            // ����ʱ��
	uint8_t fade_time;              // ˥��ʱ��
} Envelope_PB;

typedef struct {
	int16_t center_point_offset;
	int16_t positive_coefficient;
	int16_t negative_coefficient;
	uint16_t positive_saturation;
	uint16_t negative_saturation;
	uint16_t dead_band;
} Condition_PB;

typedef struct {
	int16_t magnitude;              // ���Ĵ�С
} Constant_Force_PB;

typedef struct {
	uint16_t ramp_start;
	uint16_t ramp_end;
} Ramp_Force_PB;

typedef struct {
	uint8_t phase;
	uint16_t magnitude;
	int16_t offset;
	uint16_t period;
} Periodic_PB;

typedef struct {
	Type_Sepcific_Effect_TypeDef type;
	union {
		Envelope_PB envelope_parameters;
		Condition_PB condition_parameters;
		Constant_Force_PB constant_force_parameters;
		Periodic_PB periodic_parameters;
		Ramp_Force_PB ramp_force_parameters;
	} Parameters;
} Type_Specific_Parameters_Array_t[MAX_TYPE_SPECIFIC_BLOCKS];

typedef struct {
	Effect_StateTypeDef effect_state;
	uint32_t time_step;	// Ч�����е���ʱ�䲽 ��λ���� ÿ��������϶�����1 �´����ж�ȡ����ͣʱ������  ����ѭ���� ���һ���ֻؾͽ�����ʼ��
	Effect_PB effect_parameter;          // Ч������
	Type_Specific_Parameters_Array_t parameters_array;
	TimerHandle_t duration_timer;        // ��ӣ�Duration��ʱ�����
	TimerHandle_t sample_period_timer;   // ��ӣ�Sample Period��ʱ�����
	TickType_t duration_timer_remaining_ticks;	//duration_timerʣ��ʱ��
	TickType_t sample_period_timer_remaining_ticks;	//sample_period_timerʣ��ʱ��
	uint8_t loop_count;	//ѭ������
} Effect;

typedef struct {
	uint8_t index;
	uint8_t result;
} PM_Create_New_Effect_PB_Results_t;

void Create_New_Effect_PB(Effect_TypeDef effect_type);

uint16_t Get_Available_RAM_Size();

uint8_t Set_Effect_Parameter(uint8_t index, Effect_TypeDef effect_type,
				uint16_t duration, uint8_t trigger_repeat_interval,
				uint8_t sample_period, uint16_t gain, uint8_t trigger_button,
				uint8_t axes_direction_enable, uint8_t direction_instance1,
				uint8_t direction_instance2);
uint8_t Add_Constant_Force_PB(uint8_t index, int16_t magnitude);
uint8_t Add_Ramp_Force_PB(uint8_t index, uint16_t ramp_start, uint16_t ramp_end);
uint8_t Add_Envelope_PB(uint8_t index, uint8_t attack_level, uint8_t fade_level,
				uint8_t attack_time, uint8_t fade_time);
uint8_t Add_Periodic_PB(uint8_t index, uint8_t phase, uint16_t magnitude,
				int16_t offset, uint16_t period);
uint8_t Add_Condition_PB(uint8_t index, int16_t center_point_offset,
				int16_t positive_coefficient, int16_t negative_coefficient,
				uint16_t positive_saturation, uint16_t negative_saturation,
				uint16_t dead_band);
//uint8_t Add_Type_Specific_PB(uint8_t index,
//				Type_Sepcific_Effect_TypeDef effect_type);

uint8_t Free_Effect_PB(uint8_t index);

extern PM_Create_New_Effect_PB_Results_t PM_Create_New_Effect_PB_Results;
extern Effect PB_pool[PB_POOL_ARRAY_SIZE];
#endif /* PID_POOL_MGR_H_ */
