/*
 * pid_op.h
 *
 *  Created on: Oct 28, 2024
 *      Author: Sab1e
 */
//Effect Operation
#ifndef PID_OP_H_
#define PID_OP_H_

#define OP_SUCCESS 1
#define OP_FAIL 0

uint8_t OP_Effect_Start(uint8_t index, uint8_t loop_count);
uint8_t OP_Effect_Start_Solo(uint8_t index, uint8_t loop_count);
uint8_t OP_Effect_Stop(uint8_t index);


#endif /* PID_OP_H_ */
