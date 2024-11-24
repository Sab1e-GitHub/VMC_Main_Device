/*
 * pid_state.c
 *
 *  Created on: Oct 26, 2024
 *      Author: Sab1e
 */
#include "global.h"

#define SendReport(report) \
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report))

void Send_PIDStateReport(void){

}
