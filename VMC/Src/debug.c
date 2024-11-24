
#include "global.h"

//#define UART_BUFFER_SIZE 256
//extern UART_HandleTypeDef huart1;
extern USBD_HandleTypeDef hUsbDeviceFS;

#define MAX_REPORT_SIZE 64
#define USB_PRINTF_REPORT_ID 99
//static char uartTxBuffer[UART_BUFFER_SIZE];
//
///**
// * @brief ��ӡ�ı���USART1
// * @param �ı����ݣ���ʽ���׼���printf����һ��
// * @return �޷���ֵ
// */
//void uart_printf(const char *fmt, ...)
//{
//    va_list args;
//    va_start(args, fmt);
//    vsnprintf(uartTxBuffer, UART_BUFFER_SIZE, fmt, args);
//    va_end(args);
//    // ʹ��������ʽ��������
//	HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), HAL_MAX_DELAY);
//}


void usb_printf(const char* format, ...) {
    char report[MAX_REPORT_SIZE];
    va_list args;

    // ʹ�ÿɱ������ʽ���ַ���
    va_start(args, format);
    int length = vsnprintf(report + 1, MAX_REPORT_SIZE - 1, format, args); // ����ĵ�һ���ֽ�����Report ID
    va_end(args);

    // ����Report ID
    report[0] = USB_PRINTF_REPORT_ID;

    // ȷ����������󱨸��С
    if (length >= MAX_REPORT_SIZE - 1) {
        length = MAX_REPORT_SIZE - 2; // ��һ���ֽڸ���β
    }

    // ���ͱ���
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)report, length + 1); // length + 1����Report ID
}














