
#include "global.h"

//#define UART_BUFFER_SIZE 256
//extern UART_HandleTypeDef huart1;
extern USBD_HandleTypeDef hUsbDeviceFS;

#define MAX_REPORT_SIZE 64
#define USB_PRINTF_REPORT_ID 99
//static char uartTxBuffer[UART_BUFFER_SIZE];
//
///**
// * @brief 打印文本到USART1
// * @param 文本内容，格式与标准库的printf函数一致
// * @return 无返回值
// */
//void uart_printf(const char *fmt, ...)
//{
//    va_list args;
//    va_start(args, fmt);
//    vsnprintf(uartTxBuffer, UART_BUFFER_SIZE, fmt, args);
//    va_end(args);
//    // 使用阻塞方式发送数据
//	HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), HAL_MAX_DELAY);
//}


void usb_printf(const char* format, ...) {
    char report[MAX_REPORT_SIZE];
    va_list args;

    // 使用可变参数格式化字符串
    va_start(args, format);
    int length = vsnprintf(report + 1, MAX_REPORT_SIZE - 1, format, args); // 报告的第一个字节用于Report ID
    va_end(args);

    // 设置Report ID
    report[0] = USB_PRINTF_REPORT_ID;

    // 确保不超过最大报告大小
    if (length >= MAX_REPORT_SIZE - 1) {
        length = MAX_REPORT_SIZE - 2; // 留一个字节给结尾
    }

    // 发送报告
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)report, length + 1); // length + 1包括Report ID
}














