#ifndef __UART_OOP_H
#define __UART_OOP_H
#include "main.h"

typedef struct _UART_Device UART_Device;

typedef struct _UART_Device
{
	char *name;
	int (*Init)(UART_Device* self);
	int (*Send)(UART_Device* self,uint8_t *datas,uint32_t len,int timeout);
	int (*RecvByte)(UART_Device* self, uint8_t *data,uint16_t len, int timeout);
	int (*Flush)(UART_Device* self);
    void* priv_data;
}UART_Device;

UART_Device* GetUARTDevice(char *name);

#endif