#include "main.h"
#include "UART_OOP.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
//轮询
static int uart_init(UART_Device* self)
{
    return 0;
}
static int uart_send(UART_Device* self, uint8_t *datas, uint32_t len, int timeout)
{
    UART_HandleTypeDef *huart = self->priv_data;
    if (HAL_OK == HAL_UART_Transmit(huart, datas, len, timeout))
        return 0;
    else
        return -1;
}
static int uart_recv(UART_Device* self, uint8_t *data,uint16_t len, int timeout)
{
    UART_HandleTypeDef *huart = self->priv_data;
    if (HAL_OK == HAL_UART_Receive(huart, data, 1, timeout))
        return 0;
    else
        return -1;
}
static int uart_flush(UART_Device* self)
{
    return 0;
}
//中断_立即返回
static int uart_send_it(UART_Device* self, uint8_t *datas, uint32_t len, int timeout)
{
    UART_HandleTypeDef *huart = self->priv_data;
    if (HAL_OK == HAL_UART_Transmit_IT(huart, datas, len))
        return 0;
    else
        return -1;
}
static int uart_recv_it(UART_Device* self, uint8_t *data,uint16_t len, int timeout)
{
    UART_HandleTypeDef *huart = self->priv_data;
    if (HAL_OK == HAL_UART_Receive_IT(huart, data, 1))
        return 0;
    else
        return -1;
}
//Device
static UART_Device g_uart1_dev = {"uart1", uart_init, uart_send, uart_recv, uart_flush, &huart1};
static UART_Device g_uart1_it_dev = {"uart1_it", uart_init, uart_send_it, uart_recv_it, uart_flush, &huart1};

static UART_Device *g_uart_devices[] = {&g_uart1_dev,&g_uart1_it_dev};


UART_Device* GetUARTDevice(char *name)
{
	int i = 0;
	for (i = 0; i < sizeof(g_uart_devices)/sizeof(g_uart_devices[0]); i++)
	{
		if (!strcmp(name, g_uart_devices[i]->name))
			return g_uart_devices[i];
	}
	
	return NULL;
}