#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "UART_OOP.h"

#define MODBUS_MAX_ADU 256 

#define NUM_BITS 			10
#define NUM_INPUT_BITS 		10
#define NUM_REGISTERS 		10
#define NUM_INPUT_REGISTERS 10

#define ILLEGAL_FUNCTION 0x01
#define ILLEGAL_ADDRESS  0x02
#define ILLEGAL_DATA 	 0x03

extern uint8_t buf_bits[NUM_BITS];
extern uint8_t buf_input_bits[NUM_INPUT_BITS];
extern uint16_t buf_registers[NUM_REGISTERS];
extern uint16_t buf_input_registers[NUM_INPUT_REGISTERS];

typedef struct _Modbus_RTU Modbus_RTU;

typedef struct _Modbus_RTU
{
    uint8_t slave_id;       		// 本机从机地址
	void (*Init)(Modbus_RTU* self,uint8_t slave_id,char* uart_name);
	void (*Receive)(Modbus_RTU* self);
	void* pData;
}Modbus_RTU;

extern Modbus_RTU modbus_rtu_dev1;


#endif