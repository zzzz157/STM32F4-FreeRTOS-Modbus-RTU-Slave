# STM32-FreeRTOS-Modbus-RTU-Slave

基于 STM32 HAL 库和 FreeRTOS 实现的高效、稳定、非阻塞式 Modbus RTU 从机（Slave）协议栈。

##  项目亮点

* **精准的硬件断帧**：使用硬件定时器（TIM）严格计算并匹配 Modbus 标准的 3.5 字符时间，彻底解决丢包和粘包问题。
* **高实时性（非阻塞）**：结合 FreeRTOS 的二值信号量（Semaphore），在中断中释放信号量，在 Task 中阻塞等待。极大地降低了 CPU 占用率，不影响其他高优先级任务。
* **面向对象（OOP）封装**：底层 UART 驱动采用 C 语言结构体与函数指针进行解耦。方便一键切换轮询/中断/DMA模式，且极易移植到其他 MCU 平台。
* **代码结构清晰**：数据区（线圈、寄存器）与协议解析逻辑严格分离，二次开发极其简单。

##  支持的 Modbus 功能码

| 功能码 (Hex) | 功能描述 | 操作对象 |
| :---: | :--- | :--- |
| `0x01` | 读线圈 (Read Coils) | `buf_bits` |
| `0x02` | 读离散输入 (Read Discrete Inputs) | `buf_input_bits` |
| `0x03` | 读保持寄存器 (Read Holding Registers)| `buf_registers` |
| `0x04` | 读输入寄存器 (Read Input Registers) | `buf_input_registers` |
| `0x05` | 写单个线圈 (Write Single Coil) | `buf_bits` |
| `0x06` | 写单个寄存器 (Write Single Register) | `buf_registers` |
| `0x0F` | 写多个线圈 (Write Multiple Coils) | `buf_bits` |
| `0x10` | 写多个寄存器 (Write Multiple Registers)| `buf_registers` |

##  快速上手

代码的初始化和运行非常简单，只需在 FreeRTOS 的 Task 中调用即可：

```c
#include "Modbus_Rtu.h"

Modbus_RTU* modbus_dev = NULL;

void Modbus_Task(void* arg)
{
    // 1. 获取设备句柄
    modbus_dev = &modbus_rtu_dev1;
    
    // 2. 初始化：设置从机地址为 0x01（任意），绑定底层串口 "uart1_it"
    modbus_dev->Init(modbus_dev, 0x01, "uart1_it");
    
    while(1)
    {
        // 3. 阻塞式等待并处理 Modbus 数据包（内部由信号量调度，不占 CPU）
        modbus_dev->Receive(modbus_dev);
    }
}
