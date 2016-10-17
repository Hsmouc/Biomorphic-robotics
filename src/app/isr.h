/******************** ********************
 * 文件名       ：isr.h
 * 描述         ：重新宏定义中断号，重映射中断向量表里的中断函数地址，

**********************************************************************************/



#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"

/*                          重新定义中断向量表
 *  先取消默认的中断向量元素宏定义       #undef  VECTOR_xxx
 *  在重新定义到自己编写的中断函数       #define VECTOR_xxx    xxx_IRQHandler
 *  例如：
 *       #undef  VECTOR_003
 *       #define VECTOR_003    HardFault_Handler    重新定义硬件上访中断服务函数
 */

/*
#undef  VECTOR_084
#define VECTOR_084    PIT0_IRQHandler     //重新定义84号中断为PIT0_IRQHandler中断


extern void PIT0_IRQHandler();            //PIT0 定时中断服务函数

*/


extern void servo();
extern void MPU6050();
extern void uart_Rx();
extern void NRF();
extern void wifi();
extern void Voltage();
extern void Ultrasonic();
extern void system_init();

/*                          重新定义中断向量表
 *  先取消默认的中断向量元素宏定义       #undef  VECTOR_xxx
 *  在重新定义到自己编写的中断函数       #define VECTOR_xxx    xxx_IRQHandler
 *  例如：
 *       #undef  VECTOR_003
 *       #define VECTOR_003    HardFault_Handler    重新定义硬件上访中断服务函数
 */

#undef  VECTOR_016                        //取消中断号的定义
#define VECTOR_016    DMA0_IRQHandler    //PORTE中断

#undef  VECTOR_105                        //取消中断号的定义
#define VECTOR_105    PORTC_IRQHandler    //PORTE中断

extern void PORTC_IRQHandler();           //PORTA中断服务函数
extern void DMA0_IRQHandler();

#undef  VECTOR_085
#define VECTOR_085    PIT1_IRQHandler     //重新定义85号中断为PIT1_IRQHandler中断
extern void PIT1_IRQHandler();                 //PIT1 定时中断服务函数



extern void motionGet(u8,u8*);
extern void motionCtr(u8*);

#endif  //__ISR_H

/* End of "isr.h" */
