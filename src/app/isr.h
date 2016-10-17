/******************** ********************
 * �ļ���       ��isr.h
 * ����         �����º궨���жϺţ���ӳ���ж�����������жϺ�����ַ��

**********************************************************************************/



#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"

/*                          ���¶����ж�������
 *  ��ȡ��Ĭ�ϵ��ж�����Ԫ�غ궨��       #undef  VECTOR_xxx
 *  �����¶��嵽�Լ���д���жϺ���       #define VECTOR_xxx    xxx_IRQHandler
 *  ���磺
 *       #undef  VECTOR_003
 *       #define VECTOR_003    HardFault_Handler    ���¶���Ӳ���Ϸ��жϷ�����
 */

/*
#undef  VECTOR_084
#define VECTOR_084    PIT0_IRQHandler     //���¶���84���ж�ΪPIT0_IRQHandler�ж�


extern void PIT0_IRQHandler();            //PIT0 ��ʱ�жϷ�����

*/


extern void servo();
extern void MPU6050();
extern void uart_Rx();
extern void NRF();
extern void wifi();
extern void Voltage();
extern void Ultrasonic();
extern void system_init();

/*                          ���¶����ж�������
 *  ��ȡ��Ĭ�ϵ��ж�����Ԫ�غ궨��       #undef  VECTOR_xxx
 *  �����¶��嵽�Լ���д���жϺ���       #define VECTOR_xxx    xxx_IRQHandler
 *  ���磺
 *       #undef  VECTOR_003
 *       #define VECTOR_003    HardFault_Handler    ���¶���Ӳ���Ϸ��жϷ�����
 */

#undef  VECTOR_016                        //ȡ���жϺŵĶ���
#define VECTOR_016    DMA0_IRQHandler    //PORTE�ж�

#undef  VECTOR_105                        //ȡ���жϺŵĶ���
#define VECTOR_105    PORTC_IRQHandler    //PORTE�ж�

extern void PORTC_IRQHandler();           //PORTA�жϷ�����
extern void DMA0_IRQHandler();

#undef  VECTOR_085
#define VECTOR_085    PIT1_IRQHandler     //���¶���85���ж�ΪPIT1_IRQHandler�ж�
extern void PIT1_IRQHandler();                 //PIT1 ��ʱ�жϷ�����



extern void motionGet(u8,u8*);
extern void motionCtr(u8*);

#endif  //__ISR_H

/* End of "isr.h" */
