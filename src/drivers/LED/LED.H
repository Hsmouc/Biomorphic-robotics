#ifndef __LED_H__
#define __LED_H__




typedef enum LEDn
{
  LED0,              //��Ӧ�����ź�
  LED1,
  LED2,
  LED3,
  LED4,
  LED5,
  LED6,
  LED7,
  LED8,
  LED9
    
}LEDn;



//LED����״̬�궨��
typedef enum LED_status
{
  LED_OFF=0,         //����(��Ӧ�͵�ƽ)
  LED_ON=1           //�ư�(��Ӧ�ߵ�ƽ)
}LED_status;



extern void    LED_init(void);                                                 // LED��ʼ��    ����
extern void    LED(LEDn ledn,LED_status status);
extern void    LED_turn(LEDn ledn);
extern void    water_lights(void);                                               // ��ˮ��

//������ʱ,������ˮ�Ƶ���ʱ
#include "delay.h"
#define   LED_DELAY           delay()
#define   LED_DELAY_MS(x)     delayms(x)

#endif