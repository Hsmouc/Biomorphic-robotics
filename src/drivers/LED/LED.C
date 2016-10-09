/******************** Ƕ��ʽ���������� ********************

 * ����         ��LED���������ء�������Ϻ�������
 *

**********************************************************************************/	

#include "common.h"
#include "gpio.h"
#include  "LED.h"
#include  "gpio_cfg.h"


#define KEY()	    PTC18_IN

#define HW1()	    PTE4_IN
#define HW2()	    PTE5_IN
#define HW3()	    PTE6_IN

u8 mark=1;
/*************************************************************************
*
*  �������ƣ�LED_init
*  ����˵������ʼ��LED�˿ڣ�����IO��Ϊ�������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-2   �Ѳ���
*  ��    ע��
*************************************************************************/
void LED_init(void)
{
    //LED
    gpio_init  (PORTA, 12, GPO, HIGH);
    gpio_init  (PORTA, 13, GPO, HIGH);          
    gpio_init  (PORTA, 15, GPO, HIGH);
    
    
    gpio_init  (PORTA, 4, GPO, LOW);
    gpio_init  (PORTA, 5, GPO, LOW);          
    gpio_init  (PORTC, 5, GPO, LOW);
    gpio_init  (PORTC, 6, GPO, LOW);
    gpio_init  (PORTA, 19, GPO, LOW); 
    gpio_init  (PORTE, 26, GPO, LOW); 
    
    //��������
//    gpio_init  (PORTC, 18, GPI, LOW);
    
    //����
//    gpio_init  (PORTE, 4, GPI, LOW);
//    gpio_init  (PORTE, 5, GPI, LOW);
//    gpio_init  (PORTE, 6, GPI, LOW);
    
    //������
    gpio_init(PORTC, 14, GPO, LOW);   
    //���������Ŷ���
    gpio_init  (PORTC, 1, GPO, LOW);   //Trig
    
}

/*************************************************************************
*
*  �������ƣ�LED_ST
*  ����˵��������LED������
*  ����˵����LEDn        LED�˿ڣ�LED0��LED1��LED2��LED3��
*            status      LED״̬��LED_ON��LED_OFF��
*  �������أ���
*  �޸�ʱ�䣺2012-2-2   �Ѳ���
*  ��    ע��
*************************************************************************/
void LED(LEDn ledn,LED_status status)
{
    switch(ledn)
    {
        case 1:
        {
            if(status)
                PTA12_OUT=0;
            else
                PTA12_OUT=1;
        }break;
        case 2:
        {
            if(status)
                PTA13_OUT=0;
            else
                PTA13_OUT=1;
        }break;
        case 3:
        {
            if(status)
                PTA15_OUT=0;
            else
                PTA15_OUT=1;
        }break;        
        case 4:
        {
            if(status)
                PTA4_OUT=1;
            else
                PTA4_OUT=0;
        }break;
        case 5:
        {
            if(status)
                PTA5_OUT=1;
            else
                PTA5_OUT=0;

        }break;
        case 6:
        {
            if(status)
                PTC5_OUT=1;
            else
                PTC5_OUT=0;            

        }break;
        case 7:
        {
                    if(status)
                PTC6_OUT=1;
            else
                PTC6_OUT=0;  

        }break;
        case 8:
        {
            if(status)
                PTA19_OUT=1;
            else
                PTA19_OUT=0;

        }break;
        case 9:
        {
            if(status)
                PTE26_OUT=1;
            else
                PTE26_OUT=0;
        }break;
        default: break;
    }
}

void  LED_turn(LEDn ledn)
{
    if(mark)
    {
        LED(ledn,LED_ON);
        mark=0;
    }
    else    
    {
        LED(ledn,LED_OFF);
        mark=1;
    }
}

/*************************************************************************
*
*  �������ƣ�water_lights
*  ����˵�����ĸ�LED������˸����ˮ��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-10   �Ѳ���
*  ��    ע��
*************************************************************************/
void water_lights(void)
{

    LEDn    n;
    LED_init();
    for(n=LED0;n<=LED4;n++)
    {
        LED(n,LED_ON);
        LED_DELAY_MS(100);
        LED(n,LED_OFF);
    }
}










