
/******************** ********************
 * �ļ���       ��isr.c
 * ����         ���жϴ�������
 *

**********************************************************************************/

#include "common.h"
#include "include.h"


volatile u16 Vnum=0;

/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  �������ƣ�PORTA_IRQHandler
*  ����˵����PORTA�˿��жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-1-25    �Ѳ���
*  ��    ע�����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
void PORTC_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTC_ISFR;
    PORTC_ISFR  = ~0;                                   //���жϱ�־λ

    n = 7;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
          //���ж���Ҫ�ж��ǳ��������ǳ���ʼ
          if(img_flag == IMG_START)                   //��Ҫ��ʼ�ɼ�ͼ��
          {
              img_flag = IMG_GATHER;                  //���ͼ��ɼ���
              disable_irq(89);
      
              PORTC_ISFR = 1 << 7;            //���PCLK��־λ
              DMA_DADDR(CAMERA_DMA_CH)=  (u32)IMG_BUFF;//���ϻָ�Ŀ�ĵ�ַ
              DMA_EN(CAMERA_DMA_CH);                  //ʹ��ͨ��CHn Ӳ������
              
          }
          else                                        //ͼ��ɼ�����
          {
              disable_irq(89);                        //�ر�PTC���ж�
              img_flag = IMG_FAIL;                    //���ͼ��ɼ�ʧ��
          }
    }
  
}


void DMA0_IRQHandler()
{
    img_flag = IMG_FINISH ;
//    DMA_IRQ_DIS(CAMERA_DMA_CH);
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //���ͨ�������жϱ�־λ
}


