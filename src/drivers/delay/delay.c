/****************************************
 * �ļ���       ��delay.c
 * ����         ����ʱ��������
 *

**********************************************************************************/

#include  "delay.h"


/*************************************************************************
*
*  �������ƣ�delay
*  ����˵������ʱ��������׼ȷ��
*  ����˵������
*  �������أ���
*  ��    ע��
*************************************************************************/
void  delay(void)
{
    delayms(500);
}

/*************************************************************************
*
*  �������ƣ�delayms
*  ����˵������ʱ��������׼ȷ�����ں�Ƶ��Ϊ100Mʱ��Ϊ׼ȷ
*  ����˵������
*  �������أ���
*  ��    ע��
*************************************************************************/
void  delayms(uint32  ms)
{

    uint32  i, j;
    for(i = 0; i < ms; i++)
    {
        for(j = bus_clk_khz; j > 0; j--)
        {
            asm("nop");
        }
    }
}