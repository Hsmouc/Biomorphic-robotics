/********************  ********************
 * �ļ���       ��main.c

**********************************************************************************/

#include "common.h"
#include "include.h"

u8 interrupt_mark1=0;     //ȫ�ֱ���
u8 servo_init_mark;
u8 uart_Rx_dat[10000];
u32 receive_add;    //���ڽ������ݳ���  
u8 receive_true_mark=0;
u8 uart_receive_mark=0;

void main(void)
{
    volatile u32 i;
    u8 status;	//�����жϽ���/����״̬
    u8 txbuf[32]={0x1f,2,3,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};	 //���ͻ���
    u8 rxbuf[32];
    u8 Tx_Rx_mark=0;
    gpio_init  (PORTA, 5, GPO, LOW);
    gpio_init  (PORTC, 5, GPO, LOW);
    gpio_init  (PORTC, 6, GPO, LOW);
    gpio_init  (PORTA, 19, GPO, LOW);
    NRF_Init();

    while(1)
    {
          if(Tx_Rx_mark==0)
          {
		/*����ģʽ*/
                NRF_TX_Mode();
		
		/*��ʼ��������*/
		status = NRF_Tx_Dat(txbuf,MAX_ONCE_TX_NUM);
		
		/*�жϷ���״̬*/  
		if(status& TX_DS)
		{
                        PTA5_OUT=1;
                        PTC5_OUT=0;
                        PTC6_OUT=0;
                        Tx_Rx_mark=1;
		}
		else if(status & MAX_RT)
		{
			PTA5_OUT=0;
                        PTC5_OUT=0;
                        PTC6_OUT=1;
		}
		else
		{
			PTA5_OUT=0;
                        PTC5_OUT=0;
                        PTC6_OUT=1;
		}
          }
          else if(Tx_Rx_mark==1)
          {
                /*����ģʽ*/
                NRF_RX_Mode();
                /*�ȴ���������*/
		status = NRF_Rx_Dat(rxbuf);
		/*�жϽ���״̬*/
		switch(status)
		{
                    case RX_DR:
                    {					
                          PTA5_OUT=0;
                          PTC5_OUT=1;
                          PTC6_OUT=0;
                          Tx_Rx_mark=0;
                    }
                    break;
      
                    default:
                    {
                          PTA5_OUT=0;
                          PTC5_OUT=0;
                          PTC6_OUT=1;
                    }
                    break;  	
		}
                
                if(rxbuf[0]==0x1f)
                {
                    PTA19_OUT=1;
                }
                else
                {
                    PTA19_OUT=0;               
                }
          
          }
    } 
}
