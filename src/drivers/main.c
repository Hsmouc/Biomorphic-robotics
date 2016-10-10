/******************** ********************
 * �ļ���       ��main.c
 * ����         ��UART���ڲ�ѯ����ʵ��
 *
**********************************************************************************/
#include "common.h"
#include "include.h"

u32 receive_add=0;

u8 interrupt_mark1=0;     //ȫ�ֱ���

u8 str[500];
u8 str1[500];
u8 str2[500];
u8 str0[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
u32 i = 0;
u32 j = 0;

u8 status;	//�����жϽ���/����״̬
u8 txbuf[32]={0x1f,2,3,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};	 //���ͻ���
u8 rxbuf[32];
u8 Tx_Rx_mark=0;  

u32 servo_position_set[32];     //����λ��
u32 servo_time_set[32];    //��������ʱ�� ��������ʱ��
u8  servo_num_set[32];        //������

u8 read_value=0;

void NRF()
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
                  LED(LED2,LED_ON);
                  LED(LED3,LED_OFF);
                  LED(LED4,LED_OFF);
                  Tx_Rx_mark=1;
          }
          else if(status & MAX_RT)
          {
                  LED(LED2,LED_OFF);
                  LED(LED3,LED_OFF);
                  LED(LED4,LED_ON);
          }
          else
          {
                  LED(LED2,LED_OFF);
                  LED(LED3,LED_OFF);
                  LED(LED4,LED_ON);
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
                  LED(LED2,LED_OFF);
                  LED(LED3,LED_ON);
                  LED(LED4,LED_OFF);
                  Tx_Rx_mark=0;
              }
              break;

              default:
              {
                  LED(LED2,LED_OFF);
                  LED(LED3,LED_OFF);
                  LED(LED4,LED_OFF);
              }
              break;  	
          }
          
          if(rxbuf[1]==2)
          {
              PTA19_OUT=1;
          }
          else
          {
              PTA19_OUT=0;               
          }
    
    }
}




void  main(void)
{
    
    uart_init(UART4, 19200);                    //��ʼ������4��������Ϊ19200 ,������̫�����ײ��ȶ�
    LED_init();
    Servo_Init();
    NRF_Init();
    AT24C02_Init();
    
//    uart_sendStr(UART4 , "\n\n\n�������ݣ������з���:");
    ReadAT24C02_flash(str1,0,500);
    uart_sendN(UART4, str1,500);
    LED(LED3,LED_OFF); 
    for(i=0;i<256;i++)
    {
        str2[i]=0xff;
    }
    for(i=0;i<244;i++)
    {
        str2[i+256]=0x01;
    }
    
    while(1)
    {
 
/*
        if(uart_query(UART4)!=0)
        {

             read_value=uart_getchar (UART4);
             WriteAT24C02_byte(1,read_value);
             DelayMs1(10);
             read_value=ReadAT24C02_byte(1);
             uart_putchar(UART4,read_value);
        }
*/
                  
        if(uart_pendstr(UART4, str)!=0)
        {
          
//            WriteAT24C02_byte(0,str[0]);
//            DelayMs1(1);
//            WriteAT24C02_byte(1,str[1]);
             WriteAT24C02_flash(str2,0,500);
/*            for(i=0;i<16;i++)
            {
                WriteAT24C02_byte(i,str[i]);
                DelayMs1(2);
            }
*/
            DelayMs1(1);
            
            ReadAT24C02_flash(str1,0,500);
/*            for(i=0;i<16;i++)
            {
                str1[i]=ReadAT24C02_byte(i);
//                DelayMs1(2);
            }
*/
//            str1[0]=ReadAT24C02_byte(0);
//            str1[1]=ReadAT24C02_byte(1);
            uart_sendN(UART4, str1,500);
        }
      
      
    }
}
