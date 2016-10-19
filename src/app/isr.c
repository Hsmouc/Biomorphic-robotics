/******************** ********************
 * �ļ���       ��isr.c
 * ����         ���жϴ�������
 *

**********************************************************************************/



#include "common.h"
#include "include.h"
#include "math.h"




u8 interrupt_mark1=0;     //ȫ�ֱ���



u32 i = 0;
u32 j = 0;

u8 system_run_mark;  //�����ʼ��־
u8 system_mark;   //ϵͳ��ʼ��־
///////////////////////////////////////////////////

u8 Motor_dat[30];      // �����Ϣ
u8 Offline_RUN;
u8 ONline_RUN;
u8 servo_SQ;
/////////////////////////////��λ����������///////////////////////////////
u8 Send_Voltage[9]       = {0x40,0x5b,0x53,0x01,0x00,0x00,0x06,0x5d,0x25};    // ���͵�ѹ
u8 Send_Ampere[9]        = {0x40,0x5b,0x53,0x02,0x00,0x00,0x06,0x5d,0x25};     // ���͵���
u8 Servo_dat_ACK[8]      = {0x40,0x5b,0x53,0x03,0x00,0x05,0x5d,0x25};       // �����Ϣ����
u8 Motor_dat_ACK[8]      = {0x40,0x5b,0x53,0x05,0x00,0x05,0x5d,0x25};      // �����Ϣ����
u8 RUN_Online_Ack[8]     = {0x40,0x5b,0x53,0x06,0x00,0x05,0x5d,0x25};         // �������з���
u8 RUN_Online_loop_Ack[8]= {0x40,0x5b,0x53,0x07,0x00,0x05,0x5d,0x25};    // ����ѭ�����з���
u8 EEPROM_Download_Ack[8]= {0x40,0x5b,0x53,0x0a,0x00,0x05,0x5d,0x25};     // ���ض����鷴��
u8 EEPROM_Erase_Ack[8]   = {0x40,0x5b,0x53,0x0b,0x00,0x05,0x5d,0x25};       // ���������鷴��
u8 RUN_Offline_Ack[8]    = {0x40,0x5b,0x53,0x0c,0x00,0x05,0x5d,0x25};        // �ѻ����з���
u8 STOP_Offline_Ack[8]   = {0x40,0x5b,0x53,0x0d,0x00,0x05,0x5d,0x25};        // �ѻ�����ֹͣ
u8 Send_Image[9]         = {0x40,0x5b,0x53,0x0e,0x00,0x00,0x06,0x5d,0x25};      // ����ͼ��
u8 Send_Attitude[16]     = {0x40,0x5b,0x53,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0d,0x5d,0x25};   // ������̬
u8 Send_ID[9]            = {0x40,0x5b,0x53,0x11,0x00,0x00,0x06,0x5d,0x25};         // ����ID
//u8 MPU6050_Dat[9]        = {0x40,0x5b,0x53,0x11,0x00,0x00,0x06,0x5d,0x25}




///////////////////////////�������////////////////////////
//ÿ������������һ������������ɣ�ÿ����������һ������������
// ���˳�� ����� 1���ֽ� ���λ�� 2���ֽ� ����ʱ�� 2���ֽ�
// ÿ�������Ϣ5���ֽڣ�һ������32�������Ϣ
u8 servo_num;                    //ÿ�������ж������
u8 servo_group_num[100];        //�������ж�������
u32 servo_position_set[32];     //�������λ��
u32 servo_time_set[32];    //�������ʱ�� ��������ʱ��
u8  servo_num_set[32];        //������
u8 servo_mark=0;              //��������������б�־
u8  servo_run_over[32];        //����ﵽ����λ�ñ�־
u8  servo_run_over_mark=1;     //���ȫ���ﵽָ��λ�ñ�־
u8 servo_control_mark=1; 

void motionGet(u8 flag,u8 *servo_dat){ 
  system_run_mark=1;
  servo_control_mark=1;
  //read_mark=1;
    LED(LED1,LED_ON);
    LED(LED2,LED_OFF);    
    servo_SQ=flag;
    servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
    if(servo_group_num[servo_SQ]!=0){
      ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
    }
  servo_run_over_mark=1;
}

void motionCtr(u8 *servo_dat){
  j=0;
  while(j<servo_group_num[servo_SQ]){
    if(servo_run_over_mark)  //���ж����������λ�ú��ȡ��һ������ֵ
    {
      for(i=0;i<32;i++){
        servo_run_over[i]=0;    
      }
      for(i=0;i<32;i++){
        if((servo_dat[ 160*j + 5*i ]-1)==i){
          servo_num_set[i]=servo_dat[ 160*j + 5*i ];
          servo_position_set[i]=(servo_dat[ 160*j + 5*i + 1]*256 + servo_dat[ 160*j + 5*i + 2 ])/5;
          servo_time_set[i]=(servo_dat[ 160*j + 5*i + 3 ]*256 + servo_dat[ 160*j + 5*i + 4])/100;
        }
      } 
      SERVO_control(servo_num_set,servo_position_set,servo_time_set);
      j++;
     }
    servo_run_over_mark=1;
    SERVO_run_all();
    //������Ƿ�ȫ������ָ��λ��
    for(i=0;i<32;i++){
      servo_run_over_mark=servo_run_over_mark&&servo_run_over[i];    
    }
  } 
}





/////////////////////////���ڽ�������/////////////////////////////////////
u8 uart_Rx_dat[10000];
u32 receive_add;    //���ڽ������ݳ���  
u8 receive_true_mark=0;
u8 uart_receive_mark=0;
void uart_Rx()
{

      if(uart_receive_mark==1)
      {
          uart_receive_mark=0;       
          if(receive_add>=8)
          {
              LED(LED9,LED_ON);
              if(uart_Rx_dat[1]==0x5b && uart_Rx_dat[2]==0x52&& uart_Rx_dat[receive_add-2]==0x5d&& uart_Rx_dat[receive_add-1]==0x25)
              {
                  LED(LED4,LED_ON);
                  switch(uart_Rx_dat[3])
                  {
                      case 0x01:        //��ȡ��ѹ
                      {
                          uart_sendN(UART4, Send_Voltage,9);
    
                      }break;
                      case 0x02:       //��ȡ����   
                      {
                          uart_sendN(UART4, Send_Ampere,9);
       
                      }break;
                      case 0x03:      //�������
                      {
                          
                          servo_num = ( uart_Rx_dat[receive_add-4]*256 + uart_Rx_dat[receive_add-3]-4 )/11;
                          for(i=0;i<(receive_add-8)/11;i++)
                          {
                              servo_dat[5*i]= uart_Rx_dat[11*i+5];
                              servo_dat[5*i+1]= uart_Rx_dat[11*i+7];
                              servo_dat[5*i+2]= uart_Rx_dat[11*i+8];
                              servo_dat[5*i+3]= uart_Rx_dat[11*i+13];
                              servo_dat[5*i+4]= uart_Rx_dat[11*i+14];
                          }
                          ONline_RUN=1;
                          Offline_RUN=0;
 //                         uart_sendN(UART4, Servo_dat_ACK,8);
                          uart_sendN(UART4, servo_dat,servo_num*5);           //�����������Ϣ��������

                      }break;
                      case 0x04:    //�������
                      {
                          
                      }break;
                      case 0x05:    //�������
                      {
                          uart_sendN(UART4, Motor_dat_ACK,8);
                      }break;
                      case 0x06:    //�������ж�����
                      {
                        
                          servo_num = ( uart_Rx_dat[receive_add-4]*256 + uart_Rx_dat[receive_add-3]-4 )/11;
                          for(i=0;i<(receive_add-8)/11;i++)
                          {
                              servo_dat[5*i]= uart_Rx_dat[11*i+5];
                              servo_dat[5*i+1]= uart_Rx_dat[11*i+7];
                              servo_dat[5*i+2]= uart_Rx_dat[11*i+8];
                              servo_dat[5*i+3]= uart_Rx_dat[11*i+13];
                              servo_dat[5*i+4]= uart_Rx_dat[11*i+14];
                          }
                          ONline_RUN=1;
                          Offline_RUN=0;
                          uart_sendN(UART4, RUN_Online_Ack,8);
                      }break;
                      case 0x07:   //����ѭ�����ж�����
                      {
                          servo_num = ( uart_Rx_dat[receive_add-4]*256 + uart_Rx_dat[receive_add-3]-4 )/11;
                          for(i=0;i<(receive_add-8)/11;i++)
                          {
                              servo_dat[5*i]= uart_Rx_dat[11*i+5];
                              servo_dat[5*i+1]= uart_Rx_dat[11*i+7];
                              servo_dat[5*i+2]= uart_Rx_dat[11*i+8];
                              servo_dat[5*i+3]= uart_Rx_dat[11*i+13];
                              servo_dat[5*i+4]= uart_Rx_dat[11*i+14];
                          }
                          ONline_RUN=1;
                          Offline_RUN=0;
                          uart_sendN(UART4, RUN_Online_loop_Ack,8);
                      }break;
                      case 0x08:  //����ֹͣ���ж�����
                      {
                          uart_sendN(UART4, Motor_dat_ACK,8);
                      }break;
                      case 0x09:  //EEPROM��ʼ��
                      {

                      }break;
                      case 0x0a:  //���ض�����
                      {
                          ONline_RUN=0;                          
                          Offline_RUN=0;
                          servo_SQ = uart_Rx_dat[6];
                          servo_group_num[servo_SQ]=(receive_add-11)/(11*32);
                          for(i=0;i<(receive_add-11)/11;i++)
                          {
                              servo_dat[5*i]= uart_Rx_dat[11*i+8];
                              servo_dat[5*i+1]= uart_Rx_dat[11*i+10];
                              servo_dat[5*i+2]= uart_Rx_dat[11*i+11];
                              servo_dat[5*i+3]= uart_Rx_dat[11*i+16];
                              servo_dat[5*i+4]= uart_Rx_dat[11*i+17];
                          }
                          if(servo_SQ<13)
                          {
                              WriteAT24C1024_byte(servo_SQ*5000,servo_group_num[servo_SQ],0x00);
                              DelayMs1(10);
                              WriteAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160);
                          }
                          else
                          {
 
                              WriteAT24C1024_byte((servo_SQ-13)*5000,servo_group_num[servo_SQ],0x02);
                              DelayMs1(10);
                              WriteAT24C1024_flash(servo_dat,(servo_SQ-13)*5000+1,0x02,servo_group_num[servo_SQ]*160);                          
                          }
                      
//                          uart_sendN(UART4, EEPROM_Download_Ack,8);
                          uart_sendN(UART4, servo_dat,servo_group_num[servo_SQ]*160);   //�����������Ϣ��������
                      }break;  
                      case 0x0b:  //����������
                      {

                          uart_sendN(UART4, EEPROM_Erase_Ack,8);
                      }break;
                      case 0x0c:  //�ѻ����ж�����
                      {
                          ONline_RUN=0;                          
                          Offline_RUN=1;
                          servo_SQ = uart_Rx_dat[6];
                          if(servo_SQ<13)
                          {
                              servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
                              ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160);
                          }
                          else
                          {
                              servo_group_num[servo_SQ]=ReadAT24C1024_byte((servo_SQ-13)*5000,0x02);
                              ReadAT24C1024_flash(servo_dat,(servo_SQ-13)*5000+1,0x02,servo_group_num[servo_SQ]*160);                          
                          }
 //                         uart_putchar(UART4,servo_group_num[servo_SQ]);
                          uart_sendN(UART4, servo_dat,servo_group_num[servo_SQ]*160);     //�����������Ϣ��������
 //                         uart_sendN(UART4, RUN_Offline_Ack,8);
                      }break;
                      case 0x0d:  //�ѻ�ֹͣ������
                      {
                          ONline_RUN=1;                          
                          Offline_RUN=0;
                          servo_init_mark=1;     //����ϵ��־����
                          uart_sendN(UART4, STOP_Offline_Ack,8);
                      }break;
                      case 0x0e:   //��ȡͼ����Ϣ
                      {
                          uart_sendN(UART4, Send_Image,9);
                      }break; 
                      case 0x0f:   //��ȡ��̬��Ϣ
                      {

                          uart_sendN(UART4, Send_Attitude,16);
                      }break;
                      case 0x10:   //��ȡѹ����Ϣ
                      {

                      }break;
                      case 0x11:   //��ȡID��Ϣ
                      {
                          uart_sendN(UART4, Send_ID,9);
                      }break;
                      default: break; 
                  }
                  
              }
          }
      }
      else
      {
            LED(LED9,LED_OFF);
            LED(LED4,LED_OFF);

      }
      
}



///////////////////////////����ͨ��////////////////////////
u8 status;	//�����жϽ���/����״̬
u8 txbuf[32]={0x1f,2,3,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};	 //���ͻ���
u8 rxbuf[32];
u8 Tx_Rx_mark; 
u8 NRF_Rx_dat[10000];
u8 NRF_receive_mark=0;
u32 rx_i,NRF_receive_num;
u8 rx_last_num;

void NRF()
{
    if(Tx_Rx_mark==0)
    {
          /*����ģʽ*/
          NRF_TX_Mode();
          
          /*��ʼ��������*/
//          status=NRF_Tx_Dat(txbuf); 
          NRF_Tx_Dat(txbuf);
//          uart_putchar(UART4,status);
          
          /*�жϷ���״̬*/  
          if(status& TX_DS)
          {
                  LED(LED5,LED_ON);
                  LED(LED8,LED_OFF);
//                  Tx_Rx_mark=1;
          }
          else if(status & MAX_RT)
          {
                  LED(LED5,LED_OFF);
                  LED(LED8,LED_ON);
          }
          else
          {
                  LED(LED5,LED_OFF);
                  LED(LED8,LED_ON);
          }
    }
    else if(Tx_Rx_mark==1)
    {
          /*����ģʽ*/
          NRF_RX_Mode();
          /*�ȴ���������*/
//          status = NRF_Rx_Dat(rxbuf);
          /*�жϽ���״̬*/
          switch(status)
          {
              case RX_DR:
              {					
                  LED(LED6,LED_ON);
                  LED(LED7,LED_OFF);
                  if(rxbuf[1]==0x40&&rxbuf[2]==0x5B&&rxbuf[3]==0x52)
                  {
                      rx_i=0;
                  }
                  
                  for(i=2;i<32;i++)
                  {
                      if(rxbuf[i-1]==0x5D&&rxbuf[i]==0x25)
                      {
                          NRF_receive_mark=1;
                          Tx_Rx_mark=0;          //��ȷ���պ��л�������ģʽ
                          rx_last_num=i;
                          
                          break;
                      }
                  }
                  
                  if(NRF_receive_mark==0)
                  {
                      for(i=0;i<31;i++)
                      {
                          NRF_Rx_dat[rx_i*31+i]=rxbuf[i+1];
                      }
                  }
                  else
                  {
                      for(i=0;i<rx_last_num;i++)
                      {
                          NRF_Rx_dat[rx_i*31+i]=rxbuf[i+1];
                      }
                      NRF_receive_num=rx_i*31+rx_last_num;
                  }
                  
                  if(NRF_receive_mark)
                  {
                      switch(NRF_Rx_dat[3])
                      {
                          case 0x01:        //��ȡ��ѹ
                          {
                              txbuf[0]=9;
                              for(i=0;i<9;i++)
                              {
                                  txbuf[i+1]=Send_Voltage[i];
                              }
                              uart_sendN(UART4, Send_Voltage,9);
        
                          }break;
                          case 0x02:       //��ȡ����   
                          {
                              txbuf[0]=9;
                              for(i=0;i<9;i++)
                              {
                                  txbuf[i+1]=Send_Ampere[i];
                              }
                              uart_sendN(UART4, Send_Ampere,9);
           
                          }break;
          
                          case 0x0c:  //�ѻ����ж�����
                          {
                              ONline_RUN=0;                          
                              Offline_RUN=1;
                              servo_SQ = NRF_Rx_dat[6];
                              if(servo_SQ<13)
                              {
                                  servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
                                  ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160);
                              }
                              else
                              {
                                  servo_group_num[servo_SQ]=ReadAT24C1024_byte((servo_SQ-13)*5000,0x02);
                                  ReadAT24C1024_flash(servo_dat,(servo_SQ-13)*5000+1,0x02,servo_group_num[servo_SQ]*160);                          
                              }
                              RUN_Offline_Ack[4]= servo_SQ;
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=RUN_Offline_Ack[i];
                              }
                          }break;
                          case 0x0d:  //�ѻ�ֹͣ������
                          {
                              ONline_RUN=1;                          
                              Offline_RUN=0;
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=STOP_Offline_Ack[i];
                              }
                              uart_sendN(UART4, STOP_Offline_Ack,8);
                          }break;
                          case 0x0e:   //��ȡͼ����Ϣ
                          {
                              txbuf[0]=9;
                              for(i=0;i<9;i++)
                              {
                                  txbuf[i+1]=Send_Image[i];
                              }
                              uart_sendN(UART4, Send_Image,9);
                          }break; 
                          case 0x0f:   //��ȡ��̬��Ϣ
                          {
                              
                              txbuf[0]=16;
                              for(i=0;i<16;i++)
                              {
                                  txbuf[i+1]=Send_Attitude[i];
                              }
                              uart_sendN(UART4, Send_Attitude,16);
                          }break;
                          case 0x10:   //��ȡѹ����Ϣ
                          {
      
                          }break;
                          case 0x11:   //��ȡID��Ϣ
                          {
                              txbuf[0]=9;
                              for(i=0;i<9;i++)
                              {
                                  txbuf[i+1]=Send_ID[i];
                              }
                              uart_sendN(UART4, Send_ID,9);
                          }break;

                          case 0x12:   //��ȡ��̬��Ϣ
                          {

                              txbuf[0]=16;
                              for(i=0;i<16;i++)
                              {
                                  txbuf[i+1]=Send_Attitude[i];
                              }
                              uart_sendN(UART4, Send_Attitude,16);
                          }break;

                          default: break; 
                      }
                  
                  }

                  rx_i++;
                  
              }
              break;

              default:
              {
                  LED(LED6,LED_OFF);
                  LED(LED7,LED_ON);
              }
              break;  	
          }
          

    
    }
}



//////////////////////////////////////////////////////////


void system_init()
{
    uart_init(UART4, 115200);                    //��ʼ������4��������Ϊ19200 ,������̫�����ײ��ȶ�
    DisableInterrupts;          //�����ж�
    uart_irq_EN(UART4);
    EnableInterrupts;           //�����ж� 
    
    LED_init();
    Servo_Init();
    NRF_Init();
    AT24C1024_Init();
    //MPU6050_Init();
    
    
    pit_init_ms(PIT1, 1);                          //��ʼ��PIT1    
    
    //Tx_Rx_mark=1;
    //Offline_RUN=1; 
    //ONline_RUN=0;
    //system_run_mark=1;
    
    exti_init(PORTC, 18, rising_down);      //PORTC18 �˿��ⲿ�жϳ�ʼ�� �������ش����жϣ��ڲ�����
    exti_init(PORTC, 8, falling_down);       //PORTC8 �˿��ⲿ�жϳ�ʼ�� ���͵�ƽ�����жϣ��ڲ�����
    exti_init(PORTE, 4, falling_down);      //PORTE4 �˿��ⲿ�жϳ�ʼ�� ���͵�ƽ�����жϣ��ڲ�����
    //exti_init(PORTE, 5, falling_down);      //PORTE5 �˿��ⲿ�жϳ�ʼ�� ���͵�ƽ�����жϣ��ڲ�����//
    exti_init(PORTE, 6, falling_down);      //PORTE6 �˿��ⲿ�жϳ�ʼ�� ���͵�ƽ�����жϣ��ڲ�����
        
}











/*************************************************************************
*
*  �������ƣ�PIT1_IRQHandler
*  ����˵����PIT1 ��ʱ�жϷ�����

*************************************************************************/
void PIT1_IRQHandler(void)
{
   

    
    PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}



/*************************************************************************
*
*  �������ƣ�PORTn_IRQHandler
*  ����˵����PORTn�˿��жϷ�����
*  ����˵������
*  �������أ���
*  ��    ע�����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
u8 read_mark=0;   //��ȡ�������־
u8 read_num=0;
u8 hw_read_mark=0;
u8 hw_read=0;  

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

void PORTE_IRQHandler()
{

    if(PORTE_ISFR & (1 << 4) || PORTE_ISFR & (1 << 6))           //PTE4 5 6�����ж�
    {
        read_mark = 1;
        PORTE_ISFR  |= (1 << 4);        //д1���жϱ�־λ      //д1���жϱ�־λ
        PORTE_ISFR  |= (1 << 6);        //д1���жϱ�־λ
        /*  ����Ϊ�û�����  */
        //LED(LED3,LED_ON);
        //system_run_mark=0;
        //servo_control_mark=0;
        hw_read_mark=1;
        /*  ����Ϊ�û�����  */
       
    }

}
