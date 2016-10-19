/********************  ********************
 * �ļ���       ��AT24C02.c
 * ����         ��AT24C02оƬ������������
 *
**********************************************************************************/
#include "include.h"
#include "default.h"
#include "AT24C02.h"

u8 ack1=0;


void DelayUs1(unsigned int us)
{
  int ii,jj;
  if (us<1) us=1;
  for(ii=0;ii<us;ii++)
    for(jj=0;jj<13;jj++);   //50MHz--1us
}

void DelayMs1(unsigned int ms)
{
  int ii,jj;
  if (ms<1) ms=1;
  for(ii=0;ii<ms;ii++)
    for(jj=0;jj<8800;jj++);   //50MHz--1ms
}



void IIC1_Init(void)
{
  gpio_init(PORTB,0, GPO,1);
  gpio_init(PORTB,1, GPO,1);
  SDA_OUT1();
  SCL_OUT1();
}

void IIC1_Start(void)
{
  SDA_OUT1();
  IIC_SDA1(1);
  DelayUs1(5);
  IIC_SCL1(1);
  DelayUs1(5);
  IIC_SDA1(0);
  DelayUs1(5);
//  IIC_SCL1(0);
//  DelayUs1(2);
}

void IIC1_Stop(void)
{
  SDA_OUT1();
  IIC_SDA1(0);
  DelayUs1(5);
  IIC_SCL1(1);
  DelayUs1(5);
  IIC_SDA1(1);
  DelayUs1(5);							   	
}

void IIC_Ack(void)
{
  u32 a=0;

  SDA_IN1();                 /*8λ��������ͷ������ߣ�׼������Ӧ��λ*/

  IIC_SCL1(1);
  DelayUs1(5);
  while((READ_SDA1()==1)&&(a<1000))a++;
  IIC_SCL1(0);
  DelayUs1(5); 
 
}


void IIC1_SendByte(unsigned char dat)
{
  unsigned char BitCnt;

  SDA_OUT1();  
  for(BitCnt=0;BitCnt<8;BitCnt++)   //Ҫ���͵����ݳ���Ϊ8λ//
  {
    IIC_SCL1(0); 
    DelayUs1(5);
    if((dat<<BitCnt)&0x80) IIC_SDA1(1);   //�жϷ���λ//
      else IIC_SDA1(0);                
    DelayUs1(5);
    IIC_SCL1(1);              //��ʱ����Ϊ�ߣ�֪ͨ��������ʼ��������λ//  
    DelayUs1(5);              //��֤ʱ�Ӹߵ�ƽ���ڴ���4��s//
           
  }
  IIC_SCL1(0);
  DelayUs1(5);
  IIC_SDA1(1);
  DelayUs1(5);
}


unsigned char IIC1_ReadByte(void)
{
   unsigned char retc;
   unsigned char BitCnt;
  
   retc=0; 
  
   SDA_OUT1();  
   IIC_SCL1(0); 
   DelayUs1(5);  
   IIC_SDA1(1);
   DelayUs1(5); 
   
   SDA_IN1();              //��������Ϊ���뷽ʽ//
   
   for(BitCnt=0;BitCnt<8;BitCnt++)
   {
     IIC_SCL1(1);                  //��ʱ����Ϊ�ͣ�׼����������λ//
    
     DelayUs1(5);                  //ʱ�ӵ͵�ƽ���ڴ���4.7��s//
    
     retc=retc<<1;
     if(READ_SDA1()) retc=retc+1;   //������λ,���յ�����λ����retc�� //
     IIC_SCL1(0);                  //��ʱ����Ϊ��ʹ��������������Ч//
     DelayUs1(5); 
   }
   return(retc);
}



void AT24C02_Init(void)
{
  IIC1_Init();
  IIC_SDA1(1);
  DelayUs1(5);
  IIC_SCL1(1);
  DelayUs1(5);   
}


/*************************************************************/
/*            дAT24C02                            */
/*        ��������byte�β�AT24C02��ַbyte,                   */
/*        Ҫд��Ĵ����ĵ�ַbyte,                            */
/*        Ҫд��Ĵ�������ֵbyte,                            */
/*************************************************************/
void WriteAT24C02_byte(u32 address,u8 date)
{
  IIC1_Start();//�������߿�ʼ��������
  IIC1_SendByte(0xa0);  //1010 0000  1010 a2 a1a 0 R/W  W=0 R=1
  IIC_Ack();
  IIC1_SendByte(address);
  IIC_Ack();
  IIC1_SendByte(date);
  IIC_Ack();
  IIC1_Stop();

}

void WriteAT24C02_flash(u8 *nContent,u32 address,u32 nLen)
{
   u32 ii;
//   u32 addr;
//   addr=address;
/*
    IIC1_Start();
    IIC1_SendByte(0xa0); 
    IIC_Ack();
    IIC1_SendByte(address);
    IIC_Ack();
   
   
   for(ii=0;ii<nLen;ii++)
   {

      temp=nContent[ii];
      IIC1_SendByte(temp);
      IIC_Ack();


   }
   IIC1_Stop();
*/
    for(ii=0;ii<nLen;ii++)
    {
        WriteAT24C02_byte(address+ii,nContent[ii]);
        DelayMs1(2);
    }
   
}



u8 ReadAT24C02_byte(u32 address)
{
  u8 date;
  IIC1_Start();//�������߿�ʼ��������
  IIC1_SendByte(0xa0);  //1010 0000  1010 a2 a1a 0 R/W  W=0 R=1
  IIC_Ack();
  IIC1_SendByte(address);
  IIC_Ack();
  IIC1_Start();
  IIC1_SendByte(0xa1);
  IIC_Ack();
  date=IIC1_ReadByte();
  
  IIC1_Stop();
  
  return date;

}

void ReadAT24C02_flash(u8 *nContent,u32 address,u32 nLen)
{
/*
  IIC1_Start();
  IIC1_SendByte(0xa0); 
  IIC_Ack();
  IIC1_SendByte(address);
  IIC_Ack();
  IIC1_Start();
  IIC1_SendByte(0xa1);
  IIC_Ack();
  while(--nLen)
  {
      *nContent=IIC1_ReadByte();
      nContent++;
      
  }
  *nContent=IIC1_ReadByte();;
  
  IIC1_Stop();
  
*/
    u32 ii;
    for(ii=0;ii<nLen;ii++)
    {
        nContent[ii]=ReadAT24C02_byte(address+ii);
    }
  
  
}



