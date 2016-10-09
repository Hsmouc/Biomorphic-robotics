#include "default.h"
#include "iic2.h"
#include "gpio.h"

unsigned char ack2;

void DelayUs2(unsigned int us)
{
  int ii,jj;
  if (us<1) us=1;
  for(ii=0;ii<us;ii++)
    for(jj=0;jj<13;jj++);   //50MHz--1us
}

void DelayMs2(unsigned int ms)
{
  int ii,jj;
  if (ms<1) ms=1;
  for(ii=0;ii<ms;ii++)
    for(jj=0;jj<8800;jj++);   //50MHz--1ms
}

void get_ms2(unsigned long *time)
{

}
/**
* @fn:         IIC_Init
* @bref:       IIC port initialization
* @params:     none    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/
void IIC2_Init(void)
{
  gpio_init(PORTB,9,GPO,1);
  gpio_init(PORTB,10,GPO,1);
  SDA_OUT2();
  SCL_OUT2();
}
/**
* @fn:         IIC_Start
* @bref:       generate IIC start signal
* @params:     none    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/
void IIC2_Start(void)
{
  SDA_OUT2();
  IIC_SDA2(1);
  IIC_DelayUs(1);
  IIC_SCL2(1);
  IIC_DelayUs(5);
  IIC_SDA2(0);
  IIC_DelayUs(5);
  IIC_SCL2(0);
  IIC_DelayUs(2);
}
/**
* @fn:         IIC_Stop
* @bref:       generate IIC stop signal
* @params:     none    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/
void IIC2_Stop(void)
{
  SDA_OUT2();
  IIC_SDA2(0);
  IIC_DelayUs(1);
  IIC_SCL2(1);
  IIC_DelayUs(5);
  IIC_SDA2(1);
  IIC_DelayUs(4);							   	
}
/**
* @fn:         IIC_NAck
* @bref:       IIC No ACK action
* @params:     none    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/
//void IIC_NAck(void)
//{
//  IIC_SCL(0);
//  SDA_OUT();
//  IIC_SDA(1);
//  IIC_DelayUs(1);
//  IIC_SCL(1);
//  IIC_DelayUs(1);
//  IIC_SCL(0);  
//}
///**
//* @fn:         IIC_NAck
//* @bref:       IIC ACK action
//* @params:     none    
//* @return:     none
//* @version:    0.1
//* @date:       2013-1-5 
//* 
//*/
//void IIC_Ack(void)
//{
//  IIC_SCL(0);
//  SDA_OUT();
//  IIC_SDA(0);
//  IIC_DelayUs(2);
//  IIC_SCL(1);
//  IIC_DelayUs(2);
//  IIC_SCL(0);
//}

void IIC2_SendACK(unsigned char a)  
{  
  SDA_OUT2();  
  if(a==0) IIC_SDA2(0);           /*在此发出应答或非应答信号 */
       else IIC_SDA2(1);
  IIC_DelayUs(3);      
  IIC_SCL2(1);
  
  IIC_DelayUs(5);                     /*时钟低电平周期大于4μs*/
  
  IIC_SCL2(0);                      /*清时钟线，钳住I2C总线以便继续接收*/
  IIC_DelayUs(2);    
}  
//
//unsigned char IIC_RecvACK(void)
//{
//  SDA_IN();
//  SCL_OUT();
//  IIC_SCL(1);
//  IIC_DelayUs(1);
//  if(READ_SDA())
//  {
//    IIC_SCL(0);
//    IIC_DelayUs(1);
//    IIC_SCL(1);
//    if(READ_SDA()) return 1;
//  }
//  else return 0;
//}



/**
* @fn:         IIC_SendByte
* @bref:       IIC sned one byte
* @params:     byte to send    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/

void IIC2_SendByte(unsigned char dat)
{
  unsigned char BitCnt;

  SDA_OUT2();  
  for(BitCnt=0;BitCnt<8;BitCnt++)   /*要传送的数据长度为8位*/
  {
    if((dat<<BitCnt)&0x80) IIC_SDA2(1);   /*判断发送位*/
      else IIC_SDA2(0);                
    IIC_DelayUs(1);
    IIC_SCL2(1);              /*置时钟线为高，通知被控器开始接收数据位*/
    
    IIC_DelayUs(5);              /*保证时钟高电平周期大于4μs*/
           
    IIC_SCL2(0); 
  }
  
  IIC_DelayUs(2);
  SDA_IN2();                 /*8位发送完后释放数据线，准备接收应答位*/
  IIC_DelayUs(2);  
  IIC_SCL2(1);
  IIC_DelayUs(3);
  if(READ_SDA2())ack2=0;     
    else ack2=1;         /*判断是否接收到应答信号*/
  IIC_SCL2(0);
  IIC_DelayUs(2); 
}
/**
* @fn:         IIC_ReadByte
* @bref:       IIC read one byte
* @params:     byte to read    
* @return:     none
* @version:    0.1
* @date:       2013-1-5 
* 
*/
unsigned char IIC2_ReadByte(void)
{
   unsigned char retc;
   unsigned char BitCnt;
  
   retc=0; 
   SDA_IN2();               /*置数据线为输入方式*/
   for(BitCnt=0;BitCnt<8;BitCnt++)
   {
     IIC_DelayUs(1);   
     IIC_SCL2(0);                  /*置时钟线为低，准备接收数据位*/
    
     IIC_DelayUs(5);                  /*时钟低电平周期大于4.7μs*/
   
     IIC_SCL2(1);                  /*置时钟线为高使数据线上数据有效*/
     IIC_DelayUs(3);
     retc=retc<<1;
     if(READ_SDA2()) retc=retc+1;   /*读数据位,接收的数据位放入retc中 */
     IIC_DelayUs(2); 
   }
   IIC_SCL2(0);   
   IIC_DelayUs(2);
   return(retc);
}

char i2c2WriteBuffer(unsigned char addr, unsigned char reg, unsigned char len, unsigned char * data)
{  
    unsigned char i;

    IIC2_Start();                /*启动总线*/
    IIC2_SendByte(addr<<1 + 0);              /*发送器件地址*/
    if(ack2==0)return(1);
    IIC2_SendByte(reg);             /*发送器件子地址*/
    if(ack2==0)return(1);
    for(i=0;i<len;i++)
    {   
      IIC2_SendByte(*data);             /*发送数据*/
      if(ack2==0)return(1);
      data++;
    } 
    IIC2_Stop();                 /*结束总线*/ 

    return(0);
}  

char i2c2Read(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf)
{  
    unsigned char i;

    IIC2_Start();                 /*启动总线*/
    IIC2_SendByte((addr<<1) + 0);                 /*发送器件地址*/
    if(ack2==0)return(1);
    IIC2_SendByte(reg);                /*发送器件子地址*/
    if(ack2==0)return(1);
    IIC2_Start();     /*重新启动总线*/
    IIC2_SendByte((addr<<1) + 1);
    if(ack2==0)return(1);
    for(i=0;i<len-1;i++)
    {   
      *buf=IIC2_ReadByte();                /*发送数据*/
      IIC2_SendACK(0);                 /*发送就答位*/  
      buf++;
    } 
    *buf=IIC2_ReadByte();
    IIC2_SendACK(1);                    /*发送非应位*/
    IIC2_Stop();                     /*结束总线*/ 
    return(0);
}  


unsigned char HMC5883_RecvByte2(void)
{
  unsigned char i,receive=0;
  SDA_IN2();
  for(i=0;i<8;i++ )
  {
    IIC_SCL2(0); 
    DelayUs(2);
    IIC_SCL2(1);
    receive<<=1;
    if(READ_SDA2())
      receive++;   
    DelayUs(1); 
  }
  return receive;
}
void HMC5883_SendByte2(unsigned char dat)
{
  unsigned char i;
  SDA_OUT2();
  for(i=0;i<8;i++)
  {
    if(dat&0x80) 
      IIC_SDA2(1);
    else 
      IIC_SDA2(0);
    dat<<=1;
    DelayUs(5);
    IIC_SCL2(1);
    DelayUs(5);
    IIC_SCL2(0);
  }
  IIC2_RecvACK();
}
