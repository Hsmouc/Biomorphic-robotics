#include "MPU6050.h"
#include "default.h"
//#include "iic.h"

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

void IIC2_Init(void)
{
  gpio_init(PORTB,9,GPO,1);
  gpio_init(PORTB,10,GPO,1);
  SDA_OUT2();
  SCL_OUT2();
}

void IIC2_Start(void)
{
  SDA_OUT2();
  IIC_SDA2(1);
  DelayUs2(5);
  IIC_SCL2(1);
  DelayUs2(5);
  IIC_SDA2(0);
  DelayUs1(5);
//  IIC_SCL1(0);
//  DelayUs1(2);
}

void IIC2_Stop(void)
{
  SDA_OUT2();
  IIC_SDA2(0);
  DelayUs2(5);
  IIC_SCL2(1);
  DelayUs2(5);
  IIC_SDA2(1);
  DelayUs2(5);							   	
}

void IIC2_Ack(void)
{
  u32 a=0;

  SDA_IN2();                 /*8位发送完后释放数据线，准备接收应答位*/

  IIC_SCL2(1);
  DelayUs2(5);
  while((READ_SDA2()==1)&&(a<1000))a++;
  IIC_SCL2(0);
  DelayUs2(5); 
 
}


void IIC2_SendByte(unsigned char dat)
{
  unsigned char BitCnt;

  SDA_OUT2();  
  for(BitCnt=0;BitCnt<8;BitCnt++)   //要传送的数据长度为8位//
  {
    IIC_SCL2(0); 
    DelayUs2(5);
    if((dat<<BitCnt)&0x80) IIC_SDA2(1);   //判断发送位//
      else IIC_SDA2(0);                
    DelayUs2(5);
    IIC_SCL2(1);              //置时钟线为高，通知被控器开始接收数据位//  
    DelayUs2(5);              //保证时钟高电平周期大于4μs//
           
  }
  IIC_SCL2(0);
  DelayUs2(5);
  IIC_SDA2(1);
  DelayUs2(5);
}


unsigned char IIC2_ReadByte(void)
{
   unsigned char retc;
   unsigned char BitCnt;
  
   retc=0; 
  
   SDA_OUT2();  
   IIC_SCL2(0); 
   DelayUs2(5);  
   IIC_SDA2(1);
   DelayUs2(5); 
   
   SDA_IN2();              //置数据线为输入方式//
   
   for(BitCnt=0;BitCnt<8;BitCnt++)
   {
     IIC_SCL2(1);                  //置时钟线为低，准备接收数据位//
    
     DelayUs2(5);                  //时钟低电平周期大于4.7μs//
    
     retc=retc<<1;
     if(READ_SDA2()) retc=retc+1;   //读数据位,接收的数据位放入retc中 //
     IIC_SCL2(0);                  //置时钟线为高使数据线上数据有效//
     DelayUs2(5); 
   }
   return(retc);
}



unsigned char MPU6050_ReadByte(unsigned char address)
{
  unsigned char ret = 100;
  IIC2_Start();
  IIC2_SendByte(0xd0);	
  IIC2_Ack();
  IIC2_SendByte(address);
  IIC2_Ack();
  IIC2_Start();	
  IIC2_SendByte(0xd1);
  IIC2_Ack();
  ret = IIC2_ReadByte();
  IIC2_Stop();
  return ret;
}

void MPU6050_WriteByte(unsigned char address,unsigned char thedata)
{
  IIC2_Start();
  IIC2_SendByte(0Xd0);
  IIC2_Ack();
  IIC2_SendByte(address);
  IIC2_Ack();
  IIC2_SendByte(thedata);
  IIC2_Ack();
  IIC2_Stop();
}
void MPU6050_Init(void)
{	
  IIC2_Init();
  DelayMs2(100);
  //解除休眠
  MPU6050_WriteByte(PWR_MGMT_1 , 0x00 );
  DelayMs2(10);
  MPU6050_WriteByte(SMPLRT_DIV , 0x07 );   //采样频率  1KHZ
  DelayMs2(10);
  MPU6050_WriteByte(CONFIG0 , 0x07 );      //低通滤波器设置
  DelayMs2(10);
  MPU6050_WriteByte(AUX_VDDIO,0x80);      //
  DelayMs2(10);
  MPU6050_WriteByte(GYRO_CONFIG , 0x18 );  //陀螺仪自检及测量范围 典型值0x18(不自检 正负2000deg/s)
  DelayMs2(10);
  MPU6050_WriteByte(ACCEL_CONFIG , 0x00 );  //配置加速度传感器工作在正负2g，0-2g  1-4g   2-8g   3-16g
  DelayMs2(10);
  MPU6050_WriteByte(I2C_MST_CTRL,0x00);
  DelayMs2(10);
  MPU6050_WriteByte(INT_PIN_CFG,0x02);
  //  /**************HMC5883寄存器配置通过6050输出******************/
  //  HMC_SingleWrite(HMC_CFG1,0x78);
  //  HMC_SingleWrite(HMC_CFG2,0x00);
  //  HMC_SingleWrite(HMC_MOD,0x00);
  DelayMs2(100);	
}
unsigned short MPU6050_GetData(unsigned char REG_Address)
{
  unsigned char H,L;
  H = MPU6050_ReadByte(REG_Address);
  L = MPU6050_ReadByte(REG_Address +1);
  return ((H << 8) +L);  
}


unsigned short mpu6050_getdata(unsigned char moudle,unsigned char axis)
{
  if( 0x68 != MPU6050_ReadByte(WHO_AM_I))
    return 1;		//硬件错误或连接失败
  if('G' == moudle)
  {
    if('X' == axis)
    {
      return MPU6050_GetData(GYRO_XOUT_H);
    }
    if('Y' == axis)
    {
      return MPU6050_GetData(GYRO_YOUT_H);
    }
    if('Z' == axis)
    {
      return MPU6050_GetData(GYRO_ZOUT_H);
    }
  }
  if('A' == moudle)
  {
    if('X' == axis)
    {
      return MPU6050_GetData(ACCEL_XOUT_H);
    }
    if('Y' == axis)
    {
      return MPU6050_GetData(ACCEL_YOUT_H);
    }
    if('Z' == axis)
    {
      return MPU6050_GetData(ACCEL_ZOUT_H);
    }
  }
  return 2;	//	输入参数配置错误
}

void HMC_SingleWrite(unsigned char regAddress,unsigned char data)
{
  IIC2_Start();
  IIC2_SendByte(SLAVE_WRITE_ADDRESS);
  IIC2_SendByte(regAddress);
  IIC2_SendByte(data);
  IIC2_Stop();
}
//**************************************
//从HMC5883L-I2C设备寄存器读取一个字节数据
//**************************************

unsigned char HMC_SingleRead(unsigned char regAddress)
{
  unsigned char data;
  IIC2_Start();
  IIC2_SendByte(SLAVE_WRITE_ADDRESS);
  IIC2_SendByte(regAddress);
  IIC2_Start();
  IIC2_SendByte(SLAVE_READ_ADDRESS);
  data = IIC2_ReadByte();
  IIC2_Stop();
  return data;
}
