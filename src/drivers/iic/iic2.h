#ifndef __IIC2_H__
#define __IIC2_H__


void DelayUs2(unsigned int us);
void DelayMs2(unsigned int ms);
void get_ms2(unsigned long *time);
void IIC2_Init(void);
void IIC2_Start(void);
void IIC2_Stop(void);
//void IIC_NAck(void);
//void IIC_Ack(void);
unsigned char IIC2_RecvACK(void);
void IIC2_SendByte(unsigned char dat);
unsigned char IIC2_ReadByte(void);
char i2c2WriteBuffer(unsigned char addr, unsigned char reg, unsigned char len, unsigned char * data);
char i2c2Read(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
unsigned char HMC5883_RecvByte2(void);
void HMC5883_SendByte2(unsigned char dat);


#endif