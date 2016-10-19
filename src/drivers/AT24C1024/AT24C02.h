/********************  ********************
 * 文件名       ：AT24C02.h
 * 描述         ：AT24C02芯片操作函数定义

**********************************************************************************/



#ifndef _AT24C02_H_
#define _AT24C02_H_


extern void DelayUs1(unsigned int us);
extern void DelayMs1(unsigned int ms);
extern void IIC1_Init(void);
extern void IIC1_Start(void);
extern void IIC1_Stop(void);
extern void IIC_Ack(void);
extern void IIC1_SendACK(unsigned char a);  
extern void IIC1_SendByte(unsigned char dat);
extern unsigned char IIC1_ReadByte(void);
extern void AT24C02_Init(void);
extern void WriteAT24C02_byte(u32 address,u8 date);
extern void WriteAT24C02_flash(u8 *nContent,u32 address,u32 nLen);
extern u8 ReadAT24C02_byte(u32 address);
extern void ReadAT24C02_flash(u8 *nContent,u32 address,u32 nLen);


#endif  //_AT24C02_H_