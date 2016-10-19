/********************  ********************
 * 文件名       ：AT24C1024.h
 * 描述         ：AT24C1024芯片操作函数定义

**********************************************************************************/



#ifndef _AT24C1024_H_
#define _AT24C1024_H_

#define Device_Add  0xa0
#define EEprom_W    0x00
#define EEprom_R    0x01


extern void DelayUs1(unsigned int us);
extern void DelayMs1(unsigned int ms);
extern void IIC1_Init(void);
extern void IIC1_Start(void);
extern void IIC1_Stop(void);
extern void IIC_Ack(void);
extern void IIC1_SendACK(unsigned char a);  
extern void IIC1_SendByte(unsigned char dat);
extern unsigned char IIC1_ReadByte(void);
extern void AT24C1024_Init(void);
extern void WriteAT24C1024_byte(u16 address,u8 date,u8 EEprom_P0);
extern void WriteAT24C1024_flash(u8 *nContent,u16 address,u8 EEprom_P0,u32 nLen);
extern u8 ReadAT24C1024_byte(u16 address,u8 EEprom_P0);
extern void ReadAT24C1024_flash(u8 *nContent,u16 address,u8 EEprom_P0,u32 nLen);


#endif  //_AT24C1024_H_