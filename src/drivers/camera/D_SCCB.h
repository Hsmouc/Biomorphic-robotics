/*! *
 * @file       FIRE_SCCB.h
 * @brief      OV摄像头配置总线SCCB函数库

 */


#ifndef _D_SCCB_H_
#define _D_SCCB_H_

	 


#define SCL_H()         IIC_SCL0(1)
#define SCL_L()         IIC_SCL0(0)
#define SCL_DDR_OUT()   SCL_OUT0()
//#define SCL_DDR_IN()    

#define SDA_H()         IIC_SDA0(1)
#define SDA_L()         IIC_SDA0(0)
#define SDA_IN_DAT()    READ_SDA0()
#define SDA_DDR_OUT()   SDA_OUT0()
#define SDA_DDR_IN()    SDA_IN0()

#define ADDR_OV7725   0x42
#define ADDR_OV7620   0x42

#define DEV_ADR  ADDR_OV7725             /*设备地址定义*/

#define SCCB_DELAY()    SCCB_delay(400)


extern void SCCB_GPIO_init(void);
extern int SCCB_WriteByte( uint16 WriteAddress , uint8 SendByte);
extern int SCCB_ReadByte(uint8 *pBuffer,   uint16 length,   uint8 ReadAddress);


#endif      //_D_SCCB_H_
