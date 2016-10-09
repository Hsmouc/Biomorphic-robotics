#include "include.h"
unsigned char AutoWritePCA9685_TAB[71]={ 0x80,0x00,0x20,0x1c,0x66,0x44,0x22,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                                  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/*/ 要发送到PCA9685的数据寄存器初始化和pwm占空比 */
unsigned int kuozhan_pwm_TAB[16]={ 0x80,0x10,0x30,0x50,0x70,0x90,0x22};

/*************************************************************/
/*            写PCA9685单个寄存器                            */
/*        函数类型byte形参PCA9685地址byte,                   */
/*        要写入寄存器的地址byte,                            */
/*        要写入寄存器的数值byte,                            */
/*************************************************************/
u8 WritePCA9685REG_1(u8 address,u8 REG,u8 value)
{
  IIC_Start();//启动总线开始发送数据
  
  IIC_SendByte(address);
  IIC_SendByte(REG);
  IIC_SendByte(value);
  IIC_Stop();
  
  return 1;

}
/*************************************************************/
/*            自动写PCA9685所有寄存器自动增量                */                    
/*            要写入寄存器的数值表,                          */
/*************************************************************/

u8 AutoWritePCA9685(u8 tap[])
{
     unsigned char *p=0;
     u8 i=0;
     unsigned char temp=0;
     IIC_Start();//启动总线开始发送数据
     for (;i<=70;i++) {
     temp=tap[i];     
     IIC_SendByte(temp) ;//发送要接受数据的器件地址
   
     p++;
     }
     IIC_Stop();
     return 0;
 }


/*************************************************************/
/*     要写入PCA9685的pwm值数据转换uint to uchar 16转12位    */
/*        函数类型void形参PCA9685地址byte,                   */
/*        要写入寄存器的地址byte,                            */
/*        要写入寄存器的数值byte,                            */
/*************************************************************/
void PCA9685PWM_convert(void)
{
     unsigned char i=10,ii,a;
     unsigned char pwm[2];
     unsigned int aa,bb;
     float d;
     for (ii=0;ii<=15;ii++){
      
     aa=kuozhan_pwm_TAB[ii];
     d=(float)(aa)/2.4;
     bb=(unsigned int)(d);
     pwm[0]=bb&0X00ff;
     bb=bb>>8;
     pwm[1]=bb&0x00ff;
     for (a=0;a<=1;a++){
     AutoWritePCA9685_TAB[i]=pwm[a];
     i++;}
     i+=2; 
     }
     } 