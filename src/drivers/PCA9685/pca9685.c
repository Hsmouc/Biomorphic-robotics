#include "include.h"
unsigned char AutoWritePCA9685_TAB[71]={ 0x80,0x00,0x20,0x1c,0x66,0x44,0x22,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                                  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/*/ Ҫ���͵�PCA9685�����ݼĴ�����ʼ����pwmռ�ձ� */
unsigned int kuozhan_pwm_TAB[16]={ 0x80,0x10,0x30,0x50,0x70,0x90,0x22};

/*************************************************************/
/*            дPCA9685�����Ĵ���                            */
/*        ��������byte�β�PCA9685��ַbyte,                   */
/*        Ҫд��Ĵ����ĵ�ַbyte,                            */
/*        Ҫд��Ĵ�������ֵbyte,                            */
/*************************************************************/
u8 WritePCA9685REG_1(u8 address,u8 REG,u8 value)
{
  IIC_Start();//�������߿�ʼ��������
  
  IIC_SendByte(address);
  IIC_SendByte(REG);
  IIC_SendByte(value);
  IIC_Stop();
  
  return 1;

}
/*************************************************************/
/*            �Զ�дPCA9685���мĴ����Զ�����                */                    
/*            Ҫд��Ĵ�������ֵ��,                          */
/*************************************************************/

u8 AutoWritePCA9685(u8 tap[])
{
     unsigned char *p=0;
     u8 i=0;
     unsigned char temp=0;
     IIC_Start();//�������߿�ʼ��������
     for (;i<=70;i++) {
     temp=tap[i];     
     IIC_SendByte(temp) ;//����Ҫ�������ݵ�������ַ
   
     p++;
     }
     IIC_Stop();
     return 0;
 }


/*************************************************************/
/*     Ҫд��PCA9685��pwmֵ����ת��uint to uchar 16ת12λ    */
/*        ��������void�β�PCA9685��ַbyte,                   */
/*        Ҫд��Ĵ����ĵ�ַbyte,                            */
/*        Ҫд��Ĵ�������ֵbyte,                            */
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