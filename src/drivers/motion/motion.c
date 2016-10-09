#include "common.h"
#include "include.h"
///////////////////////////�������////////////////////////
//ÿ������������һ������������ɣ�ÿ����������һ������������
u8 servo_SQ;  //��������
// ���˳�� ����� 1���ֽ� ���λ�� 2���ֽ� ����ʱ�� 2���ֽ�
// ÿ�������Ϣ5���ֽڣ�һ������32�������Ϣ
u8 servo_dat[5000]; 
u8 servo_num;                    //ÿ�������ж������
u8 servo_group_num[100];        //�������ж�������
u32 servo_position_set[32];     //�������λ��
u32 servo_time_set[32];    //�������ʱ�� ��������ʱ��
u8  servo_num_set[32];        //������
u8 servo_mark=0;              //��������������б�־
u8  servo_run_over_mark=1;     //���ȫ���ﵽָ��λ�ñ�־
 
void servo_data_tranfer(void) {
    u32 i = 0;
    u32 j = 0;
    for(i=0;i<32;i++) {                       
        if((servo_dat[ 160*j + 5*i ]-1)==i) {
            servo_num_set[i]=servo_dat[ 160*j + 5*i ];
            servo_position_set[i]=(servo_dat[ 160*j + 5*i + 1]*256 + servo_dat[ 160*j + 5*i + 2 ])/5;
            servo_time_set[i]=(servo_dat[ 160*j + 5*i + 3 ]*256 + servo_dat[ 160*j + 5*i + 4])/100;
        }             
   } 
}

void motionCtr(int flag) {        
      if(flag == 1) {        
            LED(LED1,LED_ON);
            LED(LED2,LED_OFF);    
            servo_SQ=0;
            servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
            if(servo_group_num[servo_SQ]!=0)
            {   
                ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160);
            }
        }   
        else if(flag == 2) {        
            LED(LED1,LED_OFF);
            LED(LED2,LED_ON);    
            servo_SQ=1;
            servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
            if(servo_group_num[servo_SQ]!=0)
            {   
                ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
            }
   
        }
        else if(flag == 3) {        
            LED(LED1,LED_OFF);
            LED(LED2,LED_ON);    
            servo_SQ=2;
            servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
            if(servo_group_num[servo_SQ]!=0)
            { 
                ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
            }
   
        }
         servo_data_tranfer();
         SERVO_control(servo_num_set,servo_position_set,servo_time_set);
}
