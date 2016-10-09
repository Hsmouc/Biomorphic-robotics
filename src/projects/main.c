/******************** ********************
 * �ļ���       ��main.c
 * ����         ��
 *
**********************************************************************************/
#include "common.h"
#include "include.h"
#include "math.h"

//extern u32 servo_position_set[32];     //�������λ��
//extern u32 servo_time_set[32];    //�������ʱ�� ��������ʱ��
//extern u8  servo_num_set[32];        //������
u8 image_bin[CAMERA_SIZE]; 
u8 img[CAMERA_W * CAMERA_H];

void sendimg(void *imgaddr, uint32_t imgsize)
{
    uart_putchar(UART4, 0x00);                          //�����ĸ��ֽ�����
    uart_putchar(UART4, 0xff);
    uart_putchar(UART4, 0x01);
    uart_putchar(UART4, 0x01);
    uart_sendN(UART4, (uint8_t *)imgaddr, imgsize);   //�ٷ���ͼ��
}

void  main(void)
{
  
    system_init();
    uart_init(UART4, 115200);
  
        
    Ov7725_Init(image_bin);


    while(1)
    {
        /*if(system_mark)
        {
//            //�������
            if(hw_read_mark)    
            {
                LED(LED3,LED_ON);
                if(PTE4_IN&&PTE6_IN)           //���ⶼû���ź�ʱ
                {
                    hw_read_mark=0;
                    LED(LED3,LED_OFF);
                    system_run_mark=1;
                    servo_control_mark=1;
                }
            } 
    
            MPU6050();
            NRF();
            uart_Rx();
//            wifi();
            if(system_run_mark)
            {
    
                if(ONline_RUN==1)
                {
                    servo_mark=1;
                }
                if(Offline_RUN == 1)
                {
                    
                    servo_mark=2; 
        
                }
                servo();
            }
        }*/
      ov7727_get_img();

      img_extract(image_bin,img,CAMERA_SIZE);
      sendimg(img,CAMERA_W * CAMERA_H);
        
    }
}







