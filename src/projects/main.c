/******************** ********************
 * �ļ���       ��main.c
 * ����         �������շ�����  �������ݲ�����
 * 
 *
**********************************************************************************/
#include "common.h"
#include "include.h"

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
  
//    u8 str;
//    uart_init(UART4, 19200);   //��ʼ������4��������Ϊ19200 ,������̫�����ײ��ȶ�
    uart_init(UART4, 115200);
  
        
    Ov7725_Init(image_bin);

    Servo_Init();
    while(1)
    {
//        Ov7725_Init(image_bin);
        ov7727_get_img();

        img_extract(image_bin,img,CAMERA_SIZE);
        sendimg(img,CAMERA_W * CAMERA_H);
      
    }
}