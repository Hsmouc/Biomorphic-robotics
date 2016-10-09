/******************** ********************
 * 文件名       ：main.c
 * 描述         ：串口收发测试  接收数据并发送
 * 
 *
**********************************************************************************/
#include "common.h"
#include "include.h"

u8 image_bin[CAMERA_SIZE]; 
u8 img[CAMERA_W * CAMERA_H];

void sendimg(void *imgaddr, uint32_t imgsize)
{
    uart_putchar(UART4, 0x00);                          //发送四个字节命令
    uart_putchar(UART4, 0xff);
    uart_putchar(UART4, 0x01);
    uart_putchar(UART4, 0x01);
    uart_sendN(UART4, (uint8_t *)imgaddr, imgsize);   //再发送图像
}
void  main(void)
{
  
//    u8 str;
//    uart_init(UART4, 19200);   //初始化串口4，波特率为19200 ,波特率太大，容易不稳定
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