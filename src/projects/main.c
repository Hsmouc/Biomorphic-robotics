/******************** ********************
 * 文件名       ：main.c
 * 描述         ：
 *
**********************************************************************************/
#include "common.h"
#include "include.h"
#include "math.h"
u32 fuck;
u32 LEdge,REdge;
u8 image_bin[CAMERA_SIZE]; 
u8 img[CAMERA_W * CAMERA_H];
int imgErr;
int mid = 160;
u8 flag = 0;

void sendimg(void *imgaddr, uint32_t imgsize)
{
    uart_putchar(UART4, 0x00);                          //发送四个字节命令
    uart_putchar(UART4, 0xff);
    uart_putchar(UART4, 0x01);
    uart_putchar(UART4, 0x01);
    uart_sendN(UART4, (uint8_t *)imgaddr, imgsize);   //再发送图像
}

int imgProcess(void){
  u32 i;
  int err;
  //u8 min = 0;
  //u8 max = 300;
  LEdge = 10;
  REdge = 310;
  for(i = 320*50+mid; i < 320*50+310 ;i++){
    if(*(img+i) == 0x00){
      REdge = i-320*50;
      break;
    }
  }
  for(i = 320*50+mid; i > 320*50+10 ;i--){
    if(*(img+i) == 0x00){
      LEdge = i-320*50;
      break;
    }
  }
  mid = (LEdge + REdge)/2;
  err = (LEdge + REdge)/2 - 160;
  
  return err;
}


void  main(void) {

    system_init();
    //gpio_init(PORTC,14,GPO,HIGH);
    
       
    while(1){
      
        Ov7725_Init(image_bin);
        ov7727_get_img();
        img_extract(image_bin,img,CAMERA_SIZE);
        imgErr = imgProcess();
        if(imgErr < -25){
           flag = 1;
        }   
        if(imgErr > 25) {
           flag = 2;
        }
        if(imgErr >= -25 && imgErr <= 25){
          flag = 0;
        }
        motionSwitch(flag);
        motionCtr();
        sendimg(img,CAMERA_W * CAMERA_H);
    }
}







