/******************** ********************
 * 文件名       ：main.c
 * 描述         ：
 *
**********************************************************************************/
#include "common.h"
#include "include.h"
#include "math.h"

u32 LEdge,REdge;
u8 image_bin[CAMERA_SIZE]; 
u8 img[CAMERA_W * CAMERA_H];
int last_err;
int err;
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
  u8 min = 12;
  u8 max = 45;
  for(i = 320*200; i < 320*201 ;i++){
    if(*(img+i) == 0){
      LEdge = i-320*200;
      break;
    }
  }
  for(i = 320*201;i > 320*200;i--){
    if(*(img+i) == 0){
      REdge = i-320*200;
      break;
    }
  }
  err = (LEdge + REdge)/2 - 160;
  
  if(REdge - LEdge < max && REdge - LEdge > min)
    err = err;
  else
    err = last_err;
  
  last_err = err;
  return err;
}


void  main(void) {

    u8 last_flag = 0;
    system_init();
    uart_init(UART4, 115200);
       
    Ov7725_Init(image_bin);
    gpio_init(PORTC, 14, GPO, LOW); 
    while(1){
      //motionSwitch(flag);
      if(flag == 1){
         PTC14_OUT = 1;
      }
        while(1){
          //motionCtr();
          ov7727_get_img();
          img_extract(image_bin,img,CAMERA_SIZE);
          err = imgProcess();
          if(err < -80)
            flag = 1;
          else if(err > 80) 
            flag = 2;
          else
            flag = 0;
          sendimg(img,CAMERA_W * CAMERA_H);
         
          if(last_flag != flag){
            last_flag = flag; 
            break;
          }
          last_flag = flag;
        }
    }
}







