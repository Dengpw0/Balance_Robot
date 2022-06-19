#ifndef __LED_H 
#define __LED_H 

void LED_Init(void); 
#define LED1 PDout(13)// PB13
#define LED2 PDout(14)// PB14	
#endif //定义完毕，或者引用过头文件到达这一步
