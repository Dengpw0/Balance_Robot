#include "led.h" 
#include "stm32f10x_gpio.h" 

//LEDӲ����ʼ����������
void LED_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //ʹ��GPIODʱ�ӣ�GPIOD������APB2ʱ���£���STM32��ʹ��IO��ǰ��Ҫʹ�ܶ�Ӧʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14; //����13 14
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; //�����������ģʽΪ�������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //��������ٶ�Ϊ50MHZ
	GPIO_Init(GPIOD, &GPIO_InitStructure); //�����������úõ�GPIO_InitStructure��������ʼ������GPIOD13 14
	
	GPIO_ResetBits(GPIOD, GPIO_Pin_13|GPIO_Pin_14); //��ʼ����������GPIOD13.14Ϊ�͵�ƽ
}
