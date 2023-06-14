#include "sys.h"
#include "usart.h"
#include "main.h"
#include "led.h"
#include "oled.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

#define DEBUG_USARTx USART1

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
/* ����һ���ֽ� */
void Usart_SendByte(USART_TypeDef* pUSARTx, uint8_t data)
{
	USART_SendData(pUSARTx, data);
	while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET );
}
void USART1_Send_Data(u8 data)
{
	USART_SendData(USART1, data);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET){}; 
		USART1->DR=(data & (uint16_t)0x01FF);
}
/* ���������ֽڵ����� */
void Usart_SendHalfWord(USART_TypeDef* pUSARTx, uint16_t data)
{
	uint8_t temp_h,temp_l;
	
	temp_h = (data&0xff00) >> 8 ;
	temp_l = data&0xff;
	
	USART_SendData(pUSARTx, temp_h);
	while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET );
	
	USART_SendData(pUSARTx, temp_l);
	while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET );
}
 
/* ����8λ���ݵ����� */
void Usart_SendArray(USART_TypeDef* pUSARTx, uint8_t *array,uint8_t num)
{
	uint8_t i;
	for( i=0; i<num; i++ )
  {
		Usart_SendByte(pUSARTx, array[i]);
	}
	while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET );
}
 
/* �����ַ��� */
void Usart_SendStr(USART_TypeDef* pUSARTx, uint8_t *str)
{
	uint8_t i=0;
	do
  {
		Usart_SendByte(pUSARTx, *(str+i));
		i++;
	}while(*(str+i) != '\0');
	while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET );
}
 
///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
 
///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);
 
		return (int)USART_ReceiveData(DEBUG_USARTx);
}
#endif
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}
u8  Send_Data_Buf[30];
void USART_Send_Data(u8 data);	//ͨ������2����һ����λ�ַ�
void USART_Send_Buf(u8 *DataToSend , u8 data_num)
{
	int data_i=0;
	for(data_i=0;data_i<data_num;data_i++)
		USART1_Send_Data(*(DataToSend+data_i));
}
void Send_data8(u8 *dataddr,u8 len,u8 func_word)  
{ 
	/*������������λ���汾�ͺ�ʹ�����������*/
	u8 i,count;
	u8 now_word;
	u8 sc,ac;
	ac=sc=0;
	if(len>20) len=20;
	if((func_word>=1)&&(func_word<=10))
		now_word=0xF0|func_word;
	Send_Data_Buf[0]=0xAA;
	Send_Data_Buf[1]=0xFF;
	Send_Data_Buf[2]=now_word;
	Send_Data_Buf[3]=len;
	for(i=0;i<len;i++) 
		Send_Data_Buf[4+i] = *(dataddr+i);       
	for(i=0;i<Send_Data_Buf[3]+4;i++)
	{
		sc+=Send_Data_Buf[i];
		ac+=sc;
	}
	Send_Data_Buf[4+len]=sc;
	Send_Data_Buf[5+len]=ac;

 	USART_Send_Buf(Send_Data_Buf,count);
}

int Res;
void USART1_IRQHandler(void)    //����1�жϷ������
{
	Res = USART_ReceiveData(USART1);
	switch(Res)
	{
		case 48://0
			OLED_Clear();			//�����ϴ�����
			targetspeed = 0;	//�ص�ƽ��״̬
		  Yaw_target = yaw;
			task_flag = 0;		//��ֹ����
		  LED=!LED;
		  break;
		case 49://1
		  targetspeed = 60; //ǰ��
		  LED=!LED;
		  break;
		case 50:
		  targetspeed = -60;//����
			LED=!LED;
		  break;
		case 51:
			Yaw_target -= 88;	//��ת
			LED=!LED;
		  break;
		case 52:
			Yaw_target += 88;	//��ת
			LED=!LED;
		  break;
		case 53://5
			OLED_Clear();			//�����ϴ�����
			task_flag = 1;		//ִ�а�������
	  	times = 0;				//�����ʱ����ʼ���¼�ʱ
			LED=!LED;
			break;
	}
	if(Yaw_target>100)		//����ת�޷�
		Yaw_target = 100;
	if(Yaw_target < -100)
		Yaw_target = -100;
} 
 #endif	



