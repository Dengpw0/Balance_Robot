#include "sys.h"
#include "usart.h"
#include "main.h"
#include "led.h"
#include "oled.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

#define DEBUG_USARTx USART1

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
/* 发送一个字节 */
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
/* 发送两个字节的数据 */
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
 
/* 发送8位数据的数组 */
void Usart_SendArray(USART_TypeDef* pUSARTx, uint8_t *array,uint8_t num)
{
	uint8_t i;
	for( i=0; i<num; i++ )
  {
		Usart_SendByte(pUSARTx, array[i]);
	}
	while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET );
}
 
/* 发送字符串 */
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
 
///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
 
///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);
 
		return (int)USART_ReceiveData(DEBUG_USARTx);
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}
u8  Send_Data_Buf[30];
void USART_Send_Data(u8 data);	//通过串口2发送一个八位字符
void USART_Send_Buf(u8 *DataToSend , u8 data_num)
{
	int data_i=0;
	for(data_i=0;data_i<data_num;data_i++)
		USART1_Send_Data(*(DataToSend+data_i));
}
void Send_data8(u8 *dataddr,u8 len,u8 func_word)  
{ 
	/*更改了匿名上位机版本型号使用这个更方便*/
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
void USART1_IRQHandler(void)    //串口1中断服务程序
{
	Res = USART_ReceiveData(USART1);
	switch(Res)
	{
		case 48://0
			OLED_Clear();			//清屏上次内容
			targetspeed = 0;	//回到平衡状态
		  Yaw_target = yaw;
			task_flag = 0;		//终止任务
		  LED=!LED;
		  break;
		case 49://1
		  targetspeed = 60; //前进
		  LED=!LED;
		  break;
		case 50:
		  targetspeed = -60;//后退
			LED=!LED;
		  break;
		case 51:
			Yaw_target -= 88;	//右转
			LED=!LED;
		  break;
		case 52:
			Yaw_target += 88;	//左转
			LED=!LED;
		  break;
		case 53://5
			OLED_Clear();			//清屏上次内容
			task_flag = 1;		//执行搬运任务
	  	times = 0;				//清楚计时，开始重新计时
			LED=!LED;
			break;
	}
	if(Yaw_target>100)		//左右转限幅
		Yaw_target = 100;
	if(Yaw_target < -100)
		Yaw_target = -100;
} 
 #endif	



