#include "sys.h"
#include "rs485.h"
#include "delay.h"
#include "led.h"

//接收缓冲区
#if EN_USART2_RX
uint8_t RS485_RX_BUF[64];
uint8_t RS485_RX_CNT = 0;//接收数据的长度
void USART2_IRQHandler(void)
{
	uint8_t res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)	//接收到数据
	{
		res=USART_ReceiveData(USART2);//读取接收到的数据USART2->DR
		if(RS485_RX_CNT < 64)
		{
			RS485_RX_BUF[RS485_RX_CNT] = res;//接收的数据存入数组RX_BUF[]
			RS485_RX_CNT++;	//接收数据增加1
		}	
	}
}
#endif
//初始化GPIO和USART2
//bound:串口波特率
void RS485_Init(uint32_t bound)	
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOG,ENABLE);//开启GPIOA和GPIOG时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//开启USART时钟
	//串口2引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

	//USART2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;//GPIOA2和GPIOA3
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//速度100MHZ
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStruct);//初始化PA2和PA3
	
	//PG8推挽输出，用于485模式控制
	//PG8=0表示接收模式；PG8=1表示发送模式
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;//GPIOG8
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//输出模式
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//速度100MHZ
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOG, &GPIO_InitStruct);//初始化PG8

	//USART2初始化设置
	USART_InitStruct.USART_BaudRate = bound;//波特率设置
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;//字长8位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init(USART2, &USART_InitStruct);//初始化USART2
	
	USART_Cmd(USART2, ENABLE);//使能串口2
	USART_ClearFlag(USART2, USART_FLAG_TC);//清除发送完成标志
	
	//如果变量EN_USART2_RX非0，则执行条件编译--rs485.h头文件中包含对该变量的宏定义。
	#if EN_USART2_RX	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//开启接收中断

	//USART2 NVIC配置
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;//响应优先级1
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能
	NVIC_Init(&NVIC_InitStruct);//根据指定的参数初始化NVIC寄存器
	#endif
	RS485_TX_EN = 0;//默认接收模式
}

/*
**RS485发送len个字节
**buf:发送缓冲区首地址
**len:发送的字节数（小于64个字节）
*/


void RS485_Send_Data(uint8_t *buf,uint8_t len)
{
	uint8_t t;//for循环计数
	RS485_TX_EN = 1;//发送模式
	for(t = 0;t < len;t++)
	{
//		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//等待发送结束
//		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);//等待发送结束	
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET)//一直死等，等待发送完成中断标志位为1，结束循环		//发送下一个字节
		{}	
	//	delay_ms(5);
		if(USART_GetFlagStatus(USART2, USART_FLAG_TC))
		{
			USART_SendData(USART2, buf[t]);//发送数据
			LED0=0;
		}
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//等待发送结束
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;	
}


	
/*
**RS485查询接收到的数据
**buf:发送缓冲区首地址
**len:读到的数据长度
*/

void RS485_Receive_Data(uint8_t *buf,uint8_t *len)	
{
	uint8_t rxlen = RS485_RX_CNT;
	uint8_t i = 0;
	*len = 0;//默认为0
	delay_ms(10);//等待10ms，连续超过10ms没有接收到一个数据，则认为接收结束
	if(rxlen == RS485_RX_CNT && rxlen)//接收到了数据，且接收完成了
	{
		for(i= 0;i < rxlen;i++)
		{
			buf[i] = RS485_RX_BUF[i]; 
		}
		*len = RS485_RX_CNT;//记录本次数据长度
		RS485_RX_CNT = 0;//接收计数清零
	}
}







