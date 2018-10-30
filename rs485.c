#include "sys.h"
#include "rs485.h"
#include "delay.h"
#include "led.h"

//���ջ�����
#if EN_USART2_RX
uint8_t RS485_RX_BUF[64];
uint8_t RS485_RX_CNT = 0;//�������ݵĳ���
void USART2_IRQHandler(void)
{
	uint8_t res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)	//���յ�����
	{
		res=USART_ReceiveData(USART2);//��ȡ���յ�������USART2->DR
		if(RS485_RX_CNT < 64)
		{
			RS485_RX_BUF[RS485_RX_CNT] = res;//���յ����ݴ�������RX_BUF[]
			RS485_RX_CNT++;	//������������1
		}	
	}
}
#endif
//��ʼ��GPIO��USART2
//bound:���ڲ�����
void RS485_Init(uint32_t bound)	
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOG,ENABLE);//����GPIOA��GPIOGʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//����USARTʱ��
	//����2���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

	//USART2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;//GPIOA2��GPIOA3
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//�ٶ�100MHZ
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStruct);//��ʼ��PA2��PA3
	
	//PG8�������������485ģʽ����
	//PG8=0��ʾ����ģʽ��PG8=1��ʾ����ģʽ
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;//GPIOG8
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//���ģʽ
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//�ٶ�100MHZ
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOG, &GPIO_InitStruct);//��ʼ��PG8

	//USART2��ʼ������
	USART_InitStruct.USART_BaudRate = bound;//����������
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;//�ֳ�8λ
	USART_InitStruct.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStruct.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_Init(USART2, &USART_InitStruct);//��ʼ��USART2
	
	USART_Cmd(USART2, ENABLE);//ʹ�ܴ���2
	USART_ClearFlag(USART2, USART_FLAG_TC);//���������ɱ�־
	
	//�������EN_USART2_RX��0����ִ����������--rs485.hͷ�ļ��а����Ըñ����ĺ궨�塣
	#if EN_USART2_RX	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//���������ж�

	//USART2 NVIC����
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;//��Ӧ���ȼ�1
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStruct);//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	#endif
	RS485_TX_EN = 0;//Ĭ�Ͻ���ģʽ
}

/*
**RS485����len���ֽ�
**buf:���ͻ������׵�ַ
**len:���͵��ֽ�����С��64���ֽڣ�
*/


void RS485_Send_Data(uint8_t *buf,uint8_t len)
{
	uint8_t t;//forѭ������
	RS485_TX_EN = 1;//����ģʽ
	for(t = 0;t < len;t++)
	{
//		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ����ͽ���
//		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);//�ȴ����ͽ���	
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET)//һֱ���ȣ��ȴ���������жϱ�־λΪ1������ѭ��		//������һ���ֽ�
		{}	
	//	delay_ms(5);
		if(USART_GetFlagStatus(USART2, USART_FLAG_TC))
		{
			USART_SendData(USART2, buf[t]);//��������
			LED0=0;
		}
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//�ȴ����ͽ���
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;	
}


	
/*
**RS485��ѯ���յ�������
**buf:���ͻ������׵�ַ
**len:���������ݳ���
*/

void RS485_Receive_Data(uint8_t *buf,uint8_t *len)	
{
	uint8_t rxlen = RS485_RX_CNT;
	uint8_t i = 0;
	*len = 0;//Ĭ��Ϊ0
	delay_ms(10);//�ȴ�10ms����������10msû�н��յ�һ�����ݣ�����Ϊ���ս���
	if(rxlen == RS485_RX_CNT && rxlen)//���յ������ݣ��ҽ��������
	{
		for(i= 0;i < rxlen;i++)
		{
			buf[i] = RS485_RX_BUF[i]; 
		}
		*len = RS485_RX_CNT;//��¼�������ݳ���
		RS485_RX_CNT = 0;//���ռ�������
	}
}







