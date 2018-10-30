#include "sys.h"
#include "delay.h"
#include "led.h"
#include "key.h"  
#include "rs485.h"

//ALIENTEK ̽����STM32F407������ ʵ��
//RS485ͨ��ʵ��-�⺯���汾
//���ߣ�linkenwild
//��վ����10���ֽڵ�����֡������վ�յ�����֡�����һ���ֽں�
//��վ�İ���LED������Ƶ������վ������ɻ���һֻLED
int main(void)
{ 
	u8 key;
	uint8_t rs485buf[10] = {0x3A,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x0D,0x0A};
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);     //��ʼ����ʱ����
	LED_Init();					//��ʼ��LED  
	KEY_Init(); 				//������ʼ��  
	RS485_Init(9600);
	
	while(1)								
	{
		key = KEY_Scan(0);
		if(key == KEY0_PRES)
		{
			RS485_Send_Data(rs485buf,10);
		}
		RS485_Receive_Data(rs485buf,&key);
		if(RS485_RX_BUF[9] == 0x0A)
		{
			LED0 =!LED0;
			delay_ms(100);
			LED1 =!LED1;	
			delay_ms(100);			
		}
	}      
}

