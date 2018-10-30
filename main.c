#include "sys.h"
#include "delay.h"
#include "led.h"
#include "key.h"  
#include "rs485.h"

//ALIENTEK 探索者STM32F407开发板 实验
//RS485通信实验-库函数版本
//作者：linkenwild
//主站发送10个字节的数据帧，当从站收到数据帧的最后一个字节后
//从站的板载LED会来回频闪，主站发送完成会亮一只LED
int main(void)
{ 
	u8 key;
	uint8_t rs485buf[10] = {0x3A,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x0D,0x0A};
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);     //初始化延时函数
	LED_Init();					//初始化LED  
	KEY_Init(); 				//按键初始化  
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

