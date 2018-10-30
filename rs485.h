#ifndef __RS485_H
#define __RS485_H
#include "sys.h"

extern uint8_t RS485_RX_BUF[64];
extern uint8_t RS485_RX_CNT;



#define RS485_TX_EN PGout(8)
#define EN_USART2_RX 1


 void RS485_Init(uint32_t bound);
void RS485_Send_Data(uint8_t *buf,uint8_t len);
void RS485_Receive_Data(uint8_t *buf,uint8_t *len);	
#endif

