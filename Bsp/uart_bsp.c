#include "uart_bsp.h"
#include "string.h"
#include "usart.h"
#include "printf.h"

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00

#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)

uint8_t rx_buff[BUFF_SIZE];
remoter_t remoter;
float prx,pry,prz = 0;

void sbus_frame_parse(remoter_t *remoter, uint8_t *buf)
{
    remoter->rc.ch[0] = (buf[0] | (buf[1] << 8)) & 0x07ff;        //!< Channel 0
    remoter->rc.ch[1] = ((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff; //!< Channel 1
    remoter->rc.ch[2] = ((buf[2] >> 6) | (buf[3] << 2) |          //!< Channel 2
                         (buf[4] << 10)) &0x07ff;
    remoter->rc.ch[3] = ((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff; //!< Channel 3
    remoter->rc.ch[4] = ((buf[5] >> 4) & 0x0003);
    remoter->rc.ch[5] = ((buf[5] >> 4) & 0x000C) >> 2;

    remoter->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remoter->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remoter->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remoter->rc.ch[3] -= RC_CH_VALUE_OFFSET;
//    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
//        return;
//
//    if (buf[23] == 0x0C)
//        remoter->online = 0;
//    else
//        remoter->online = 1;
//
//    remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
//    remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
//    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
//    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
//    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
//    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
//    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
//    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
//    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
//    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == UART5)
	{
		if (Size <= BUFF_SIZE)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // 接收完毕后重启
			sbus_frame_parse(&remoter, rx_buff);
//			memset(rx_buff, 0, BUFF_SIZE);
		}
		else  // 接收数据长度大于BUFF_SIZE，错误处理
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // 接收完毕后重启
			memset(rx_buff, 0, BUFF_SIZE);							   
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // 接收发生错误后重启
		memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存		
	}
}

void RC_process(void)
{
//    usart_printf("%d,%d,%d,%d,%d,%d\n",remoter.rc.ch[0],remoter.rc.ch[1],remoter.rc.ch[2],remoter.rc.ch[3],remoter.rc.ch[4],remoter.rc.ch[5]);

    if(remoter.rc.ch[2] >= 600)
        prx += 0.01f;
    else if(remoter.rc.ch[2] <= -600)
        prx -= 0.01f;

    if(remoter.rc.ch[3] >= 600)
        pry += 0.01f;
    else if(remoter.rc.ch[3] <= -600)
        pry -= 0.01f;

    if(remoter.rc.ch[1] >= 600)
        prz += 0.01f;
    else if(remoter.rc.ch[1] <= -600)
        prz -= 0.01f;
}