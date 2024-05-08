#include "can_bsp.h"
/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void can_bsp_init(void)
{
    can1_filter_init();
    can2_filter_init();
    can3_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can1_filter_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig1;
    sFilterConfig1.IdType = FDCAN_STANDARD_ID; //标准ID
    sFilterConfig1.FilterIndex = 0;
    sFilterConfig1.FilterType = FDCAN_FILTER_RANGE; // 列表模式  后续应该要改成掩码模式
    sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig1.FilterID1 = 0X0000;
    sFilterConfig1.FilterID2 = 0X07FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1) != HAL_OK)
    {
        Error_Handler();
    }
    // 整个外设的开启要放在中断使能前面
    HAL_FDCAN_Start(&hfdcan1);
    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

void can2_filter_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig1;
    sFilterConfig1.IdType = FDCAN_STANDARD_ID; //标准ID
    sFilterConfig1.FilterIndex = 1;
    sFilterConfig1.FilterType = FDCAN_FILTER_RANGE; // 列表模式  后续应该要改成掩码模式
    sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig1.FilterID1 = 0X0000;
    sFilterConfig1.FilterID2 = 0X07FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig1) != HAL_OK)
    {
        Error_Handler();
    }
    // 整个外设的开启要放在中断使能前面
    HAL_FDCAN_Start(&hfdcan2);
    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

void can3_filter_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig1;
    sFilterConfig1.IdType = FDCAN_STANDARD_ID; //标准ID
    sFilterConfig1.FilterIndex = 2;
    sFilterConfig1.FilterType = FDCAN_FILTER_RANGE; // 列表模式  后续应该要改成掩码模式
    sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig1.FilterID1 = 0X0000;
    sFilterConfig1.FilterID2 = 0X07FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig1) != HAL_OK)
    {
        Error_Handler();
    }
    // 整个外设的开启要放在中断使能前面
    HAL_FDCAN_Start(&hfdcan3);
    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;																// 标准ID 
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;														// 数据帧 
  TxHeader.DataLength = len << 16;																		// 发送数据长度 
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										// 设置错误状态指示 								
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;															// 不开启可变波特率 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															// 普通CAN格式 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										// 用于发送事件FIFO控制, 不存储 
  TxHeader.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF                
    
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)!=HAL_OK)
		return 0;//接收数据
  return fdcan_RxHeader.DataLength>>16;	
}
/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan；FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的FDCAN中断回调函数
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
            fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
		if(hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}
/**
************************************************************************
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	供用户调用的接收弱函数
************************************************************************
**/
void fdcan1_rx_callback(void)
{
    HAL_StatusTypeDef HAL_RetVal;
    FDCAN_RxHeaderTypeDef RxHeader;
    union_64 rxdata;
    /*电机号记录*/
    static uint8_t index;

    HAL_RetVal = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, rxdata.data_8);
    if(HAL_RetVal == HAL_OK)
    {
        if(RxHeader.Identifier >= Disable_ID1_Receive && RxHeader.Identifier <= Disable_ID6_Receive)
        {
            if(rxdata.data_8[0] == 0)
            {
//                while(1);
            }
            else if(rxdata.data_8[0] == 1)
            {

            }
        }

        else if(RxHeader.Identifier >= Able_ID1_Receive && RxHeader.Identifier <= Able_ID6_Receive)
        {
            if(rxdata.data_8[0] == 0)
            {
//                while(1);
            }
            else if(rxdata.data_8[0] == 1)
            {

            }
        }

        else if(RxHeader.Identifier >= Control_ID1_Receive && RxHeader.Identifier <= Control_ID6_Receive)
        {
            index = (RxHeader.Identifier - 0x10) & 0x01F;

            FeedBack_Data.Angle = rxdata.data_8[0] | rxdata.data_8[1] << 8 | ((rxdata.data_8[2] << 16) & 0x0FFFFF);
            FeedBack_Data.Speed = ((rxdata.data_8[2] >> 4) & 0x0F) | rxdata.data_8[3] << 4 | rxdata.data_8[4] << 12;
            FeedBack_Data.Torque = rxdata.data_8[5] | rxdata.data_8[6] << 8;
            FeedBack_Data.Temperature_flag = rxdata.data_8[7] & 0x01;
            FeedBack_Data.Temperature = (rxdata.data_8[7] >> 1) & 0x7F;

            ReceiveData_Process(&FeedBack_Data,index);
        }

    }
    __HAL_FDCAN_ENABLE_IT(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}
uint8_t rx_data2[8] = {0};
void fdcan2_rx_callback(void)
{
	fdcanx_receive(&hfdcan2, rx_data2);
}
uint8_t rx_data3[8] = {0};
void fdcan3_rx_callback(void)
{
	fdcanx_receive(&hfdcan3, rx_data3);
}
