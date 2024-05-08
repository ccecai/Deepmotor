#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
#include "fdcan.h"
#include "DeepMotor.h"

typedef union
{
    uint8_t data_8[8];
    uint16_t data_16[4];
    int32_t data_int[2];
    uint32_t data_uint[2];
    float data_f[2];
}union_64;

void can_bsp_init(void);
void can1_filter_init(void);
void can2_filter_init(void);
void can3_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

#endif /* __CAN_BSP_H_ */

