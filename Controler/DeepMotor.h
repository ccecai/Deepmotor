//
// Created by 1 on 2024-05-02.
//

#ifndef ROBOTIC_ARM_B_DEEPMOTOR_H
#define ROBOTIC_ARM_B_DEEPMOTOR_H

#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"

typedef struct {
    uint32_t Angle;
    uint32_t Speed;
    uint16_t Torque;
    uint8_t Temperature_flag;
    uint8_t Temperature;
}ControlData;

typedef struct {
    float Angle;
    float Speed;
    float Torque;
    uint8_t Temperature_flag;
    uint8_t Temperature;
}FinalData;

#define Disable_Motor_ID 0x020 //1<<5
#define Able_Motor_ID    0x040 //2<<5
#define Control_Motor_ID 0x080 //4<<5

#define Disable_ID1 1 + Disable_Motor_ID
#define Disable_ID2 2 + Disable_Motor_ID
#define Disable_ID3 3 + Disable_Motor_ID
#define Disable_ID4 4 + Disable_Motor_ID
#define Disable_ID5 5 + Disable_Motor_ID
#define Disable_ID6 6 + Disable_Motor_ID

#define Able_ID1    1 + Able_Motor_ID
#define Able_ID2    2 + Able_Motor_ID
#define Able_ID3    3 + Able_Motor_ID
#define Able_ID4    4 + Able_Motor_ID
#define Able_ID5    5 + Able_Motor_ID
#define Able_ID6    6 + Able_Motor_ID

#define Control_ID1    1 + Control_Motor_ID
#define Control_ID2    2 + Control_Motor_ID
#define Control_ID3    3 + Control_Motor_ID
#define Control_ID4    4 + Control_Motor_ID
#define Control_ID5    5 + Control_Motor_ID
#define Control_ID6    6 + Control_Motor_ID

#define Disable_ID1_Receive 1 + Disable_Motor_ID + 0x10
#define Disable_ID2_Receive 2 + Disable_Motor_ID + 0x10
#define Disable_ID3_Receive 3 + Disable_Motor_ID + 0x10
#define Disable_ID4_Receive 4 + Disable_Motor_ID + 0x10
#define Disable_ID5_Receive 5 + Disable_Motor_ID + 0x10
#define Disable_ID6_Receive 6 + Disable_Motor_ID + 0x10

#define Able_ID1_Receive    1 + Able_Motor_ID + 0x10
#define Able_ID2_Receive    2 + Able_Motor_ID + 0x10
#define Able_ID3_Receive    3 + Able_Motor_ID + 0x10
#define Able_ID4_Receive    4 + Able_Motor_ID + 0x10
#define Able_ID5_Receive    5 + Able_Motor_ID + 0x10
#define Able_ID6_Receive    6 + Able_Motor_ID + 0x10

#define Control_ID1_Receive    1 + Control_Motor_ID + 0x10
#define Control_ID2_Receive    2 + Control_Motor_ID + 0x10
#define Control_ID3_Receive    3 + Control_Motor_ID + 0x10
#define Control_ID4_Receive    4 + Control_Motor_ID + 0x10
#define Control_ID5_Receive    5 + Control_Motor_ID + 0x10
#define Control_ID6_Receive    6 + Control_Motor_ID + 0x10

extern ControlData FeedBack_Data;
extern FinalData Final_Data[7];
extern float TargetAngle[7];

void CAN_CMD_MOTOR_DISABLE(FDCAN_HandleTypeDef *_hfdcan,uint32_t stdid);
void CAN_CMD_MOTOR_ENABLE(FDCAN_HandleTypeDef *_hfdcan,uint32_t stdid);
void CAN_CMD_MOTOR_CONTROL(FDCAN_HandleTypeDef *_hfdcan,float TargetAngle,float TargetSpeed,
                           float Kp,float Kd,float TargetTorque,float stdid);
void ReceiveData_Process(ControlData *Data,uint8_t id);
void AllMotor_ENABLE(void);
void AllMotor_DISABLE(void);

#endif //ROBOTIC_ARM_B_DEEPMOTOR_H
