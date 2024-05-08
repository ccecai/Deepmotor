//
// Created by 1 on 2024-05-04.
//
#include "Programming_by_Demonstration.h"

float Memory_theta[7][50] = {0};

uint8_t Memory_count = 0;

void Demonstration(void)
{
    static uint8_t count = 0;

    if(Final_Data[1].Speed != 0 || Final_Data[2].Speed != 0 || Final_Data[3].Speed != 0
       || Final_Data[4].Speed != 0 || Final_Data[5].Speed != 0 || Final_Data[6].Speed != 0)
    {
        count = 0;
    }

    else if(Final_Data[1].Speed == 0 && Final_Data[2].Speed == 0 && Final_Data[3].Speed == 0
    && Final_Data[4].Speed == 0 && Final_Data[5].Speed == 0 && Final_Data[6].Speed == 0 && count == 0)
    {
        for (int i = 1; i < 7; ++i)
        {
            Memory_theta[i][Memory_count] = Final_Data[i].Angle;
        }
        Memory_count++;
        count++;
    }
}
void by_Programming(void )
{
    static uint8_t count = 0;

    for (int i = 1; i < 7; ++i)
    {
        TargetAngle[i] = Memory_theta[i][count];
    }

    count++;
    osDelay(2000);
}