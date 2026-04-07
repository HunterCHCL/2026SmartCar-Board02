/*
 * UARTComms.c
 *
 *  Created on: Mar 19, 2026
 *      Author: HunterCHCL
 */

#include "UARTComms.h"


extern osThreadId_t CommsHandle;

#define COMMS_SIGNAL_RECEIVED 0x01

uint8_t receiveBuffer[50];
uint8_t globalBuffer[50];
uint8_t receivedData[48];
uint8_t receivedCMD;

void UARTComms_Transmmit_Data(UART_HandleTypeDef *UARTPort,uint8_t cmd,uint8_t *data,uint8_t len)
{
	globalBuffer[0]=cmd;
	memcpy(&globalBuffer[1], data, len);
	Verification_AddXOR(globalBuffer, len+1);
	memmove(globalBuffer+2, globalBuffer, len + 2);
	globalBuffer[0] = PackageHead1;
	globalBuffer[1] = PackageHead2;
	HAL_UART_Transmit_DMA(UARTPort,globalBuffer,len+4);
}

void UARTComms_Receive_Data(UART_HandleTypeDef *UARTPort,uint8_t *received,uint8_t len)
{
    if(received[0]==PackageHead1&&received[1]==PackageHead2)
    {
        // 剥离包头
        memmove(received,received+2,len-2);
        len-=2;
        if(Verification_CheckXOR(received, len))
        {
            receivedCMD=received[0];
            // 复制数据
            memcpy(receivedData, received+1, len-2);
        }
        else
        {
            uint8_t errorMsg[] = "Verification Error\r\n";
            HAL_UART_Transmit(UARTPort, errorMsg, sizeof(errorMsg) - 1, HAL_MAX_DELAY);
            return;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&UARTComms_Port)
	{
		UARTComms_Receive_Data(&UARTComms_Port, receiveBuffer, Size);
        if (CommsHandle != NULL) {
            osThreadFlagsSet(CommsHandle, COMMS_SIGNAL_RECEIVED);
        }
		HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_Port, receiveBuffer, sizeof(receiveBuffer));
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart==&UARTComms_Port)
    {
        /* 发生错误（如Overrun Error）后重新启动DMA接收 */
        HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_Port, receiveBuffer, sizeof(receiveBuffer));
    }
}

void UARTComms_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_Port, receiveBuffer, sizeof(receiveBuffer));
}

void CommsTask(void *argument)
{
    UARTComms_Init();
    // uint8_t hello[]={'h','e','l','l','o'};
    // UARTComms_Transmmit_Data(&UARTComms_Port, 0x01, hello, sizeof(hello));
    uint32_t flags;

    while(1)
    {
        flags = osThreadFlagsWait(COMMS_SIGNAL_RECEIVED, osFlagsWaitAny, osWaitForever);

        if (flags & COMMS_SIGNAL_RECEIVED)
        {
            if(receivedCMD==0x1A)
            {
                Motor_RL.current_position = 0.0f;
                Motor_RR.current_position = 0.0f;
                memcpy(&Motor_RL.target_position, &receivedData[0], sizeof(float));
                Motor_RL.mode = MOTOR_CONTROL_POSITION;
                memcpy(&Motor_RR.target_position, &receivedData[4], sizeof(float));
                Motor_RR.mode = MOTOR_CONTROL_POSITION;
                // UARTComms_Transmmit_Data(&UARTComms_Port, 0xF1, receivedData, sizeof(float)*2);
            }
            else if(receivedCMD==0x1B)
            {
                memcpy(&Motor_RL.target_velocity, &receivedData[0], sizeof(float));
                Motor_RL.mode = MOTOR_CONTROL_VELOCITY;
                memcpy(&Motor_RR.target_velocity, &receivedData[4], sizeof(float));
                Motor_RR.mode = MOTOR_CONTROL_VELOCITY;
                // UARTComms_Transmmit_Data(&UARTComms_Port, 0xF1, receivedData, sizeof(float)*2);
            }
            receivedCMD = 0;
        }
    }
}
