/**
 * @file drv_can.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 3.2
 * @date 2023-08-02 0.1 23赛季定稿
 * @date 2023-11-10 1.1 修改成cpp
 * @date 2024-01-01 1.2 官方6020驱动更新, 适配电压控制与电流控制
 * @date 2024-03-09 1.3 适配新赛季超级电容, 24赛季定稿
 * @date 2024-08-19 2.1 适配达妙电机
 * @date 2024-08-22 2.2 新增回调函数空指针判定
 * @date 2025-08-13 3.1 适配达妙MC02板
 * @date 2025-11-19 3.2 改为C语言实现(纯C, 可被C/C++混合工程包含)
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

/* Includes ------------------------------------------------------------------*/

#include "fdcan.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN通信接收回调函数数据类型
 * @note  C版本用指针传递帧头, 不再是C++的引用
 *
 * @param Header 接收帧头
 * @param Buffer 接收数据缓冲区
 */
typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);

/**
 * @brief CAN通信处理结构体
 *
 */
typedef struct
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;

    // 与接收相关的数据
    FDCAN_RxHeaderTypeDef Rx_Header;
    uint8_t Rx_Buffer[64];

    // 接收时间戳
    uint64_t Rx_Timestamp;
} Struct_CAN_Manage_Object;

/* Exported variables ---------------------------------------------------------*/

extern bool init_finished;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;

extern uint8_t CAN1_0x1fe_Tx_Data[8];
extern uint8_t CAN1_0x1ff_Tx_Data[8];
extern uint8_t CAN1_0x200_Tx_Data[8];
extern uint8_t CAN1_0x2fe_Tx_Data[8];
extern uint8_t CAN1_0x2ff_Tx_Data[8];
extern uint8_t CAN1_0x3fe_Tx_Data[8];
extern uint8_t CAN1_0x4fe_Tx_Data[8];

extern uint8_t CAN2_0x1fe_Tx_Data[8];
extern uint8_t CAN2_0x1ff_Tx_Data[8];
extern uint8_t CAN2_0x200_Tx_Data[8];
extern uint8_t CAN2_0x2fe_Tx_Data[8];
extern uint8_t CAN2_0x2ff_Tx_Data[8];
extern uint8_t CAN2_0x3fe_Tx_Data[8];
extern uint8_t CAN2_0x4fe_Tx_Data[8];

extern uint8_t CAN3_0x1fe_Tx_Data[8];
extern uint8_t CAN3_0x1ff_Tx_Data[8];
extern uint8_t CAN3_0x200_Tx_Data[8];
extern uint8_t CAN3_0x2fe_Tx_Data[8];
extern uint8_t CAN3_0x2ff_Tx_Data[8];
extern uint8_t CAN3_0x3fe_Tx_Data[8];
extern uint8_t CAN3_0x4fe_Tx_Data[8];

/* Exported function declarations ---------------------------------------------*/

void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function);

uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length);

void TIM_100us_CAN_PeriodElapsedCallback(void);

void TIM_1ms_CAN_PeriodElapsedCallback(void);

/**
 * @brief 获取当前时间戳(单位: 微秒)的弱函数声明
 * @note  默认为弱实现(见drv_can.c), 工程里有自己的C版时间戳模块时,
 *        在任意.c文件里实现同名强函数即可自动覆盖, 无需修改本文件
 */
uint64_t SYS_Timestamp_Get_Current_Timestamp(void);

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
