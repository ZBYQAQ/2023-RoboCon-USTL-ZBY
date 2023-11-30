#ifndef  BSP_CAN_H
#define  BSP_CAN_H

#include "can.h"
#include "motor.h"
enum{
    _CAN1,
    _CAN2,
    CAN_PORT_NUM,
};

enum{
    _0x1FF,
    _0x200,
    stdID_NUM,
};

typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header;
    uint8_t 			data[8];
} CAN_RxFrameTypeDef;

typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef header;
    uint8_t				data[8];
}CAN_TxFrameTypeDef;

extern CAN_TxFrameTypeDef hcan1TxFrame,ChassisTxFrame;
extern CAN_TxFrameTypeDef hcan2TxFrame,GimbalTxFrame;

extern float chassis_pitangle;
extern float gimbal_pitangle;

/* mycan functions ---------------------------------------------------------*/
extern void BSP_CAN_Init(void);
extern void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader);

#endif //GIMBAL_MYCAN_H
