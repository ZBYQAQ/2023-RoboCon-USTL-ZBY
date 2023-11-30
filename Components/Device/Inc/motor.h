//
// Created by YanYuanbin on 22-10-5.
//

#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H

#include "pid.h"
#include "bsp_can.h"

typedef enum{
     A,
	   B,
	   C,
    MOTOR_NUM,
}MOTOR_USAGE;
typedef  enum {
     motor1,
	   motor2,
	   motor3,
	   motor4,
    MOTOR_NUM_GIMBAL,
}MOTOR_USAGE_GIMBAL;

typedef enum{
    _6020,
    _3508,
    _2006,
		_3510,
    MOTOR_TYPE_NUM,
}MOTOR_TYPE;

typedef enum{
	GIMBAL,
	SHOOT,
	MOTOR_TOTAL_TYPE_NUM
}MOTOR_TOTAL;

/*  ??????,??CAN?????,
    ??:???????????!!!??????????????
*/
//DJI电机基础参数
typedef struct
{
    struct{
        uint32_t StdId;
        volatile float angle;
        volatile int16_t current;
        volatile int16_t velocity;
        volatile int16_t encoder;
        volatile uint8_t temperature;
			  volatile int16_t   last_encoder;
    }Data;

    CAN_TypeDef *CANx;
    MOTOR_TYPE type;
    MOTOR_USAGE usage;
		MOTOR_USAGE_GIMBAL usage2;
		uint8_t Initlized;
}DJI_MOTOR;

extern DJI_MOTOR Chassis_motor[MOTOR_NUM];
extern DJI_MOTOR Gimbal_motor[MOTOR_NUM_GIMBAL];

extern void get_Motor_Data(uint32_t *canId, uint8_t *rxBuf,DJI_MOTOR* DJI_Motor);

#endif //GIMBAL_MOTOR_H

