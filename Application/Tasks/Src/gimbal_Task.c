#include "Gimbal_Task.h"
#include "cmsis_os.h"
#include "motor.h"
#include "remote_control.h"
#include "arm_math.h"
#include "gpio.h"
#include "bsp_can.h"
#include "CANTx_Task.h"
#include "pid.h"
#include "bsp_uart.h"
int i,m,n;
float x;
float y;
//降压：IN分电板，out：光电开关
Gimbal_Info_Typedef Gimbal_Info={
	.v1=0,
	.v2=0,
	.v3=0,
	.v4=0,
	.Target.v1_angle=0,
	.Target.v2_angle=0,
	.Target.v3_angle=0,
	.Target.v4_angle=0,
  .angle_SendValue=0,
  .SendValue=0,
};
float Gimbal_PID_v1_Param [6]={30,0.00001f,0,0.f,5000.f,15000,};
float Gimbal_PID_v2_Param [6]={28,0.00001f,0,0.f,5000.f,15000,};
float Gimbal_PID_v3_Param [6]={25,0.00001f,0,0.f,5000.f,15000,};
float Gimbal_PID_v4_Param [6]={33,0.00001f,0,0.f,5000.f,15000,};
float Angle_PID_A1_Param [6]={17,0.f,0,0.f,5000.f,15000,};
float Angle_PID_A2_Param [6]={17,0.f,0,0.f,5000.f,15000,};
float Angle_PID_A3_Param [6]={15,0.f,0,0.f,5000.f,15000,};
float Angle_PID_A4_Param [6]={15,0.f,0,0.f,5000.f,15000,};

PID_Info_TypeDef Gimbal_PID_v1;
PID_Info_TypeDef Gimbal_PID_v2;
PID_Info_TypeDef Gimbal_PID_v3;
PID_Info_TypeDef Gimbal_PID_v4;

PID_Info_TypeDef Angle_Gimbal_A1_PID;
PID_Info_TypeDef Angle_Gimbal_A2_PID;
PID_Info_TypeDef Angle_Gimbal_A3_PID;
PID_Info_TypeDef Angle_Gimbal_A4_PID;

static void Gimbal_Task_Init(void);

static void Gimbal(Gimbal_Info_Typedef *Gimbal_Info);

void gimbal_Task(void const * argument)
{
	
  Gimbal_Task_Init();  
  for(;;)
{
	if(remote_ctrl.rc.s[1]==1)
	{
	  Gimbal_Info.SendValue[0]=0;
		Gimbal_Info.SendValue[1]=0;
		Gimbal_Info.SendValue[2]=0;
		Gimbal_Info.SendValue[3]=0;
	}
	else {
//光电开关未检测到光
	 m=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9);
	 n=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
   if(m&&n) 
{ 	
  Gimbal_Info.Target.v1_angle=-30;
	Gimbal_Info.Target.v2_angle=-30;
	Gimbal_Info.Target.v3_angle=-30;
	Gimbal_Info.Target.v4_angle=-30;
 }    // 光电开关检测到光
else 
	{
	 // 电机复位	 
//	Gimbal_Info.Target.v1_angle=-115;
//	Gimbal_Info.Target.v2_angle=-110;
//	Gimbal_Info.Target.v3_angle=-105;
//	Gimbal_Info.Target.v4_angle=-100;
		
	Gimbal_Info.Target.v1_angle=-90+remote_ctrl.rc.ch[3]*0.7;
	Gimbal_Info.Target.v2_angle=-85-remote_ctrl.rc.ch[3]*0.7;
	Gimbal_Info.Target.v3_angle=-90+remote_ctrl.rc.ch[2]*0.7;
	Gimbal_Info.Target.v4_angle=-85-remote_ctrl.rc.ch[2]*0.7;
//限幅
  VAL_LIMIT(Gimbal_Info.Target.v1_angle,-105,-15);
	VAL_LIMIT(Gimbal_Info.Target.v2_angle,-105,-15);
	VAL_LIMIT(Gimbal_Info.Target.v3_angle,-105,-15);
	VAL_LIMIT(Gimbal_Info.Target.v4_angle,-105,-15);
    } 
 Gimbal( &Gimbal_Info);
  }
}
 osDelay(1);	
}

static void Gimbal_Task_Init(){
  PID_Init(&Gimbal_PID_v1,PID_POSITION ,Gimbal_PID_v1_Param); 
  PID_Init(&Gimbal_PID_v2,PID_POSITION ,Gimbal_PID_v2_Param);
  PID_Init(&Gimbal_PID_v3,PID_POSITION ,Gimbal_PID_v3_Param);
	PID_Init(&Gimbal_PID_v4,PID_POSITION ,Gimbal_PID_v4_Param);
	
	PID_Init(&Angle_Gimbal_A1_PID,PID_POSITION ,Angle_PID_A1_Param); 
  PID_Init(&Angle_Gimbal_A2_PID,PID_POSITION ,Angle_PID_A2_Param);
  PID_Init(&Angle_Gimbal_A3_PID,PID_POSITION ,Angle_PID_A3_Param);
	PID_Init(&Angle_Gimbal_A4_PID,PID_POSITION ,Angle_PID_A4_Param);
}
static void Gimbal(Gimbal_Info_Typedef *Gimbal_Info){
	//云台速度关联遥控器
//  Gimbal_Info->Target.v1_velocity = remote_ctrl.rc.ch[1]*10;
//  Gimbal_Info->Target.v2_velocity = remote_ctrl.rc.ch[1]*10;
//  Gimbal_Info->Target.v3_velocity = remote_ctrl.rc.ch[1]*10;
//  Gimbal_Info->Target.v4_velocity = remote_ctrl.rc.ch[1]*10;
	//遥控器读取角度 
//  Gimbal_Info->Target.v1_angle=-remote_ctrl.rc.ch[3]*0.1;
//	Gimbal_Info->Target.v2_angle=remote_ctrl.rc.ch[3]*0.1;
//	Gimbal_Info->Target.v3_angle=-remote_ctrl.rc.ch[2]*0.1;
//	Gimbal_Info->Target.v4_angle=remote_ctrl.rc.ch[2]*0.1;
	//云台角度关联PID
	f_PID_Calculate(& Angle_Gimbal_A1_PID,Gimbal_Info->Target.v1_angle,Gimbal_motor[motor1].Data.angle);
	f_PID_Calculate(& Angle_Gimbal_A2_PID,Gimbal_Info->Target.v2_angle,Gimbal_motor[motor2].Data.angle);
	f_PID_Calculate(& Angle_Gimbal_A3_PID,Gimbal_Info->Target.v3_angle,Gimbal_motor[motor3].Data.angle);
  f_PID_Calculate(& Angle_Gimbal_A4_PID,Gimbal_Info->Target.v4_angle,Gimbal_motor[motor4].Data.angle);
	//云台速度关 联PID
	Gimbal_Info->SendValue[0]=f_PID_Calculate(&Gimbal_PID_v1,Angle_Gimbal_A1_PID.Output ,Gimbal_motor[motor1].Data.velocity);
	Gimbal_Info->SendValue[1]=f_PID_Calculate(&Gimbal_PID_v2,Angle_Gimbal_A2_PID.Output ,Gimbal_motor[motor2].Data.velocity);
	Gimbal_Info->SendValue[2]=f_PID_Calculate(&Gimbal_PID_v3,Angle_Gimbal_A3_PID.Output ,Gimbal_motor[motor3].Data.velocity);
  Gimbal_Info->SendValue[3]=f_PID_Calculate(&Gimbal_PID_v4,Angle_Gimbal_A4_PID.Output ,Gimbal_motor[motor4].Data.velocity);
	
}
