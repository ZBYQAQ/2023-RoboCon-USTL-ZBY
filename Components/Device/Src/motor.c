//
// Created by YanYuanbin on 22-10-5.
//

#include "motor.h"
#define ABS(x) (((x) > 0) ? (x) : -(x))
DJI_MOTOR Chassis_motor[MOTOR_NUM]=
{
	[A] = {
			.CANx = CAN1,
			.type = _3508,
			.Data.StdId = 0x201,
		   .usage = B,
		 .Initlized =0,
	},
	[B] = {
			.CANx = CAN1,
			.type = _3508,
			.Data.StdId = 0x202,
	    .usage = B,
		 .Initlized =0,
	},
	[C] = {
			.CANx = CAN2,
			.type = _3508,
			.Data.StdId = 0x203,
		  .usage = C,
		   .Initlized =0,
	},
};
DJI_MOTOR Gimbal_motor [MOTOR_NUM_GIMBAL]=
{
    [motor1]={
		  .CANx = CAN2,
			.type = _3508,
			.Data.StdId = 0x201,
			 .usage2 = motor1,
		 .Initlized =0,
		},
		[motor2]={
		.CANx = CAN2,
			.type = _3508,
			.Data.StdId = 0x202,
			.usage2 = motor2,
		 .Initlized =0,
		},
		[motor3]={
		.CANx = CAN2,
			.type = _3508,
			.Data.StdId = 0x203,
			.usage2 = motor3,
		 .Initlized =0,
		},
		[motor4]={
		.CANx = CAN2,
			.type = _3508,
			.Data.StdId = 0x204,
			.usage2 = motor4,
		 .Initlized =0,
		},
		

};
	
static float f_loop_constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }
  
  float len = maxValue - minValue;    

  if (Input > maxValue)
  {
      do{
          Input -= len;
      }while (Input > maxValue);
  }
  else if (Input < minValue)
  {
      do{
          Input += len;
      }while (Input < minValue);
  }
  return Input;
}

//static float encoder2Angle(MOTOR_USAGE usage,int16_t act_Encoder,float torque_Ratio)
//{
//    static int32_t angle_Last[MOTOR_NUM];
//    static int32_t result_Encoder[MOTOR_NUM];
//    int res1,res2;		
//	
//    if(act_Encoder<angle_Last[usage])
//		{
//        res1 = act_Encoder-angle_Last[usage]+8192;
//        res2 = act_Encoder-angle_Last[usage];
//    }else
//		{
//        res1=act_Encoder-angle_Last[usage]-8192;
//        res2=act_Encoder-angle_Last[usage];
//    }
//		//????????
//    angle_Last[usage]=act_Encoder;
//		
//    ABS(res1)<ABS(res2) ? (result_Encoder[usage]+=res1) : (result_Encoder[usage]+=res2);
//		
//    if(usage == _3508){
//        ABS(result_Encoder[usage]) > 8192*torque_Ratio ? (result_Encoder[usage]-result_Encoder[usage]/ABS(result_Encoder[usage])*8192*torque_Ratio): result_Encoder[usage];
//    }
//    return (float)result_Encoder[usage]/(8192.f*torque_Ratio)*360.0f;
//}
static float encoder2Anglesum(MOTOR_USAGE usage,int16_t act_Encoder,float torque_Ratio)
{
    static int32_t angle_Last[MOTOR_NUM];
    static int32_t result_Encoder[MOTOR_NUM];
    int res1,res2;		
	
    if(act_Encoder<angle_Last[usage])
		{
        res1 = act_Encoder-angle_Last[usage]+8192;
        res2 = act_Encoder-angle_Last[usage];
    }else
		{
        res1=act_Encoder-angle_Last[usage]-8192;
        res2=act_Encoder-angle_Last[usage];
    }
		//????????
    angle_Last[usage]=act_Encoder;
		
    ABS(res1)<ABS(res2) ? (result_Encoder[usage]+=res1) : (result_Encoder[usage]+=res2);
		
    if(usage == _3508){
        ABS(result_Encoder[usage]) > 8192*torque_Ratio ? (result_Encoder[usage]-result_Encoder[usage]/ABS(result_Encoder[usage])*8192*torque_Ratio): result_Encoder[usage];
    }
    return f_loop_constrain((float)result_Encoder[usage]/(8192.f*torque_Ratio)*360.0f,-180.f,180.f);
	}

static float encoder_to_anglesum(DJI_MOTOR *Info,float torque_ratio,uint16_t MAXencoder)
{
  float res1 = 0,res2 =0;
  
  if(Info == NULL) return 0;
  
  /* Judge the motor Initlized */
  if(Info->Initlized != true)
  {
    /* update the last encoder */
    Info->Data.last_encoder = Info->Data.encoder;

    /* reset the angle */
    Info->Data.angle = 0;

    /* Set the init flag */
    Info->Initlized = true;
  }
  
  /* get the possiable min encoder err */
  if(Info->Data.encoder < Info->Data.last_encoder)
  {
      res1 = Info->Data.encoder - Info->Data.last_encoder + MAXencoder;
  }
  else if(Info->Data.encoder > Info->Data.last_encoder)
  {
      res1 = Info->Data.encoder - Info->Data.last_encoder - MAXencoder;
  }
  res2 = Info->Data.encoder - Info->Data.last_encoder;
  
  /* update the last encoder */
  Info->Data.last_encoder = Info->Data.encoder;
  
  /* transforms the encoder data to tolangle */
	if(fabsf(res1) > fabsf(res2))
	{
		Info->Data.angle += (float)res2/(MAXencoder*torque_ratio)*360.f;
	}
	else
	{
		Info->Data.angle += (float)res1/(MAXencoder*torque_ratio)*360.f;
	}
  
  return Info->Data.angle;
}
float encoder_to_angle(DJI_MOTOR *Info,float torque_ratio,uint16_t MAXencoder)
{	
  float encoder_err = 0.f;
  
  /* check the motor init */
  if(Info->Initlized != true)
  {
    /* update the last encoder */
    Info->Data.last_encoder = Info->Data.encoder;

    /* reset the angle */
    Info->Data.angle = 0;

    /* config the init flag */
    Info->Initlized = true;
  }
  
  encoder_err = Info->Data.encoder - Info->Data.last_encoder;
  
  /* 0 -> MAXencoder */		
  if(encoder_err > MAXencoder*0.5f)
  {
    Info->Data.angle += (float)(encoder_err - MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  /* MAXencoder-> 0 */		
  else if(encoder_err < -MAXencoder*0.5f)
  {
    Info->Data.angle += (float)(encoder_err + MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  else
  {
    Info->Data.angle += (float)(encoder_err)/(MAXencoder*torque_ratio)*360.f;
  }
  
  /* update the last encoder */
  Info->Data.last_encoder = Info->Data.encoder;
  
  /* loop constrain */
  f_loop_constrain(Info->Data.angle,-180.f,180.f);

  return Info->Data.angle;
}
void get_Motor_Data(uint32_t *canId, uint8_t *rxBuf,DJI_MOTOR* DJI_Motor)
{
    if(*canId !=DJI_Motor->Data.StdId) return;
    //????
    DJI_Motor->Data.temperature = rxBuf[6];
    DJI_Motor->Data.encoder  = ((int16_t)rxBuf[0]<<8 | (int16_t)rxBuf[1]);
    DJI_Motor->Data.velocity = ((int16_t)rxBuf[2]<<8 | (int16_t)rxBuf[3]);
    DJI_Motor->Data.current  = ((int16_t)rxBuf[4]<<8 | (int16_t)rxBuf[5]);

    //??????????
    switch (DJI_Motor->type)
    {
        case _6020:
            DJI_Motor->Data.angle = encoder_to_anglesum(DJI_Motor,1.0f,8192);
            break;

        case _2006:
           DJI_Motor->Data.angle = encoder_to_angle(DJI_Motor,36.0f,8192);
            break;

        case _3508:
            DJI_Motor->Data.angle = encoder_to_angle(DJI_Motor,3591.f/187.f,8192);
            break;
				
				case _3510:
            DJI_Motor->Data.angle = encoder_to_angle(DJI_Motor,DJI_Motor->Data.encoder,19.f);
				break;

        default:
					return;
				break;
    }
}


