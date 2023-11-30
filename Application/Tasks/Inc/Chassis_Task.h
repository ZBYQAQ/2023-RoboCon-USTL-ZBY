#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "stdint.h"
#include "pid.h"
typedef struct
{
	  float vx,vy,vw;
   	struct{

		 int16_t A_velocity;
	   int16_t B_velocity;
		 int16_t C_velocity;
			
	 }Target;
	  int16_t SendValue[8];
}Chassis_Info_Typedef;
extern Chassis_Info_Typedef Chassis_Info;




#endif
