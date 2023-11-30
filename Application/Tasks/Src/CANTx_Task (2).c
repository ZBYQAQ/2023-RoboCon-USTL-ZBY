#include "CANTx_Task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
//#include "INS_Task.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
void CAN1Tx_Task(void const * argument)
{
   
  for(;;)
  {
		ChassisTxFrame.data[0] = (uint8_t)( Chassis_Info.SendValue[0] >>8) ;
		ChassisTxFrame.data[1] = (uint8_t)( Chassis_Info.SendValue[0] );
    ChassisTxFrame.data[2] = (uint8_t)( Chassis_Info.SendValue[1] >>8) ;
		ChassisTxFrame.data[3] = (uint8_t)( Chassis_Info.SendValue[1] );
		ChassisTxFrame.data[4] = (uint8_t)( Chassis_Info.SendValue[2] >>8) ;
		ChassisTxFrame.data[5] = (uint8_t)( Chassis_Info.SendValue[2] );
 
		USER_CAN_TxMessage(&ChassisTxFrame);
		osDelay(1);
  }
 
}

void CAN2Tx_Task(void const *arguement)
{
    
  for(;;)
  {
		GimbalTxFrame.data[0] = (uint8_t)( Gimbal_Info.SendValue[0] >>8) ;
		GimbalTxFrame.data[1] = (uint8_t)( Gimbal_Info.SendValue[0] );
    GimbalTxFrame.data[2] = (uint8_t)( Gimbal_Info.SendValue[1] >>8) ;
	  GimbalTxFrame.data[3] = (uint8_t)( Gimbal_Info.SendValue[1] );
	  GimbalTxFrame.data[4] = (uint8_t)( Gimbal_Info.SendValue[2] >>8) ;
		GimbalTxFrame.data[5] = (uint8_t)( Gimbal_Info.SendValue[2] );
		GimbalTxFrame.data[6] = (uint8_t)( Gimbal_Info.SendValue[3] >>8) ;
		GimbalTxFrame.data[7] = (uint8_t)( Gimbal_Info.SendValue[3] );
		USER_CAN_TxMessage(&GimbalTxFrame);
		osDelay(1);
  }
  

};	
