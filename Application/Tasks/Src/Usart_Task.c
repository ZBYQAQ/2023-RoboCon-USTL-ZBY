#include "cmsis_os.h"
#include "Usart_Task.h"
#include "usart.h"
#include "motor.h"
#include "INS_Task.h"
void usart_printf(const char *fmt,...)
{
   
    static va_list ap;
    static uint16_t len;
    static uint8_t tx_buf[256] = {0};
    va_start(ap, fmt);
    len = vsnprintf((char *)tx_buf,sizeof(tx_buf)+1 ,(char*)fmt, ap);

    va_end(ap);

    HAL_UART_Transmit_DMA(&huart1,(uint8_t*)tx_buf, len);
    
}
#define printf(title, fmt, args...) usart_printf("{"#title"}"fmt"\n", ##args)
void Usart_Task(void const * argument)
{
  /* USER CODE BEGIN Vision_Task */
//  TickType_t systick = 0;

	
  /* Infinite loop */
  for(;;)
  {  
		
//     printf(motor,"%d,%d",SendValue,-Gimbal_Motor[Pitch].Data.velocity);
   osDelay(1);

  }
  /* USER CODE END Vision_Task */
}
//------------------------------------------------------------------------------

