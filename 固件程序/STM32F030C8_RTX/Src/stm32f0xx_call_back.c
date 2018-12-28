//stm32中断回到函数
#include "stm32f0xx_call_back.h"
//#include "RTL.h"
#include "protocol.h"
#include "led.h"
#include "device.h"




//外部管脚中断
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SI4438_nIRQ_Pin) //无线IRQ中断，PF7
    {
        //UartSendBytes(USART1,(uint8_t*)"GPIO_PIN_6 EXIT \r\n",18);
        Si4438_Interrupt_Handler(&Wireless_Buf);
    }
}

/*
串口接收完成回调函数
*/
uint8_t frameLen = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    extern uint8_t LP_UartRec;
    if (huart->Instance == USART1) //如果是串口1
    {
		UpUart_RX_INT_Process(uart1Rec, &MAC_UartRec);   //烧录/读取设备MAC协议
		//按串口接收数据超时接收数据
		if (uart1RecBuff.rec_ok == 0)     
		{		
			uart1RecBuff.timeOut = 0;  //超时接收变量清空

			uart1RecBuff.buff[uart1RecBuff.cnt++] = uart1Rec;
			if(uart1RecBuff.cnt > UART_RECV_BUFF_SIZE)
			{
				uart1RecBuff.cnt =  UART_RECV_BUFF_SIZE;
			}	
		}	
    }
	if (huart->Instance == USART2) //如果是串口1
	{
		UpUart_RX_INT_Process(uart2Rec, &MAC_UartRec);   //烧录/读取设备MAC协议
		//按串口接收数据超时接收数据
		if (uart1RecBuff.rec_ok == 0)     
		{		
			uart1RecBuff.timeOut = 0;  //超时接收变量清空

			uart1RecBuff.buff[uart1RecBuff.cnt++] = uart2Rec;
			if(uart1RecBuff.cnt > UART_RECV_BUFF_SIZE)
			{
				uart1RecBuff.cnt =  UART_RECV_BUFF_SIZE;
			}	
		}	
		
	}
}

/*
定时器1溢出中断回调函数
*/
extern uint32_t SensorDataReadCnt;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uart1RecBuff.timeOut ++;	//串口接收超时计数
	SensorDataReadCnt ++;
}




