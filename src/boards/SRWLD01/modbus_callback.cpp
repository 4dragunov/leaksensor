/*
 * UARTCallback.c
 *
 *  Created on: Nov 27, 2024
 *      Author: Andrey Belyakov <andrei.belyakov@simbirsoft.com>
 */
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "modbus.h"

using namespace ModBus;

/**
 * @brief
 * This is the callback for HAL interrupts of UART TX used by Modbus library.
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */
void ModBus_IrqNotify(Uart_t *uart, UartNotifyId_t type) {
	if(type == UART_NOTIFY_TX)
		ModBus_TxCpltCallback(uart);
	else
		ModBus_RxCpltCallback(uart);
}

void ModBus_TxCpltCallback(Uart_t *huart)
{
	/* Modbus RTU TX callback BEGIN */

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	for (int i = 0; i < numberHandlers; i++ )
	{
	   	if (const_cast<Modbus*>(mHandlers[i])->port() == huart->handle  )
	   	{
	   		// notify the end of TX
	   		xTaskNotifyFromISR(const_cast<Modbus*>(mHandlers[i])->taskHandle(), 0, eNoAction, &xHigherPriorityTaskWoken);
	   		break;
	   	}
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * @brief
 * This is the callback for HAL interrupt of UART RX
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */
void ModBus_RxCpltCallback(Uart_t *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Modbus RTU RX callback BEGIN */
    for (int i = 0; i < numberHandlers; i++ )
    {
    	if (const_cast<Modbus*>(mHandlers[i])->port() == huart->handle  )
    	{
    		while(!UartGetChar(const_cast<Modbus*>(mHandlers[i])->port(), &const_cast<Modbus*>(mHandlers[i])->dataRX(), T35))
    			const_cast<Modbus*>(mHandlers[i])->busBuffer().Add(const_cast<Modbus*>(mHandlers[i])->dataRX());

    		HAL_UART_Receive_IT(static_cast<UART_HandleTypeDef*>(huart->handle), &const_cast<Modbus*>(mHandlers[i])->dataRX(), 1);

    		osTimerStart(const_cast<Modbus*>(mHandlers[i])->t35TimerHandle(), T35);
    		break;
    	}
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

#if  ENABLE_USART_DMA ==  1

/*
 * DMA requires to handle callbacks for special communication modes of the HAL
 * It also has to handle eventual errors including extra steps that are not automatically
 * handled by the HAL
 * */


extern "C"  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	bool processed = false;

	for (int i = 0; i < numberHandlers; i++ )
	{
    	if (const_cast<Modbus*>(mHandlers[i])->port() == huart  )
    	{
    		processed = true;
    		while(HAL_UARTEx_ReceiveToIdle_DMA(huart, const_cast<Modbus*>(mHandlers[i])->busBuffer().buffer(), MAX_BUFFER) != HAL_OK)
    		{
    			HAL_UART_DMAStop(huart);
    		}
			__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt
    		break;
    	}
   }
}


extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	bool processed = false;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	/* Modbus RTU RX callback BEGIN */
	for (int i = 0; i < numberHandlers; i++ )
	{
		if (const_cast<Modbus*>(mHandlers[i])->port() == huart  )
		{
			processed = true;
			if(Size) //check if we have received any byte
			{
				const_cast<Modbus*>(mHandlers[i])->busBuffer().available(Size);
				const_cast<Modbus*>(mHandlers[i])->busBuffer().overflow(false);

				while(HAL_UARTEx_ReceiveToIdle_DMA(huart,  const_cast<Modbus*>(mHandlers[i])->busBuffer().buffer(), MAX_BUFFER) != HAL_OK)
				{
					HAL_UART_DMAStop(huart);

				}
				__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt

				xTaskNotifyFromISR(const_cast<Modbus*>(mHandlers[i])->taskHandle(), 0 , eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
			}
			break;
		}
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
#endif
