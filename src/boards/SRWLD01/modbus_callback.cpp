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

extern "C" void ModBus_TxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Modbus RTU TX callback BEGIN */
	bool processed = false;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	for (int i = 0; i < numberHandlers; i++ )
	{
	   	if (mHandlers[i]->port() == huart  )
	   	{
	   		processed = true;
	   		// notify the end of TX
	   		xTaskNotifyFromISR(mHandlers[i]->taskHandle(), 0, eNoAction, &xHigherPriorityTaskWoken);
	   		break;
	   	}
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	if(!processed) {

	}
}



/**
 * @brief
 * This is the callback for HAL interrupt of UART RX
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */
extern "C"  void ModBus_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	bool processed = false;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Modbus RTU RX callback BEGIN */
    for (int i = 0; i < numberHandlers; i++ )
    {
    	if (mHandlers[i]->port() == UartHandle  )
    	{
    		processed = true;
    		mHandlers[i]->busBuffer().Add(mHandlers[i]->dataRX());
    		HAL_UART_Receive_IT((UART_HandleTypeDef *)mHandlers[i]->port(), &mHandlers[i]->dataRX(), 1);
    		xTimerResetFromISR(mHandlers[i]->t35TimerHandle(), &xHigherPriorityTaskWoken);

    		break;
    	}
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    if(!processed) {

    }
}



/*
 * DMA requires to handle callbacks for special communication modes of the HAL
 * It also has to handle eventual errors including extra steps that are not automatically
 * handled by the HAL
 * */


extern "C"  void ModBus_ErrorCallback(UART_HandleTypeDef *huart)
{
	bool processed = false;

	for (int i = 0; i < numberHandlers; i++ )
	{
    	if (mHandlers[i]->port() == huart  )
    	{
    		processed = true;
    		while(HAL_UARTEx_ReceiveToIdle_DMA((UART_HandleTypeDef*)mHandlers[i]->port(), mHandlers[i]->busBuffer().buffer(), MAX_BUFFER) != HAL_OK)
    		{
    			HAL_UART_DMAStop((UART_HandleTypeDef*)mHandlers[i]->port());
    		}
			__HAL_DMA_DISABLE_IT(((UART_HandleTypeDef*)mHandlers[i]->port())->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt
    		break;
    	}
   }
   if(!processed) {

   }
}


extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	bool processed = false;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	/* Modbus RTU RX callback BEGIN */
	for (int i = 0; i < numberHandlers; i++ )
	{
		if (mHandlers[i]->port() == huart  )
		{
			processed = true;
			if(Size) //check if we have received any byte
			{
				mHandlers[i]->busBuffer().available(Size);
				mHandlers[i]->busBuffer().overflow(false);

				while(HAL_UARTEx_ReceiveToIdle_DMA((UART_HandleTypeDef*)mHandlers[i]->port(), mHandlers[i]->busBuffer().buffer(), MAX_BUFFER) != HAL_OK)
				{
					HAL_UART_DMAStop((UART_HandleTypeDef*)mHandlers[i]->port());

				}
				__HAL_DMA_DISABLE_IT(((UART_HandleTypeDef*)mHandlers[i]->port())->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt

				xTaskNotifyFromISR(mHandlers[i]->taskHandle(), 0 , eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
			}
			break;
		}
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	if(!processed) {

	}
}


