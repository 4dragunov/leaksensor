/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <assert.h>
#include "stm32wlxx_hal.h"
#include "stm32wlxx.h"
#include "stm32wlxx_ll_lpuart.h"
#include "stm32wlxx_ll_rcc.h"
#include "utilities.h"
#include "board.h"
#include "sysIrqHandlers.h"
#include "uart-board.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "modbus.h"

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
#define TX_BUFFER_RETRY_COUNT                       10
#define UART_COUNT 3



static UART_HandleTypeDef UartHandle[UART_COUNT];

const USART_TypeDef *UsartTypeDefs[UART_COUNT] = {LPUART1, USART1, USART2};
const IRQn_Type UartIRQ[UART_COUNT] = {LPUART1_IRQn, USART1_IRQn, USART2_IRQn};
Uart_t *UartsRegistered[UART_COUNT]={0};

uint8_t RxData[UART_COUNT] = {0};
uint8_t TxData[UART_COUNT] = {0};


UartId_t IdByHandle(const UART_HandleTypeDef *handle){
	for(int i = (int)LPUART_1; i < (int)UART_NONE; i++)
	{
		if(&UartHandle[i] == handle) {
			return (UartId_t) i;
		}
	}
	return UART_NONE;
}

void UartMcuInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx, PinNames de, PinConfigs txPinMode )
{
    obj->UartId = uartId;
    obj->handle = &UartHandle[uartId];
    uint32_t alt = 0;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
    if( uartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbInit( obj, uartId, NC, NC, NC );
#endif
    }
    else
    {

        switch(obj->UartId) {
        	case USART_1: {
        		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        		PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
        		__HAL_RCC_USART1_FORCE_RESET( );
        		__HAL_RCC_USART1_RELEASE_RESET( );
        		__HAL_RCC_USART1_CLK_ENABLE( );
        		alt = GPIO_AF7_USART1;
        	} break;
        	case USART_2: {
        		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        		PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
        		__HAL_RCC_USART2_FORCE_RESET( );
        		__HAL_RCC_USART2_RELEASE_RESET( );
        		__HAL_RCC_USART2_CLK_ENABLE( );
        		alt = GPIO_AF7_USART2;
        	} break;
        	case LPUART_1: {
        		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
        		PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
        		__HAL_RCC_LPUART1_FORCE_RESET( );
        		__HAL_RCC_LPUART1_RELEASE_RESET( );
        		__HAL_RCC_LPUART1_CLK_ENABLE( );
        		alt = GPIO_AF8_LPUART1;
        	} break;
        	default:{

        	}
        };
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
        GpioInit( &obj->Tx, tx, PIN_ALTERNATE_FCT, txPinMode, (txPinMode == PIN_OPEN_DRAIN)? PIN_PULL_UP : PIN_NO_PULL, alt);
        GpioInit( &obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, alt );
        GpioInit( &obj->De, de, PIN_ALTERNATE_FCT, PIN_PUSH_PULL,  PIN_NO_PULL, alt );
   }
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, UartBusMode_t busmode, FifoMode_t fifo, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
	DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbConfig( obj, mode, baudrate, wordLength, stopBits, parity, flowCtrl );
#endif
    }
    else
    {
    	assert_param(obj->UartId >= LPUART_1 && obj->UartId <= USART_2);
        UartHandle[obj->UartId].Instance = (USART_TypeDef*)UsartTypeDefs[obj->UartId];
        UartHandle[obj->UartId].Init.BaudRate = baudrate;
        obj->fifo = fifo;

        if( mode == TX_ONLY )
        {
            if( obj->FifoTx.Data == NULL )
            {
                assert_param( LMN_STATUS_ERROR );
            }
            UartHandle[obj->UartId].Init.Mode = UART_MODE_TX;
        }
        else if( mode == RX_ONLY )
        {
            if( obj->FifoRx.Data == NULL )
            {
                assert_param( LMN_STATUS_ERROR );
            }
            UartHandle[obj->UartId].Init.Mode = UART_MODE_RX;
        }
        else if( mode == RX_TX )
        {
            if( ( obj->FifoTx.Data == NULL ) || ( obj->FifoRx.Data == NULL ) )
            {
                assert_param( LMN_STATUS_ERROR );
            }
            UartHandle[obj->UartId].Init.Mode = UART_MODE_TX_RX;
        }
        else
        {
            assert_param( LMN_STATUS_ERROR );
        }

        if( wordLength == UART_8_BIT )
        {
            UartHandle[obj->UartId].Init.WordLength = UART_WORDLENGTH_8B;
        }
        else if( wordLength == UART_9_BIT )
        {
            UartHandle[obj->UartId].Init.WordLength = UART_WORDLENGTH_9B;
        }

        switch( stopBits )
        {
        case UART_2_STOP_BIT:
            UartHandle[obj->UartId].Init.StopBits = UART_STOPBITS_2;
            break;
        case UART_1_STOP_BIT:
        default:
            UartHandle[obj->UartId].Init.StopBits = UART_STOPBITS_1;
            break;
        }

        if( parity == NO_PARITY )
        {
            UartHandle[obj->UartId].Init.Parity = UART_PARITY_NONE;
        }
        else if( parity == EVEN_PARITY )
        {
            UartHandle[obj->UartId].Init.Parity = UART_PARITY_EVEN;
        }
        else
        {
            UartHandle[obj->UartId].Init.Parity = UART_PARITY_ODD;
        }

        if( flowCtrl == NO_FLOW_CTRL )
        {
            UartHandle[obj->UartId].Init.HwFlowCtl = UART_HWCONTROL_NONE;
        }
        else if( flowCtrl == RTS_FLOW_CTRL )
        {
            UartHandle[obj->UartId].Init.HwFlowCtl = UART_HWCONTROL_RTS;
        }
        else if( flowCtrl == CTS_FLOW_CTRL )
        {
            UartHandle[obj->UartId].Init.HwFlowCtl = UART_HWCONTROL_CTS;
        }
        else if( flowCtrl == RTS_CTS_FLOW_CTRL )
        {
            UartHandle[obj->UartId].Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
        }

        UartHandle[obj->UartId].Init.OverSampling = UART_OVERSAMPLING_16;

        UartHandle[obj->UartId].Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        UartHandle[obj->UartId].Init.ClockPrescaler = UART_PRESCALER_DIV16;
        UartHandle[obj->UartId].AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        UartHandle[obj->UartId].FifoMode = UART_FIFOMODE_DISABLE;
        switch(busmode){
			case UART: {
				if( HAL_UART_Init( &UartHandle[obj->UartId] ) != HAL_OK )
				{
					assert_param( LMN_STATUS_ERROR );
				}
			}break;
			case RS485:{
				 if (HAL_RS485Ex_Init(&UartHandle[obj->UartId], UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK){
					 assert_param( LMN_STATUS_ERROR );
				 }
			}
			break;
			default:{
				assert_param( LMN_STATUS_ERROR );
			}
        }
        if (HAL_UARTEx_SetTxFifoThreshold(&UartHandle[obj->UartId], UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
        {
        	assert_param( LMN_STATUS_ERROR );
        }

        if (HAL_UARTEx_SetRxFifoThreshold(&UartHandle[obj->UartId], UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
        {
        	assert_param( LMN_STATUS_ERROR );
        }

        if (HAL_UARTEx_DisableFifoMode(&UartHandle[obj->UartId]) != HAL_OK)
        {
        	assert_param( LMN_STATUS_ERROR );
        }

        UartsRegistered[obj->UartId] = obj;

        HAL_NVIC_SetPriority(UartIRQ[obj->UartId], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0 );
        HAL_NVIC_EnableIRQ(UartIRQ[obj->UartId]);

        __HAL_UART_SEND_REQ( &UartHandle[obj->UartId], UART_RXDATA_FLUSH_REQUEST);
        HAL_UART_Receive_IT( &UartHandle[obj->UartId], &RxData[obj->UartId], 1);

    }
}

void UartMcuDeInit( Uart_t *obj )
{
	DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbDeInit( obj );
#endif
    }
    else
    {
    	HAL_NVIC_DisableIRQ(UartIRQ[obj->UartId]);
    	switch(obj->UartId) {
			case USART_1: {

				__HAL_RCC_USART1_FORCE_RESET( );
				__HAL_RCC_USART1_RELEASE_RESET( );
				__HAL_RCC_USART1_CLK_DISABLE( );
			} break;
			case USART_2: {

				__HAL_RCC_USART2_FORCE_RESET( );
				__HAL_RCC_USART2_RELEASE_RESET( );
				__HAL_RCC_USART2_CLK_DISABLE( );

			} break;
			case LPUART_1: {

				__HAL_RCC_LPUART1_FORCE_RESET( );
				__HAL_RCC_LPUART1_RELEASE_RESET( );
				__HAL_RCC_LPUART1_CLK_DISABLE( );
			} break;
			default:{

			}
    	};
        UartsRegistered[obj->UartId] = NULL;

        GpioInit( &obj->Tx, obj->Tx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &obj->Rx, obj->Rx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        if(obj->De.pin!=NC)
        	GpioInit( &obj->De, obj->De.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data , uint32_t timeout )
{
	//DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbPutChar( obj, data );
#else
        return 255; // Not supported
#endif
    }
    else
    {
#ifndef USART_SUPPORT_RTOS
        if(obj->fifo == SYNC) {
        	auto ret = HAL_UART_Transmit((UART_HandleTypeDef*)obj->handle, &data, 1, timeout);
        	DBG("result:%i\n",ret);
        	return ret;
        }
        else
#endif
        if(!IsFifoFull( &obj->FifoTx ))
        {
        	CRITICAL_SECTION_BEGIN( );
            FifoPush( &obj->FifoTx, data );
            // Trig UART Tx interrupt to start sending the FIFO contents.
            __HAL_UART_ENABLE_IT( &UartHandle[obj->UartId], UART_IT_TC );
            CRITICAL_SECTION_END( );
#ifdef USART_SUPPORT_RTOS
         if(obj->fifo == SYNC) {
        	 auto ret = osSemaphoreAcquire(obj->txSem, timeout) ; // OK
        	// DBG("txSem result:%i: %s\n", ret, ret==osOK? "ok": "fail");
        	 return ret != osOK;
         }
         else
         {
        	// DBG("ok\n");
        	 return 0;
         }
#endif
        }
     DBG("%s fail\n",__FUNCTION__);
     return 1; // Busy
    }
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data , uint32_t timeout )
{
	//DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbGetChar( obj, data );
#else
        return 255; // Not supported
#endif
    }
    else
    {
#ifndef USART_SUPPORT_RTOS
    	if(obj->fifo == SYNC){
    		auto ret = HAL_UART_Receive((UART_HandleTypeDef*)obj->handle, data, 1, timeout);
    		DBG("result:%i\n",ret);
    	    return ret;
    	}
#else
        if(osSemaphoreAcquire(obj->rxSem, timeout) == osOK)
        {
        	CRITICAL_SECTION_BEGIN( );
            *data = FifoPop( &obj->FifoRx );
            CRITICAL_SECTION_END( );
            //DBG("ok\n");
            return 0;
        }
        DBG("%s fail\n",__FUNCTION__);
        return 1;
#endif
    }
}

uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size , uint32_t timeout )
{
	DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbPutBuffer( obj, buffer, size );
#else
        return 255; // Not supported
#endif
    }
    else
    {
        uint8_t retryCount;
        uint16_t i;

        for( i = 0; i < size; i++ )
        {
            retryCount = 0;
            while( UartPutChar( obj, buffer[i], timeout) != 0 )
            {
                retryCount++;

                // Exit if something goes terribly wrong
                if( retryCount > TX_BUFFER_RETRY_COUNT )
                {
                    return 1; // Error
                }
            }
        }
        return 0; // OK
    }
}

uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes , uint32_t timeout )
{
    uint16_t localSize = 0;
    DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
    while( localSize < size )
    {
        if( UartGetChar( obj, buffer + localSize ,timeout) == 0 )
        {
            localSize++;
        }
        else
        {
            break;
        }
    }

    *nbReadBytes = localSize;

    if( localSize == 0 )
    {
        return 1; // Empty
    }
    return 0; // OK
}

bool UartMcuWaitReady(Uart_t *obj, uint32_t millisec){
	DBG("%s uart %i\n",__FUNCTION__, obj->UartId);
	do {
	  if ((HAL_UART_GetState(&UartHandle[obj->UartId]) & HAL_UART_STATE_READY) == HAL_UART_STATE_READY)
		  return true;
	  else
	    osDelay(1);
    }while (millisec--);
	return false;
}


uint32_t UartMcuGetBaudrate(const Uart_t *obj)
{
	//DBG("%s uart %i\n",__FUNCTION__, obj->UartId);
	return UartHandle[obj->UartId].Init.BaudRate;
}


bool UartMcuSetBaudrate(const Uart_t *obj, uint32_t newBaudRate)
{
	UART_HandleTypeDef *huart = &UartHandle[obj->UartId];

	taskENTER_CRITICAL();
	CLEAR_BIT(huart->Instance->CR1, USART_CR1_UE);
	huart->Init.BaudRate = newBaudRate;
	if (IS_LPUART_INSTANCE(huart->Instance)) {
		uint32_t clock_rate = LL_RCC_GetLPUARTClockFreq(LL_RCC_LPUART1_CLKSOURCE);
		LL_LPUART_SetPrescaler(huart->Instance, LL_LPUART_PRESCALER_DIV16);
		LL_LPUART_SetBaudRate(huart->Instance,
					  clock_rate,
	#ifdef USART_PRESC_PRESCALER
					  LL_LPUART_PRESCALER_DIV16,
	#endif
					  newBaudRate);
	}else {
		UART_SetConfig(huart);
	}
	SET_BIT(huart->Instance->CR1, USART_CR1_UE);
	taskEXIT_CRITICAL();
	return true;
}

void UartMcuAbortReceive(const Uart_t *obj) {
	DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
	HAL_UART_AbortReceive_IT(&UartHandle[obj->UartId]);
}

void UartMcuEnableTransmitter(const Uart_t *obj)
{
	DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
	HAL_HalfDuplex_EnableTransmitter(&UartHandle[obj->UartId]);
}

void UartMcuEnableReciever(const Uart_t *obj)
{
	DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
	HAL_HalfDuplex_EnableReceiver(&UartHandle[obj->UartId]);
}

void UartMcuEnableRxTx(const Uart_t *obj)
{
	UART_HandleTypeDef *huart = &UartHandle[obj->UartId];

	taskENTER_CRITICAL();
	huart->Instance->CR1 |= USART_CR1_TE;
	huart->Instance->CR1 |= USART_CR1_RE;
	taskEXIT_CRITICAL();
}

void UartMcuDisableRxTx(const Uart_t *obj)
{
	UART_HandleTypeDef *huart = &UartHandle[obj->UartId];
	taskENTER_CRITICAL();
	huart->Instance->CR1 &= ~(USART_CR1_TE);
	huart->Instance->CR1 &= ~(USART_CR1_RE);
	taskEXIT_CRITICAL();
}

void UartMcuSetState(const Uart_t *obj, bool enabled)
{
	UART_HandleTypeDef *huart = &UartHandle[obj->UartId];

	if(enabled)
		__HAL_UART_ENABLE(huart);
	else
		__HAL_UART_DISABLE(huart);
}


bool UartMcuLastByteSendOut(const Uart_t *obj, uint32_t timeout){
	DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
#if defined(STM32H7)  || defined(STM32F3) || defined(STM32L4) || defined(STM32L082xx) || defined(STM32F7) || defined(STM32WL) || defined(STM32WB) || defined(STM32G070xx) || defined(STM32F0) || defined(STM32G431xx) || defined(STM32H5)
	while(timeout-- && (UartHandle[obj->UartId].Instance->ISR & USART_ISR_TC) ==0 )
#else
	while(timeout-- && (UartHandle[obj->UartId].Instance->SR & USART_SR_TC) ==0 )
#endif
	{
		osDelay(1);
	}

	return timeout;
}

void ModBus_ErrorCallback(Uart_t *huart);
void ModBus_RxCpltCallback(Uart_t *huart);
void ModBus_TxCpltCallback(Uart_t *huart);

extern "C" void HAL_UART_TxCpltCallback( UART_HandleTypeDef *handle )
{
	UartId_t uart = IdByHandle(handle);
	switch(uart){
		case LPUART_1:{
			//DBG("%s %i\n",__FUNCTION__, uart);
			if( !IsFifoEmpty( &UartsRegistered[uart]->FifoTx ) )
			{
				TxData[uart] = FifoPop( &UartsRegistered[uart]->FifoTx );
				//  Write one byte to the transmit data register
				HAL_UART_Transmit_IT( &UartHandle[uart], &TxData[uart], 1 );
			}
			else{
				if( UartsRegistered[uart]->IrqNotify != NULL )
				{
					UartsRegistered[uart]->IrqNotify(UartsRegistered[uart], UART_NOTIFY_TX );
				}
		#ifdef USART_SUPPORT_RTOS
				osSemaphoreRelease(UartsRegistered[uart]->txSem);
		#endif
			}

		}break;
		case USART_1:{
			ModBus_TxCpltCallback(UartsRegistered[uart]);
		}break;
		default:{

		}
	}
}

extern "C" void HAL_UART_RxCpltCallback( UART_HandleTypeDef *handle )
{

	UartId_t uart = IdByHandle(handle);
	switch(uart){
		case LPUART_1:{
		//DBG("%s %i\n",__FUNCTION__, uart);
		if( !IsFifoFull( &UartsRegistered[uart]->FifoRx ) )
		{
			// Read one byte from the receive data register
			FifoPush( &UartsRegistered[uart]->FifoRx, RxData[uart] );
		}

		if( UartsRegistered[uart]->IrqNotify != NULL )
		{
			UartsRegistered[uart]->IrqNotify(UartsRegistered[uart], UART_NOTIFY_RX );
		}
	#ifdef USART_SUPPORT_RTOS
		osSemaphoreRelease(UartsRegistered[uart]->rxSem);
	#endif

		HAL_UART_Receive_IT( &UartHandle[uart], &RxData[uart], 1 );
		}break;
		case USART_1:{

		}break;

		default:{
			ModBus_RxCpltCallback(UartsRegistered[uart]);
		}
	}
}

extern "C" void HAL_UART_ErrorCallback( UART_HandleTypeDef *handle )
{
	UartId_t uart = IdByHandle(handle);
	switch(uart){
		case LPUART_1:{
			DBG("%s %02x\n",__FUNCTION__, handle->ErrorCode);
			if(handle->ErrorCode != HAL_UART_ERROR_NONE) {
				switch(handle->ErrorCode ){
					case HAL_UART_ERROR_NONE:{

					}break;
					case HAL_UART_ERROR_PE:{

					}break;
					case HAL_UART_ERROR_NE:{

					}break;
					case HAL_UART_ERROR_FE:{
						 __HAL_UART_CLEAR_IT(handle, UART_IT_FE);
					}break;
					case HAL_UART_ERROR_ORE:{

					}break;
					case HAL_UART_ERROR_DMA:{

					}break;
					case HAL_UART_ERROR_RTO:{
						HAL_UART_Receive_IT(handle, &RxData[uart], 1 );
					}break;
					default:{
						assert(0);
					}
				}
			}
		}
		break;
		case USART_1:{
			//ModBus_ErrorCallback(UartsRegistered[uart]);
		}break;
		default:{
		}
	}
}

void USART_IRQHandler(const UartId_t usart )
{
    HAL_UART_IRQHandler( &UartHandle[usart] );
}

extern "C" void USART1_IRQHandler( void )
{
	USART_IRQHandler(USART_1);
}

extern "C" void USART2_IRQHandler( void )
{
	USART_IRQHandler(USART_2);
}

extern "C" void LPUART1_IRQHandler( void )
{
    USART_IRQHandler(LPUART_1);
}

extern "C" void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
	DBG("HAL_UART_AbortReceiveCpltCallback \n");
}

extern "C" void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
	DBG("HAL_UART_AbortTransmitCpltCallback \n");
}

extern "C" void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
	DBG("HAL_UART_AbortCpltCallback \n");
}

const char* stateUART(HAL_UART_StateTypeDef State)
{
	switch(State)
	{
	case HAL_UART_STATE_RESET: 		return "HAL_UART_STATE_RESET";
	case HAL_UART_STATE_READY: 		return "HAL_UART_STATE_READY";
	case HAL_UART_STATE_BUSY: 		return "HAL_UART_STATE_BUSY";
	case HAL_UART_STATE_BUSY_TX: 	return "HAL_UART_STATE_BUSY_TX";
	case HAL_UART_STATE_BUSY_RX: 	return "HAL_UART_STATE_BUSY_RX";
	case HAL_UART_STATE_BUSY_TX_RX: return "HAL_UART_STATE_BUSY_TX_RX";
	case HAL_UART_STATE_TIMEOUT: 	return "HAL_UART_STATE_TIMEOUT";
	case HAL_UART_STATE_ERROR: 		return "HAL_UART_STATE_ERROR";
	default : 						return "?????";
	}
	return "????";
}

