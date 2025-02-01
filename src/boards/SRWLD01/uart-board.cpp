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
#include "stm32f1xx.h"
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
#define UART_COUNT 5



static UART_HandleTypeDef UartHandle[UART_COUNT];

const USART_TypeDef *UsartTypeDefs[UART_COUNT] = {USART1, USART2, USART3, UART4, UART5};
const IRQn_Type UartIRQ[UART_COUNT] = {USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn};
Uart_t *UartsRegistered[UART_COUNT];

uint8_t RxData[UART_COUNT] = {0};
uint8_t TxData[UART_COUNT] = {0};




UartId_t IdByHandle(const UART_HandleTypeDef *handle){
	for(int i = (int)USART_1; i < (int)UART_NONE; i++)
	{
		if(&UartHandle[i] == handle) {
			return (UartId_t) i;
		}
	}
	return UART_NONE;
}

void UartMcuInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx )
{
    obj->UartId = uartId;
    obj->handle = &UartHandle[uartId];
    DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
    if( uartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbInit( obj, uartId, NC, NC );
#endif
    }
    else
    {

        switch(obj->UartId) {
        	case USART_1: {
        		__HAL_RCC_USART1_FORCE_RESET( );
        		__HAL_RCC_USART1_RELEASE_RESET( );
        		__HAL_RCC_USART1_CLK_ENABLE( );
        	} break;
        	case USART_2: {
        		__HAL_RCC_USART2_FORCE_RESET( );
        		__HAL_RCC_USART2_RELEASE_RESET( );
        		__HAL_RCC_USART2_CLK_ENABLE( );
        	} break;
        	case USART_3: {
        		__HAL_RCC_USART3_FORCE_RESET( );
        		__HAL_RCC_USART3_RELEASE_RESET( );
        		__HAL_RCC_USART3_CLK_ENABLE( );
        	} break;
        	case UART_4: {
        		__HAL_RCC_UART4_FORCE_RESET( );
        		__HAL_RCC_UART4_RELEASE_RESET( );
        		__HAL_RCC_UART4_CLK_ENABLE( );
        	} break;
        	case UART_5: {
        		__HAL_RCC_UART5_FORCE_RESET( );
        		__HAL_RCC_UART5_RELEASE_RESET( );
        		__HAL_RCC_UART5_CLK_ENABLE( );
        	} break;
        	default:{

        	}
        };
        GpioInit( &obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
        GpioInit( &obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
        switch(obj->UartId) {
        	case USART_1: {
        		if(tx == PB_6 && rx == PB_7){
        		    __HAL_AFIO_REMAP_USART1_ENABLE();
        		}
            } break;
        	case USART_2: {
        	    if(tx == PD_5 && rx == PD_6){
        	        __HAL_AFIO_REMAP_USART2_ENABLE();
        	    }
        	} break;
            default:{
            }
        };
   }
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, FifoMode_t fifo, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
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
    	assert_param(obj->UartId >= USART1 && obj->UartId <= UART5);
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

        if( HAL_UART_Init( &UartHandle[obj->UartId] ) != HAL_OK )
        {
            assert_param( LMN_STATUS_ERROR );
        }

        HAL_NVIC_SetPriority( UartIRQ[obj->UartId], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0 );
        HAL_NVIC_EnableIRQ( UartIRQ[obj->UartId] );

        UartsRegistered[obj->UartId] = obj;
        /* Enable the UART Data Register not empty Interrupt */
        HAL_UART_Receive_IT( &UartHandle[obj->UartId], &RxData[obj->UartId], 1 );
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
			case USART_3: {

				__HAL_RCC_USART3_FORCE_RESET( );
				__HAL_RCC_USART3_RELEASE_RESET( );
				__HAL_RCC_USART3_CLK_DISABLE( );
			} break;
			case UART_4: {

				__HAL_RCC_UART4_FORCE_RESET( );
				__HAL_RCC_UART4_RELEASE_RESET( );
				__HAL_RCC_UART4_CLK_DISABLE( );
			} break;
			case UART_5: {

				__HAL_RCC_UART5_FORCE_RESET( );
				__HAL_RCC_UART5_RELEASE_RESET( );
				__HAL_RCC_UART5_CLK_DISABLE( );
			} break;
			default:{

			}
    	};
        UartsRegistered[obj->UartId] = NULL;

        GpioInit( &obj->Tx, obj->Tx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &obj->Rx, obj->Rx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
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
        	 auto ret = osSemaphoreWait(obj->txSem, timeout) ; // OK
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
#endif
        if(osSemaphoreWait(obj->rxSem, timeout) == osOK)
        {
        	CRITICAL_SECTION_BEGIN( );
            *data = FifoPop( &obj->FifoRx );
            CRITICAL_SECTION_END( );
            //DBG("ok\n");
            return 0;
        }
        DBG("%s fail\n",__FUNCTION__);
        return 1;
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

bool UartMcuSetBaudrate(const Uart_t *obj, uint32_t baudrate)
{

	//DBG("%s uart:%i to %li\n",__FUNCTION__, obj->UartId, baudrate);
	//__HAL_UART_DISABLE(&UartHandle[obj->UartId]);
	uint32_t bus_clk = (obj->UartId == USART_1)? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
	auto brr = UART_BRR_SAMPLING16(bus_clk, baudrate);
	//DBG("uart:%i old brr %li new %li\n",obj->UartId, UartHandle[obj->UartId].Instance->BRR, brr);
	UartHandle[obj->UartId].Instance->BRR = brr;
	//__HAL_UART_ENABLE(&UartHandle[obj->UartId]);

	return  true;
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


bool UartMcuLastByteSendOut(const Uart_t *obj, uint32_t timeout){
	DBG("%s uart:%i\n",__FUNCTION__, obj->UartId);
#if defined(STM32H7)  || defined(STM32F3) || defined(STM32L4) || defined(STM32L082xx) || defined(STM32F7) || defined(STM32WB) || defined(STM32G070xx) || defined(STM32F0) || defined(STM32G431xx) || defined(STM32H5)
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
}

extern "C" void HAL_UART_RxCpltCallback( UART_HandleTypeDef *handle )
{

	UartId_t uart = IdByHandle(handle);
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
}

extern "C" void HAL_UART_ErrorCallback( UART_HandleTypeDef *handle )
{
	UartId_t uart = IdByHandle(handle);
	//DBG("%s %i\n",__FUNCTION__, uart);
	if(UartHandle[uart].Instance) {
		HAL_UART_Receive_IT( &UartHandle[uart], &RxData[uart], 1 );
	}
}

void USART_IRQHandler(const UartId_t usart )
{
    // [BEGIN] Workaround to solve an issue with the HAL drivers not managing the uart state correctly.
    uint32_t tmpFlag = 0, tmpItSource = 0;
    //DBG("%s\n",__FUNCTION__);
    tmpFlag = __HAL_UART_GET_FLAG( &UartHandle[usart], UART_FLAG_TC );
    tmpItSource = __HAL_UART_GET_IT_SOURCE( &UartHandle[usart], UART_IT_TC );
    // UART in mode Transmitter end
    if( ( tmpFlag != RESET ) && ( tmpItSource != RESET ) )
    {
        if( ( UartHandle[usart].gState == HAL_UART_STATE_BUSY_RX ) || UartHandle[usart].gState == HAL_UART_STATE_BUSY_TX_RX )
        {
            UartHandle[usart].gState = HAL_UART_STATE_BUSY_TX_RX;
        }
    }
    // [END] Workaround to solve an issue with the HAL drivers not managing the uart state correctly.

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

extern "C" void USART3_IRQHandler( void )
{
	USART_IRQHandler(USART_3);
}

extern "C" void UART4_IRQHandler( void )
{
	USART_IRQHandler(UART_4);
}

extern "C" void UART5_IRQHandler( void )
{
	USART_IRQHandler(UART_5);
}

