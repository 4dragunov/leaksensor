/*!
 * \file      uart.c
 *
 * \brief     UART driver implementation
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
#include "uart-board.h"
#include "uart.h"

osSemaphoreDef(rxSem);
osSemaphoreDef(txSem);

const char *gUsartNames[] = {
		FOREACH_USART(GENERATE_STRING)
};

void UartInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx )
{
    if( obj->IsInitialized == false )
    {
    	const char usartName[20];

    	obj->rxSem = osSemaphoreCreate(osSemaphore(rxSem), 1);
    	vQueueAddToRegistry( obj->rxSem, "rxSem" );
    	obj->txSem = osSemaphoreCreate(osSemaphore(txSem), 1);
    	vQueueAddToRegistry( obj->txSem, "txSem"  );
        obj->IsInitialized = true;
        UartMcuInit( obj, uartId, tx, rx );
    }
}

void UartConfig( Uart_t *obj, UartMode_t mode, FifoMode_t fifo, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    UartMcuConfig( obj, mode, fifo, baudrate, wordLength, stopBits, parity, flowCtrl );
}

void UartDeInit( Uart_t *obj )
{
    obj->IsInitialized = false;
    UartMcuDeInit( obj );
    osSemaphoreDelete(obj->rxSem);
    osSemaphoreDelete(obj->txSem);
}

uint8_t UartPutChar( Uart_t *obj, uint8_t data , uint32_t timeout )
{
     return UartMcuPutChar( obj, data, timeout  );
}

uint8_t UartGetChar( Uart_t *obj, uint8_t *data , uint32_t timeout )
{
    return UartMcuGetChar( obj, data, timeout );
}

uint8_t UartPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size , uint32_t timeout )
{
    return UartMcuPutBuffer( obj, buffer, size, timeout );
}

uint8_t UartGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes ,uint32_t timeout )
{
    return UartMcuGetBuffer( obj, buffer, size, nbReadBytes , timeout );
}

bool UartWaitReady(Uart_t *obj, uint32_t millisec){
	return UartMcuWaitReady(obj, millisec);
}

uint8_t UartGetBufferToIdle( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes , uint32_t timeout){
	return  UartMcuGetBufferToIdle( obj, buffer, size, nbReadBytes , timeout);

}

uint32_t UartGetBaudrate(const Uart_t *obj)
{
	return UartMcuGetBaudrate(obj);
}

bool UartSetBaudrate(const Uart_t *obj, uint32_t baudrate)
{
	return UartMcuSetBaudrate(obj,baudrate);
}

void UartAbortReceive(const Uart_t *obj) {
  UartMcuAbortReceive(obj);
}

void UartEnableTransmitter(const Uart_t *obj){
	UartMcuEnableTransmitter(obj);
}

void UartEnableReciever(const Uart_t *obj){
	UartMcuEnableReciever(obj);
}

bool UartLastByteSendOut(const Uart_t *obj, uint32_t timeout){
	UartMcuLastByteSendOut(obj, timeout);
}
