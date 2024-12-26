#pragma once
/**
 ********************* Description *************************
 * This library is based on the labrary taken from 
 * https://stm32f4-discovery.net/2015/07/hal-library-05-onewire-for-stm32fxxx/
 * 
 * Baud Rate - any, it will be reset in the reset function.
 * 8-n-1
 * Connection of RX & TX should be done as specified here 
 *
 * +3.3V----+
 *          |
 *          П 4.7k Resistor
 *          U
 *          |
 * RX ------+-------- to 1Wire bus
 *          |
 * TX ------+  - no diode needed
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include <cmsis_os.h>
#include "uart.h"
#include "utils.h"

namespace OneWire {

	class Bus {

	public:
		/* OneWire commands */
		enum class Command {
			RSCRATCHPAD		=	0xBE,
			WSCRATCHPAD		=	0x4E,
			CPYSCRATCHPAD	=	0x48,
			RECEEPROM		=	0xB8,
			RPWRSUPPLY		=	0xB4,
			SEARCHROM		=	0xF0,
			READROM			=	0x33,
			MATCHROM		=	0x55,
			SKIPROM			=	0xCC
		};
	/**
	 * @brief Initialization of library. Do it before using
	 * @par uart pointer to UART handle
	 */
	 Bus(Uart_t  *const uart);
	 virtual ~Bus() = default;

	 void init(void);

	/**
	 * @brief Each communication with OneWire bus must start with
	 * this function.
	 * @return true - some devices discovered, false - no devices on the bus
	 */
	bool reset(void);

	/**
	 * @brief   Send one byte through OneWire bus
	 * @par     b - bytes to send
	 */
	void send(const uint8_t b);

	/**
	 * @brief   Send some bytes through OneWire bus
	 * @par     *bytes - array of bytes to send
	 * @par     len - length of array
	 */
	void send(const uint8_t *bytes, const uint8_t len);

	/**
	 * @brief   Receive one byte through OneWire bus
	 * @return  received byte
	 */
	uint8_t receive(void);

	/**
	 * @brief   Receive some bytes from OneWire bus
	 * @par     *bytes - array to fill
	 * @par     len - length of bytes
	 */
	void receive(uint8_t * const bytes, const uint8_t len);

	/**
	 * @brief  Calculates 8-bit CRC for 1-wire devices
	 * @par    *addr: Pointer to 8-bit array of data to calculate CRC
	 * @par    len: Number of bytes to check
	 *
	 * @return Calculated CRC from input data
	 */
	uint8_t crc8(const uint8_t* addr, uint8_t len) const;

	/**
	 * @brief  Starts search, reset states first
	 * @note   When you want to search for ALL devices on one onewire port, you should first use this function.
	\code
	//...Initialization before
	status = first(&OneWireStruct);
	while (status) {
		//Save ROM number from device
		getFullROM(&OneWireStruct, ROM_Array_Pointer);
		//Check for new device
		status = next(&OneWireStruct);
	}
	\endcode
	 * @retval  Device status:
	 *            - 0: No devices detected
	 *            - > 0: Device detected
	 */
	uint8_t first(void);

	/**
	 * @brief  Resets search states
	 */
	void resetSearch(void);

	/**
	 * @brief  Searches for OneWire devices on specific Onewire port
	 * @note   Not meant for public use. Use @ref first and @ref next for this.
	 * @param command - command to send to OneWire devices
	 * @retval Device status:
	 *            - 0: No devices detected
	 *            - > 0: Device detected
	 */
	uint8_t search(const Command command);

	/**
	 * @brief  Reads next device
	 * @note   Use @ref first to start searching
	 * @retval  Device status:
	 *            - 0: No devices detected any more
	 *            - > 0: New device detected
	 */
	uint8_t next(void);

	/**
	 * @brief  Gets ROM number from device from search
	 * @param  index: Because each device has 8-bytes long ROM address, you have to call this 8 times, to get ROM bytes from 0 to 7
	 * @retval ROM byte for index (0 to 7) at current found device
	 */
	uint8_t getROM(uint8_t index) const;

	/**
	 * @brief  Gets all 8 bytes ROM value from device from search
	 * @param  *firstIndex: Pointer to first location for first byte, other bytes are automatically incremented
	 */
	void getFullROM(uint8_t *firstIndex) const;

	/**
	 * @brief  Selects specific slave on bus
	 * @param  *addr: Pointer to first location of 8-bytes long ROM address
	 */
	void select(uint8_t* const addr);

	/**
	 * @brief  Selects specific slave on bus with pointer address
	 * @param  *ROM: Pointer to first byte of ROM address
	 */
	void selectWithPointer(uint8_t* const ROM);

	void setBaudRate(const uint32_t bdr);
	private:
		void resetUART(void);
		uint8_t receiveBit(void);
		void sendBit(const uint8_t b);

		uint8_t mLastDiscrepancy;
		uint8_t mLastFamilyDiscrepancy;
		uint8_t mLastDeviceFlag;
		uint8_t mROM[8];
		Uart_t *mUart;
		uint8_t mStatus;
	}; //class Bus

} //namespace OneWire
