#include "OneWire.h"
#include "uart.h"
#include "utilities.h"



#define WIRE_1 0xFF
#define WIRE_0 0x00

#define RESET_SPEED 9600
#define WORK_SPEED 115200

#define OW_TIMEOUT 50 //1 ms is enouph

namespace OneWire {


Bus::Bus( Uart_t *const uart):
	mLastDiscrepancy(),
	mLastFamilyDiscrepancy(),
	mLastDeviceFlag(),
	mROM(),
	mUart(uart),
	mStatus()
{

}

/**
 * internal function to reset uart when an error happens
 * 
 * It seems this function is unneeded when 8 bits are sent separatly
 */
void Bus::resetUART(void)
{
	UartDeInit(mUart);
	UartConfig(mUart, RX_TX, SYNC, RESET_SPEED, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
	mStatus = 0;
}

void Bus::init(void)
{
	mStatus = 0;
	GpioInit( &mUart->Tx, mUart->Tx.pin, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_PULL_UP, 0 );
}

static uint8_t bitsToByte(uint8_t *bits) {
    uint8_t target_byte, i;
    target_byte = 0;
    for (i = 0; i < 8; i++) {
        target_byte = target_byte >> 1;
        if (*bits != WIRE_0) {
            target_byte |= 0b10000000;
        }
        bits++;
    }
    return target_byte;
}

/// Convert one byte to array of 8 bytes
static uint8_t *byteToBits(uint8_t ow_byte, uint8_t *bits) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (ow_byte & 0x01) {
            *bits = WIRE_1;
        } else {
            *bits = WIRE_0;
        }
        bits++;
        ow_byte = ow_byte >> 1;
    }
    return bits;
}

/**
 * If set baud rate with Deinit and Init there will be an unneeded byte 0xF0
 * on the bus
 */
void Bus::setBaudRate(const uint32_t bdr)
{
    UartSetBaudrate(mUart, bdr);
}

bool Bus::reset(void)
{
	//Reset UART if there is an error
	if (mStatus != 0) {
		resetUART();
	}

    uint8_t reset = 0xF0;
    uint8_t resetBack = 0;

    setBaudRate(RESET_SPEED);

    UartPutChar(mUart, reset, OW_TIMEOUT);
    mStatus = UartGetChar(mUart, &resetBack, OW_TIMEOUT);

    setBaudRate(WORK_SPEED);

    return reset!=resetBack;
}

void Bus::sendBit(const uint8_t b)
{
    uint8_t r,s;
    s = b ? WIRE_1 : WIRE_0;
    UartPutChar(mUart, s, OW_TIMEOUT);
    mStatus = UartGetChar(mUart, &r, OW_TIMEOUT);
}

uint8_t Bus::receiveBit(void)
{
    uint8_t s = 0xFF, r;
    UartPutChar(mUart, s, OW_TIMEOUT);
    mStatus = UartGetChar(mUart, &r, OW_TIMEOUT);

	if (r==0xFF) return 1;

	return 0;
}

void Bus::send(const uint8_t b)
{
    uint8_t sendByte[8];
    //uint8_t recvByte[8];

    byteToBits(b, sendByte); //0b01101001 => 0x00 0xFF 0xFF 0x00 0xFF 0x00 0x00 0xFF

	for(uint8_t i=0;i<8;i++) {
		sendBit(sendByte[i]);
	}
	/* 
	On a high loaded system there will be desynchronization of transmit and receive
	buffer. It will lead to timeout errors.
	I'm not sure if deinit and init will clear that error.
	Below the example that creates such error.
	*/
    //HAL_UART_Transmit_IT(ow->huart, sendByte, 8);
    //ow->status = HAL_UART_Receive(ow->huart, recvByte, 8, OW_TIMEOUT);
}

void Bus::send(const uint8_t *bytes, const uint8_t len)
{
    for(uint8_t i=0; i<len; i++) {
		send(bytes[i]);
    }
}

uint8_t Bus::receive(void)
{
    uint8_t sendByte[8];
    uint8_t recvByte[8];
    byteToBits(0xFF, sendByte);

	for (uint8_t i=0;i<8;i++) {
		recvByte[i] = receiveBit();
	}

    return bitsToByte(recvByte);
}

void Bus::receive(uint8_t * const bytes, const uint8_t len)
{
    for(uint8_t i=0;i<len;i++) {
        bytes[i] = receive();
    }
}

uint8_t Bus::crc8(const uint8_t* addr, uint8_t len) const
{
    uint8_t crc = 0, inbyte, i, mix;
	
	while (len--) {
		inbyte = *addr++;
		for (i = 8; i; i--) {
			mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) {
				crc ^= 0x8C;
			}
			inbyte >>= 1;
		}
	}
	
	/* Return calculated CRC */
	return crc;
}

void Bus::resetSearch(void)
{
	/* Reset the search state */
	mLastDiscrepancy = 0;
	mLastDeviceFlag = 0;
	mLastFamilyDiscrepancy = 0;
}


uint8_t Bus::first(void)
{
	/* Reset search values */
	resetSearch();

	/* Start with searching */
	return search(Command::SEARCHROM);
}


uint8_t Bus::search(const Command command)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number, search_result;
	uint8_t id_bit, cmp_id_bit;
	uint8_t rom_byte_mask, search_direction;

	/* Initialize for search */
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = 0;

	/* Check if any devices */
	if (!mLastDeviceFlag) {
		/* 1-Wire reset */
		if (!reset()) {
			/* Reset the search */
            resetSearch();
			return 0; //Reset failed
		}

		/* Issue the search command */
        send(to_underlying(command));
		
		/* Loop to do the search */
		do {
			/* Read a bit and its complement */
			id_bit = receiveBit(); //0
			cmp_id_bit = receiveBit(); //1

			/* Check for no devices on 1-wire */
			if ((id_bit == 1) && (cmp_id_bit == 1)) {
				break;
			} else {
				/* All devices coupled have 0 or 1 */
				if (id_bit != cmp_id_bit) {
					/* Bit write value for search */
					search_direction = id_bit; //1
				} else {
					/* If this discrepancy is before the Last Discrepancy on a previous next then pick the same as last time */
					if (id_bit_number < mLastDiscrepancy) {
						search_direction = (( mROM[rom_byte_number] & rom_byte_mask) > 0);
					} else {
						/* If equal to last pick 1, if not then pick 0 */
						search_direction = (id_bit_number == mLastDiscrepancy);
					}
					
					/* If 0 was picked then record its position in LastZero */
					if (search_direction == 0) {
						last_zero = id_bit_number;

						/* Check for Last discrepancy in family */
						if (last_zero < 9) {
							mLastFamilyDiscrepancy = last_zero;
						}
					}
				}

				/* Set or clear the bit in the ROM byte rom_byte_number with mask rom_byte_mask */
				if (search_direction == 1) { // 1
					mROM[rom_byte_number] |= rom_byte_mask; // |= 1
				} else {
					mROM[rom_byte_number] &= ~rom_byte_mask;
				}
				
				/* Serial number search direction write bit */
                sendBit(search_direction);  //1

				/* Increment the byte counter id_bit_number and shift the mask rom_byte_mask */
				id_bit_number++; // 1 -> 2
				rom_byte_mask <<= 1; // 0b10

				/* If the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask */
				if (rom_byte_mask == 0) {
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		/* Loop until through all ROM bytes 0-7 */
		} while (rom_byte_number < 8);

		/* If the search was successful then */
		if (!(id_bit_number < 65)) {
			/* Search successful so set LastDiscrepancy, LastDeviceFlag, search_result */
			mLastDiscrepancy = last_zero;

			/* Check for last device */
			if (mLastDiscrepancy == 0) {
				mLastDeviceFlag = 1;
			}

			search_result = 1;
		}
	}

	/* If no device found then reset counters so next 'search' will be like a first */
	if (!search_result || !mROM[0]) {
        resetSearch();
		search_result = 0;
	}

	return search_result;
}

uint8_t Bus::next(void)
{
   return search(Bus::Command::SEARCHROM);
}

uint8_t Bus::getROM(const uint8_t index) const
{
	return mROM[index];
}

void Bus::getFullROM(uint8_t * const firstIndex) const
{
	for (uint8_t i = 0; i < 8; i++) {
		*(firstIndex + i) = mROM[i];
	}
}

void Bus::select(uint8_t* addr)
{
	send(to_underlying(Command::MATCHROM));
	for (uint8_t i = 0; i < 8; i++) {
		send(*(addr + i));
	}
}

void Bus::selectWithPointer(uint8_t* ROM)
{
	send(to_underlying(Command::MATCHROM));
	for (uint8_t i = 0; i < 8; i++) {
		send(*(ROM + i));
	}
}

} //namespace OneWire
