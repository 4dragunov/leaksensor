/*
 * flash-stm32.h
 *
 *  Created on: Jan 10, 2025
 *      Author: Andrey
 */

#ifndef SRC_BOARDS_SRWLD01_FLASH_STM32_H_
#define SRC_BOARDS_SRWLD01_FLASH_STM32_H_

#include "flash-board.h"

template<class T>
class Stm32FlashInfo:public IFlashInfo<Stm32FlashInfo> {
	Stm32FlashInfo() = default;
	virtual ~Stm32FlashInfo() = default;
public:
	virtual size_t size() override;
	virtual size_t pagesize() override;
	static IFlashInfo& Instance();
};

#endif /* SRC_BOARDS_SRWLD01_FLASH_STM32_H_ */
