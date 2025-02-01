#include <stm32f1xx.h>
#include "flash-board.h"

#define FLASH_SIZE_KB             (uint32_t)(*((uint32_t *)FLASHSIZE_BASE)&0xFFFF)
#define FLASH_SIZE                (uint32_t)(FLASH_SIZE_KB * 1024U)

class Stm32FlashInfo:public IFlashInfo {
	friend IFlashInfo& boardFlashInstance();
	Stm32FlashInfo() = default;
	virtual ~Stm32FlashInfo() = default;
	Stm32FlashInfo(Stm32FlashInfo const&)= delete;
	Stm32FlashInfo& operator= (Stm32FlashInfo const&)= delete;
public:
	static IFlashInfo& Instance();
	virtual size_t size() override;
	virtual size_t pagesize() override;
};

IFlashInfo& Stm32FlashInfo::Instance()
{
	static Stm32FlashInfo s;
	return s;
}

size_t Stm32FlashInfo::size() {
	return FLASH_SIZE;
}

size_t Stm32FlashInfo::pagesize(){
	return (FLASH_SIZE_KB > 128)? 2048 : 1024;
}

IFlashInfo& boardFlashInstance(){
	return Stm32FlashInfo::Instance();
}
