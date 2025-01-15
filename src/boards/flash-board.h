/*
 * flash.h
 *
 *  Created on: Jan 10, 2025
 *      Author: Andrey
 */

#ifndef SRC_BOARDS_SRWLD01_FLASH_H_
#define SRC_BOARDS_SRWLD01_FLASH_H_

template<class T>
class IFlashInfo {
	friend IFlashInfo& boardFlashInstance();
	IFlashInfo( )= default;;
	virtual ~IFlashInfo() = default;
	IFlashInfo(IFlashInfo const&)= delete;
	IFlashInfo& operator= (IFlashInfo const&)= delete;
public:
	virtual size_t size()  = 0;
	virtual size_t pagesize() = 0;
	static IFlashInfo& Instance() {return T::Instance();};
};

extern IFlashInfo& boardFlashInstance();

#endif /* SRC_BOARDS_SRWLD01_FLASH_H_ */
