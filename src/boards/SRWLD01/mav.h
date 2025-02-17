/*
 * mav.h
 *
 *  Created on: Sep 4, 2024
 *      Author: Andrey Belyakov <andrei.belyakov@simbirsoft.com>
 */

#ifndef _MAV_H_
#define _MAV_H_

#include <cstdint>
#include <stdbool.h>

#pragma pack(push, 1)
template<typename T, const uint8_t WindowLength>
class MAV{
public:
	MAV():History(), Sum(), WindowPointer(0), FirstRun(true){};
	virtual ~MAV() = default;
	T Filter(const T *raw_data)
	{
		if(FirstRun) {
			for(size_t i = 0; i < WindowLength - 1; i++)
			{
				DoFilter(raw_data);
			}
			FirstRun = false;
			return DoFilter(raw_data);
		}else
			return DoFilter(raw_data);
	}

private:
	T History[WindowLength]; /*Array to store values of filter window*/
	T Sum;	/* Sum of filter window's elements*/
	uint8_t WindowPointer; /* Pointer to the first element of window*/
	bool FirstRun;
#pragma GCC push_options
#pragma GCC optimize ("-O2")
	T DoFilter(const T *raw_data)
	{
		Sum += *raw_data;
		Sum -= History[WindowPointer];
		History[WindowPointer] = *raw_data;
		WindowPointer = (WindowPointer < WindowLength - 1) ? WindowPointer + 1 : 0;
		return Sum/WindowLength;
	}
#pragma GCC pop_options
};
#pragma pack(pop)
#endif /* _MAV_H_ */
