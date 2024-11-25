/*
 * mav.h
 *
 *  Created on: Sep 4, 2024
 *      Author: Andrey Belyakov <andrei.belyakov@simbirsoft.com>
 */

#ifndef _MAV_H_
#define _MAV_H_

#pragma pack(push, 1)
template<typename T, const uint8_t WindowLength>
class MAV{
public:
	MAV():History(), Sum(), WindowPointer(0){};
	virtual ~MAV() = default;
#pragma GCC push_options
#pragma GCC optimize ("-O2")
	T Filter(const T *raw_data)
	{
		Sum += *raw_data;
		Sum -= History[WindowPointer];
		History[WindowPointer] = *raw_data;
		WindowPointer = (WindowPointer < WindowLength - 1) ? WindowPointer + 1 : 0;
		return Sum/WindowLength;
	}
#pragma GCC pop_options
private:
	T History[WindowLength]; /*Array to store values of filter window*/
	T Sum;	/* Sum of filter window's elements*/
	uint8_t WindowPointer; /* Pointer to the first element of window*/
};
#pragma pack(pop)
#endif /* _MAV_H_ */
