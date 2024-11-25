/*
 * reset.h
 *
 *  Created on: Aug 31, 2024
 *      Author: Andrey
 */

#ifndef SRC_RESET_H_
#define SRC_RESET_H_


class Reset{
public:
	/// @brief  Possible STM32 system reset causes
	typedef enum Cause
	{
	    UNKNOWN = 0,
	    LOW_POWER_RESET,
	    WINDOW_WATCHDOG_RESET,
	    INDEPENDENT_WATCHDOG_RESET,
	    SOFTWARE_RESET,
	    POWER_ON_POWER_DOWN_RESET,
	    EXTERNAL_RESET_PIN_RESET,
	    BROWNOUT_RESET,
	} Cause;
	/// @brief      Obtain the STM32 system reset cause
	/// @param      None
	/// @return     The system reset cause
	static const Reset::Cause cause;
	static const char * cause_name(Reset::Cause reset_cause);
protected:
	static Reset::Cause reset_cause_get(void);
};


#endif /* SRC_RESET_H_ */
