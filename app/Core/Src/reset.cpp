#include "reset.h"
#include "main.h"

/// @brief      Obtain the STM32 system reset cause
/// @param      None
/// @return     The system reset cause
Reset::Cause Reset::reset_cause_get(void)
{
	Reset::Cause reset_cause;
#ifdef RCC_FLAG_LPWRRST
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_cause = Reset::Cause::LOW_POWER_RESET;
    }
#endif
#ifdef RCC_FLAG_WWDGRST
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_cause = Reset::Cause::WINDOW_WATCHDOG_RESET;
    }
#endif
#ifdef RCC_FLAG_IWDGRST
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_cause = Reset::Cause::INDEPENDENT_WATCHDOG_RESET;
    }
#endif
#ifdef RCC_FLAG_SFTRST
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        // This reset is induced by calling the ARM CMSIS
        // `NVIC_SystemReset()` function!
        reset_cause = Reset::Cause::SOFTWARE_RESET;
    }
#endif
#ifdef RCC_FLAG_PORRST
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_cause = Reset::Cause::POWER_ON_POWER_DOWN_RESET;
    }
#endif
#ifdef RCC_FLAG_PINRST
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_cause = Reset::Cause::EXTERNAL_RESET_PIN_RESET;
    }
#endif
    // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
    // ensure first that the reset cause is NOT a POR/PDR reset. See note
    // below.
#ifdef RCC_FLAG_BORRST
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
    {
        reset_cause = Reset::Cause::BROWNOUT_RESET;
    }
#endif
    else
    {
        reset_cause = Reset::Cause::UNKNOWN;
    }

    // Clear all the reset flags or else they will remain set during future
    // resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_cause;
}

const Reset::Cause Reset::cause(reset_cause_get());

// Note: any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock
// Controller (RCC) header files, such as
// "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
// "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h",
// etc., indicate that the brownout flag, `RCC_FLAG_BORRST`, will be set in
// the event of a "POR/PDR or BOR reset". This means that a Power-On Reset
// (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag.
// See the doxygen just above their definition for the
// `__HAL_RCC_GET_FLAG()` macro to see this:
//      "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout
//      Reset flag will *also* be set in the event of a POR/PDR.
// Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after*
// first checking the `RCC_FLAG_PORRST` flag in order to ensure first that the
// reset cause is NOT a POR/PDR reset.


/// @brief      Obtain the system reset cause as an ASCII-printable name string
///             from a reset cause type
/// @param[in]  reset_cause     The previously-obtained system reset cause
/// @return     A null-terminated ASCII name string describing the system
///             reset cause
const char * Reset::cause_name(Reset::Cause reset_cause)
{
    const char * reset_cause_name = "UNK";

    switch (reset_cause)
    {
        case UNKNOWN:
            reset_cause_name = "UNK";
            break;
#ifdef RCC_FLAG_LPWRRST
        case LOW_POWER_RESET:
            reset_cause_name = "LPWR";
            break;
#endif
#ifdef RCC_FLAG_WWDGRST
        case WINDOW_WATCHDOG_RESET:
            reset_cause_name = "WWDG";
            break;
#endif
#ifdef RCC_FLAG_IWDGRST
        case INDEPENDENT_WATCHDOG_RESET:
            reset_cause_name = "IWDG";
            break;
#endif
#ifdef RCC_FLAG_SFTRST
        case SOFTWARE_RESET:
            reset_cause_name = "SOFT";
            break;
#endif
#ifdef RCC_FLAG_PORRST
        case POWER_ON_POWER_DOWN_RESET:
            reset_cause_name = "POR/PDR";
            break;
#endif
#ifdef RCC_FLAG_PINRST
        case EXTERNAL_RESET_PIN_RESET:
            reset_cause_name = "EXT_PIN";
            break;
#endif
#ifdef RCC_FLAG_BORRST
        case BROWNOUT_RESET:
            reset_cause_name = "BOR";
            break;
#endif
    }

    return reset_cause_name;
}
