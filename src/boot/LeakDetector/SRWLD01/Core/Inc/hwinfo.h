/*
 * hwinfo.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Andrey Belyakov <andrei.belyakov@simbirsoft.com>
 */

#ifndef INC_HWINFO_H_
#define INC_HWINFO_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define FLASH_SIZE_DATA_REGISTER        FLASHSIZE_BASE

#if defined (FLASH_OPTR_DBANK)
#define FLASH_SIZE                      ((((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0xFFFFU)) ? (0x200UL << 10U) : \
                                        (((*((uint32_t *)FLASH_SIZE_DATA_REGISTER)) & 0xFFFFUL) << 10U))
#define FLASH_BANK_SIZE                 (FLASH_SIZE >> 1)
#define FLASH_PAGE_NB                   128U
#define FLASH_PAGE_SIZE_128_BITS        0x1000U /* 4 KB */
#else
#define FLASH_SIZE                      ((((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0xFFFFU)) ? (0x80UL << 10U) : \
                                        (((*((uint32_t *)FLASH_SIZE_DATA_REGISTER)) & 0xFFFFUL) << 10U))
#define FLASH_BANK_SIZE                 (FLASH_SIZE)
#define FLASH_PAGE_NB                   ((FLASH_SIZE == 0x00080000U) ? 256U : 64U)
#endif


typedef enum {
	HW_LEAKDET_LORA = 100
} HwModel;

typedef union HWInfo {
	struct {
	uint8_t hw0;
	uint8_t hw1;
	uint8_t hw2;
	uint8_t hw3;
	} id;
	uint8_t hw[4];
} HWInfo;

#ifdef __cplusplus
}
#endif

#endif /* INC_HWINFO_H_ */
