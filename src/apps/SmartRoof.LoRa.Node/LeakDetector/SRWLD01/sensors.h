/*
 * sensors.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Andrey
 */

#ifndef SRC_APPS_SMARTROOF_LORA_NODE_LEAKDETECTOR_SRWLD01_SENSORS_H_
#define SRC_APPS_SMARTROOF_LORA_NODE_LEAKDETECTOR_SRWLD01_SENSORS_H_
#include <sys/time.h>

typedef struct {
	int16_t data[8];
	uint8_t sensors;
	struct timeval timestamp;
} ThermalSensorsData ;

typedef struct{
	Samples leakSamples;
	ThermalSensorsData thermal;
	struct timeval timestamp;
}SummarySensorsData;

#endif /* SRC_APPS_SMARTROOF_LORA_NODE_LEAKDETECTOR_SRWLD01_SENSORS_H_ */
