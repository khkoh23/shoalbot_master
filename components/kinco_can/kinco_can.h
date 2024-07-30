#ifndef _KINCO_CAN_
#define _KINCO_CAN_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "esp_system.h"

#include "driver/twai.h"

void initTwai (const uint8_t tx, const uint8_t rx);
void setTargetVelocity (const uint8_t id, int32_t msg);
void setModesOfOperation (const uint8_t id, const int8_t msg);
void setProfileAcceleration (const uint8_t, const uint32_t msg);
void setProfileDeceleration (const uint8_t, const uint32_t msg);

#ifdef __cplusplus
}
#endif

#endif //_KINCO_CAN_