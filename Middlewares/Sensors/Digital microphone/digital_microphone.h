#ifndef DIGITAL_MICROPHONE_H
#define DIGITAL_MICROPHONE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include <stdint.h>

#define DIGITAL_MICROPHONE_PAYLOAD_SIZE  1U

HAL_StatusTypeDef DigitalMicrophone_Init(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef DigitalMicrophone_ConfigureWindow(uint16_t window_ms);
HAL_StatusTypeDef DigitalMicrophone_Start(void);
void DigitalMicrophone_Process(void);
void DigitalMicrophone_Stop(void);
uint16_t DigitalMicrophone_AppendPayload(uint8_t *buffer, uint16_t buffer_size);
uint32_t DigitalMicrophone_GetDroppedBlocks(void);
uint8_t DigitalMicrophone_IsRunning(void);
uint8_t DigitalMicrophone_HasValue(void);
uint8_t DigitalMicrophone_GetLastValue(void);

#ifdef __cplusplus
}
#endif

#endif /* DIGITAL_MICROPHONE_H */
