#ifndef PI_BRIDGE_H
#define PI_BRIDGE_H

#include "main.h"
#include "e22_radio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef PiBridge_Init(UART_HandleTypeDef *huart);
void PiBridge_Process(void);

/* PI_BRIDGE_UART_RECOVERY_V3 */
void PiBridge_UartService(void);
uint8_t PiBridge_UartRxHealthy(void);

void PiBridge_UartRxCpltCallback(UART_HandleTypeDef *huart);
void PiBridge_UartErrorCallback(UART_HandleTypeDef *huart);
void PiBridge_HandleRadioEvent(const E22_RadioEvent_t *event);

#ifdef __cplusplus
}
#endif

#endif /* PI_BRIDGE_H */
