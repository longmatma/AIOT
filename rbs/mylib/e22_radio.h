#ifndef E22_RADIO_H
#define E22_RADIO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define E22_RF_MAX_PACKET_LEN 255U

typedef struct
{
    uint32_t frequency_hz;
    uint32_t bandwidth_hz;
    uint8_t  spreading_factor;
    uint8_t  coding_rate_denominator; /* 5 => 4/5 */
    uint8_t  explicit_header;         /* 1 = explicit */
    int8_t   module_tx_power_dbm;     /* requested E22 module output */
    uint16_t preamble_symbols;
    uint8_t  sync_word;               /* SX127x-style: 0x12 private, 0x34 public */
    uint8_t  crc_enabled;
    uint8_t  iq_inverted;
} E22_RadioConfig_t;

typedef enum
{
    E22_RADIO_EVENT_NONE = 0,
    E22_RADIO_EVENT_TX_DONE,
    E22_RADIO_EVENT_RX_DONE,
    E22_RADIO_EVENT_TX_TIMEOUT,
    E22_RADIO_EVENT_RX_ERROR,
    E22_RADIO_EVENT_ERROR
} E22_RadioEventType_t;

typedef struct
{
    E22_RadioEventType_t type;
    uint8_t data[E22_RF_MAX_PACKET_LEN];
    uint8_t length;
    int16_t rssi_x10_dbm;
    int16_t snr_x10_db;
    uint16_t irq_status;
} E22_RadioEvent_t;

HAL_StatusTypeDef E22_Radio_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef E22_Radio_Configure(const E22_RadioConfig_t *cfg);
HAL_StatusTypeDef E22_Radio_StartRxContinuous(void);
HAL_StatusTypeDef E22_Radio_Send(const uint8_t *data, uint8_t length);
HAL_StatusTypeDef E22_Radio_ResetAndRestore(void);

void E22_Radio_NotifyDio1Irq(void);
bool E22_Radio_PollEvent(E22_RadioEvent_t *event_out);

bool E22_Radio_IsConfigured(void);
bool E22_Radio_IsTxBusy(void);
const E22_RadioConfig_t *E22_Radio_GetConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* E22_RADIO_H */
