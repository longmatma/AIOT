#include "pi_bridge.h"
#include "crc16.h"
#include <string.h>

#define BRIDGE_MAGIC0                 0xA5U
#define BRIDGE_MAGIC1                 0x5AU
#define BRIDGE_VERSION                0x01U
#define BRIDGE_MAX_PAYLOAD            600U
#define BRIDGE_MAX_BODY               (6U + BRIDGE_MAX_PAYLOAD + 2U)
#define BRIDGE_UART_RING_SIZE          1024U

/* PI_BRIDGE_UART_RECOVERY_V3 */
#define PI_BRIDGE_UART_RECOVERY_V3        1U
#define PI_BRIDGE_UART_SERVICE_MS        10U

#define CMD_PING                      0x01U
#define CMD_CONFIG_RADIO              0x02U
#define CMD_START_RX                  0x03U
#define CMD_TX_RAW                    0x04U
#define CMD_RADIO_RESET               0x05U
#define CMD_TX_BURST                  0x06U

#define EVT_ACK                       0x80U
#define EVT_RX_PACKET                 0x81U
#define EVT_TX_DONE                   0x82U
#define EVT_ERROR                     0x83U
#define EVT_STATUS                    0x84U
#define EVT_BURST_DONE                0x85U

#define PI_BRIDGE_BURST_MAX_ITEMS        6U
#define PI_BRIDGE_BURST_MAX_GUARD_MS    20U

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_uart_rx_byte;
static volatile uint16_t s_ring_head = 0U;
static volatile uint16_t s_ring_tail = 0U;
static uint8_t s_ring[BRIDGE_UART_RING_SIZE];

static uint8_t s_parse_state = 0U;
static uint8_t s_body[BRIDGE_MAX_BODY];
static uint16_t s_body_index = 0U;
static uint16_t s_expected_total = 0U;

static uint16_t s_pending_tx_seq = 0U;
static uint8_t s_pending_tx_valid = 0U;

/* V2B.2: STM32 autonomous downlink burst engine.
 * Pi queues up to 4 complete LoRa packets in one UART command. STM32 sends
 * them back-to-back with a small guard and reports one EVT_BURST_DONE.
 * This removes Python blocking between every relay packet while keeping all
 * session/AES/FEC policy on Raspberry Pi.
 */
typedef struct
{
    uint8_t active;
    uint8_t tx_inflight;
    uint8_t waiting_guard;
    uint8_t count;
    uint8_t next_index;
    uint8_t guard_ms;
    uint16_t cmd_seq;
    uint32_t next_due_ms;
    uint32_t started_ms;
    uint8_t length[PI_BRIDGE_BURST_MAX_ITEMS];
    uint8_t data[PI_BRIDGE_BURST_MAX_ITEMS][E22_RF_MAX_PACKET_LEN];
} PiBridgeBurstState_t;

static PiBridgeBurstState_t s_burst;

static volatile HAL_StatusTypeDef s_last_rx_arm_status = HAL_ERROR;
static volatile uint32_t s_uart_error_count = 0U;
static volatile uint32_t s_uart_rearm_fail_count = 0U;
static uint32_t s_uart_service_last_ms = 0U;

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static uint8_t ring_pop(uint8_t *byte_out)
{
    uint16_t tail;

    if (byte_out == NULL)
    {
        return 0U;
    }

    tail = s_ring_tail;
    if (tail == s_ring_head)
    {
        return 0U;
    }

    *byte_out = s_ring[tail];
    tail++;
    if (tail >= BRIDGE_UART_RING_SIZE)
    {
        tail = 0U;
    }
    s_ring_tail = tail;
    return 1U;
}

static void ring_push_isr(uint8_t byte)
{
    uint16_t head = s_ring_head;
    uint16_t next = head + 1U;

    if (next >= BRIDGE_UART_RING_SIZE)
    {
        next = 0U;
    }

    if (next == s_ring_tail)
    {
        /* Ring full: drop oldest byte so parser can recover on next MAGIC. */
        uint16_t tail = s_ring_tail + 1U;
        if (tail >= BRIDGE_UART_RING_SIZE)
        {
            tail = 0U;
        }
        s_ring_tail = tail;
    }

    s_ring[head] = byte;
    s_ring_head = next;
}

static HAL_StatusTypeDef bridge_send_frame(uint8_t type, uint16_t seq,
                                           const uint8_t *payload, uint16_t length)
{
    uint8_t frame[2U + 6U + BRIDGE_MAX_PAYLOAD + 2U];
    uint16_t crc;
    uint16_t total;

    if ((s_huart == NULL) || (length > BRIDGE_MAX_PAYLOAD))
    {
        return HAL_ERROR;
    }
    if ((length > 0U) && (payload == NULL))
    {
        return HAL_ERROR;
    }

    frame[0] = BRIDGE_MAGIC0;
    frame[1] = BRIDGE_MAGIC1;
    frame[2] = BRIDGE_VERSION;
    frame[3] = type;
    write_u16_le(&frame[4], seq);
    write_u16_le(&frame[6], length);
    if (length > 0U)
    {
        memcpy(&frame[8], payload, length);
    }

    crc = CRC16_CCITT_FALSE(&frame[2], (size_t)(6U + length));
    write_u16_le(&frame[8U + length], crc);
    total = (uint16_t)(10U + length);

    return HAL_UART_Transmit(s_huart, frame, total, 1000U);
}

static void bridge_send_error(uint16_t seq, const char *text)
{
    uint16_t len = 0U;

    if (text != NULL)
    {
        while ((text[len] != '\0') && (len < BRIDGE_MAX_PAYLOAD))
        {
            len++;
        }
    }
    (void)bridge_send_frame(EVT_ERROR, seq, (const uint8_t *)text, len);
}

static void bridge_send_ack(uint16_t seq)
{
    static const uint8_t ok[] = {'O', 'K'};
    (void)bridge_send_frame(EVT_ACK, seq, ok, sizeof(ok));
}

static void burst_reset(void)
{
    memset(&s_burst, 0, sizeof(s_burst));
}

static HAL_StatusTypeDef burst_start_next(void)
{
    HAL_StatusTypeDef st;
    uint8_t idx;

    if (!s_burst.active || s_burst.tx_inflight)
    {
        return HAL_ERROR;
    }
    if (s_burst.next_index >= s_burst.count)
    {
        return HAL_ERROR;
    }
    if (E22_Radio_IsTxBusy())
    {
        return HAL_BUSY;
    }

    idx = s_burst.next_index;
    st = E22_Radio_Send(s_burst.data[idx], s_burst.length[idx]);
    if (st == HAL_OK)
    {
        s_burst.tx_inflight = 1U;
        s_burst.waiting_guard = 0U;
        s_burst.next_index = (uint8_t)(idx + 1U);
    }
    return st;
}

static void burst_service(void)
{
    uint32_t now;

    if (!s_burst.active || s_burst.tx_inflight || !s_burst.waiting_guard)
    {
        return;
    }

    now = HAL_GetTick();
    if ((int32_t)(now - s_burst.next_due_ms) < 0)
    {
        return;
    }

    if (burst_start_next() != HAL_OK)
    {
        uint16_t seq = s_burst.cmd_seq;
        burst_reset();
        bridge_send_error(seq, "BURST_NEXT_FAIL");
    }
}

static void handle_tx_burst(uint16_t seq, const uint8_t *payload, uint16_t length)
{
    uint16_t pos = 0U;
    uint8_t count;
    uint8_t guard_ms;
    uint8_t i;

    if ((payload == NULL) || (length < 3U))
    {
        bridge_send_error(seq, "BAD_BURST_LEN");
        return;
    }
    if (s_pending_tx_valid || s_burst.active || E22_Radio_IsTxBusy())
    {
        bridge_send_error(seq, "TX_BUSY");
        return;
    }

    count = payload[pos++];
    guard_ms = payload[pos++];
    if ((count < 1U) || (count > PI_BRIDGE_BURST_MAX_ITEMS) ||
        (guard_ms > PI_BRIDGE_BURST_MAX_GUARD_MS))
    {
        bridge_send_error(seq, "BAD_BURST_META");
        return;
    }

    burst_reset();
    s_burst.count = count;
    s_burst.guard_ms = guard_ms;
    s_burst.cmd_seq = seq;

    for (i = 0U; i < count; i++)
    {
        uint8_t item_len;
        if (pos >= length)
        {
            burst_reset();
            bridge_send_error(seq, "BAD_BURST_ITEM");
            return;
        }
        item_len = payload[pos++];
        if ((item_len == 0U) || (item_len > E22_RF_MAX_PACKET_LEN) ||
            ((uint32_t)pos + (uint32_t)item_len > (uint32_t)length))
        {
            burst_reset();
            bridge_send_error(seq, "BAD_BURST_ITEM");
            return;
        }
        s_burst.length[i] = item_len;
        memcpy(s_burst.data[i], &payload[pos], item_len);
        pos = (uint16_t)(pos + item_len);
    }

    if (pos != length)
    {
        burst_reset();
        bridge_send_error(seq, "BURST_TRAILING_BYTES");
        return;
    }

    s_burst.active = 1U;
    s_burst.started_ms = HAL_GetTick();
    s_burst.next_index = 0U;

    if (burst_start_next() != HAL_OK)
    {
        burst_reset();
        bridge_send_error(seq, "BURST_START_FAIL");
        return;
    }

    /* ACK means accepted/started, not RF-complete. Completion is EVT_BURST_DONE. */
    bridge_send_ack(seq);
}

static void handle_ping(uint16_t seq)
{
    static const uint8_t info[] = "STM32F103-E22-BRIDGE-V1";
    (void)bridge_send_frame(EVT_ACK, seq, info, (uint16_t)(sizeof(info) - 1U));
}

static void handle_config(uint16_t seq, const uint8_t *payload, uint16_t length)
{
    E22_RadioConfig_t cfg;
    HAL_StatusTypeDef st;

    if ((payload == NULL) || (length != 16U))
    {
        bridge_send_error(seq, "BAD_CONFIG_LEN");
        return;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.frequency_hz = read_u32_le(&payload[0]);
    cfg.bandwidth_hz = read_u32_le(&payload[4]);
    cfg.spreading_factor = payload[8];
    cfg.coding_rate_denominator = payload[9];
    cfg.explicit_header = payload[10];
    cfg.module_tx_power_dbm = (int8_t)payload[11];
    cfg.preamble_symbols = read_u16_le(&payload[12]);
    cfg.sync_word = payload[14];
    cfg.crc_enabled = (payload[15] & 0x01U) ? 1U : 0U;
    cfg.iq_inverted = (payload[15] & 0x02U) ? 1U : 0U;

    st = E22_Radio_Configure(&cfg);
    if (st == HAL_OK)
    {
        bridge_send_ack(seq);
    }
    else
    {
        bridge_send_error(seq, "RADIO_CONFIG_FAIL");
    }
}

static void handle_start_rx(uint16_t seq)
{
    HAL_StatusTypeDef st;
    if (s_burst.active)
    {
        bridge_send_error(seq, "BURST_BUSY");
        return;
    }
    st = E22_Radio_StartRxContinuous();
    if (st == HAL_OK)
    {
        bridge_send_ack(seq);
    }
    else
    {
        bridge_send_error(seq, "START_RX_FAIL");
    }
}

static void handle_tx_raw(uint16_t seq, const uint8_t *payload, uint16_t length)
{
    HAL_StatusTypeDef st;

    if ((payload == NULL) || (length == 0U) || (length > E22_RF_MAX_PACKET_LEN))
    {
        bridge_send_error(seq, "BAD_TX_LEN");
        return;
    }
    if (s_pending_tx_valid || s_burst.active || E22_Radio_IsTxBusy())
    {
        bridge_send_error(seq, "TX_BUSY");
        return;
    }

    st = E22_Radio_Send(payload, (uint8_t)length);
    if (st == HAL_OK)
    {
        s_pending_tx_seq = seq;
        s_pending_tx_valid = 1U;
        /* No ACK here: Python waits for EVT_TX_DONE with the same seq. */
    }
    else
    {
        bridge_send_error(seq, "TX_START_FAIL");
    }
}

static void handle_reset(uint16_t seq)
{
    HAL_StatusTypeDef st = E22_Radio_ResetAndRestore();
    if (st == HAL_OK)
    {
        s_pending_tx_valid = 0U;
        burst_reset();
        bridge_send_ack(seq);
    }
    else
    {
        bridge_send_error(seq, "RADIO_RESET_FAIL");
    }
}

static void bridge_handle_frame(uint8_t type, uint16_t seq,
                                const uint8_t *payload, uint16_t length)
{
    switch (type)
    {
        case CMD_PING:
            handle_ping(seq);
            break;
        case CMD_CONFIG_RADIO:
            handle_config(seq, payload, length);
            break;
        case CMD_START_RX:
            handle_start_rx(seq);
            break;
        case CMD_TX_RAW:
            handle_tx_raw(seq, payload, length);
            break;
        case CMD_RADIO_RESET:
            handle_reset(seq);
            break;
        case CMD_TX_BURST:
            handle_tx_burst(seq, payload, length);
            break;
        default:
            bridge_send_error(seq, "UNKNOWN_CMD");
            break;
    }
}

static void parser_reset(void)
{
    s_parse_state = 0U;
    s_body_index = 0U;
    s_expected_total = 0U;
}

static void parser_feed(uint8_t byte)
{
    uint16_t payload_len;
    uint16_t crc_rx;
    uint16_t crc_calc;
    uint16_t seq;
    uint8_t type;

    switch (s_parse_state)
    {
        case 0U:
            if (byte == BRIDGE_MAGIC0)
            {
                s_parse_state = 1U;
            }
            break;

        case 1U:
            if (byte == BRIDGE_MAGIC1)
            {
                s_parse_state = 2U;
                s_body_index = 0U;
                s_expected_total = 0U;
            }
            else if (byte != BRIDGE_MAGIC0)
            {
                s_parse_state = 0U;
            }
            break;

        case 2U:
            if (s_body_index >= BRIDGE_MAX_BODY)
            {
                parser_reset();
                break;
            }

            s_body[s_body_index++] = byte;

            if (s_body_index == 6U)
            {
                payload_len = read_u16_le(&s_body[4]);
                if ((s_body[0] != BRIDGE_VERSION) || (payload_len > BRIDGE_MAX_PAYLOAD))
                {
                    parser_reset();
                    break;
                }
                s_expected_total = (uint16_t)(6U + payload_len + 2U);
            }

            if ((s_expected_total > 0U) && (s_body_index == s_expected_total))
            {
                payload_len = read_u16_le(&s_body[4]);
                crc_rx = read_u16_le(&s_body[6U + payload_len]);
                crc_calc = CRC16_CCITT_FALSE(s_body, (size_t)(6U + payload_len));

                if (crc_rx == crc_calc)
                {
                    type = s_body[1];
                    seq = read_u16_le(&s_body[2]);
                    bridge_handle_frame(type, seq, &s_body[6], payload_len);
                }
                parser_reset();
            }
            break;

        default:
            parser_reset();
            break;
    }
}

static void bridge_uart_clear_error_flags(void)
{
    volatile uint32_t dummy;

    if (s_huart == NULL)
        return;

    dummy = s_huart->Instance->SR;
    dummy = s_huart->Instance->DR;
    (void)dummy;

    s_huart->ErrorCode = HAL_UART_ERROR_NONE;
}

static HAL_StatusTypeDef bridge_uart_arm_rx(void)
{
    HAL_StatusTypeDef st;

    if (s_huart == NULL)
    {
        s_last_rx_arm_status = HAL_ERROR;
        return HAL_ERROR;
    }

    if (s_huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        s_last_rx_arm_status = HAL_OK;
        return HAL_OK;
    }

    st = HAL_UART_Receive_IT(s_huart, &s_uart_rx_byte, 1U);

    if ((st == HAL_BUSY) && (s_huart->RxState == HAL_UART_STATE_BUSY_RX))
        st = HAL_OK;

    s_last_rx_arm_status = st;

    if (st != HAL_OK)
        s_uart_rearm_fail_count++;

    return st;
}

HAL_StatusTypeDef PiBridge_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    s_huart = huart;
    s_ring_head = 0U;
    s_ring_tail = 0U;
    s_pending_tx_valid = 0U;
    burst_reset();
    s_last_rx_arm_status = HAL_ERROR;
    s_uart_error_count = 0U;
    s_uart_rearm_fail_count = 0U;
    s_uart_service_last_ms = HAL_GetTick();
    parser_reset();

    bridge_uart_clear_error_flags();
    return bridge_uart_arm_rx();
}

void PiBridge_Process(void)
{
    uint8_t byte;
    while (ring_pop(&byte))
    {
        parser_feed(byte);
    }
    burst_service();
}

void PiBridge_UartService(void)
{
    uint32_t now;

    if (s_huart == NULL)
        return;

    now = HAL_GetTick();

    if ((now - s_uart_service_last_ms) < PI_BRIDGE_UART_SERVICE_MS)
        return;

    s_uart_service_last_ms = now;

    if (s_huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        s_uart_error_count++;
        s_ring_head = 0U;
        s_ring_tail = 0U;
        parser_reset();
        bridge_uart_clear_error_flags();
    }

    if (s_huart->RxState != HAL_UART_STATE_BUSY_RX)
        (void)bridge_uart_arm_rx();
}

uint8_t PiBridge_UartRxHealthy(void)
{
    if (s_huart == NULL)
        return 0U;

    if (s_huart->ErrorCode != HAL_UART_ERROR_NONE)
        return 0U;

    if (s_huart->RxState != HAL_UART_STATE_BUSY_RX)
        return 0U;

    if (s_last_rx_arm_status != HAL_OK)
        return 0U;

    return 1U;
}

void PiBridge_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart != NULL) && (huart == s_huart))
    {
        ring_push_isr(s_uart_rx_byte);
        (void)bridge_uart_arm_rx();
    }
}

void PiBridge_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart != NULL) && (huart == s_huart))
    {
        s_uart_error_count++;
        s_ring_head = 0U;
        s_ring_tail = 0U;
        parser_reset();

        bridge_uart_clear_error_flags();
        (void)bridge_uart_arm_rx();
    }
}

void PiBridge_HandleRadioEvent(const E22_RadioEvent_t *event)
{
    uint8_t payload[6U + E22_RF_MAX_PACKET_LEN];
    uint16_t seq = 0U;
    uint16_t len;

    if (event == NULL)
    {
        return;
    }

    switch (event->type)
    {
        case E22_RADIO_EVENT_TX_DONE:
            if (s_burst.active && s_burst.tx_inflight)
            {
                s_burst.tx_inflight = 0U;
                if (s_burst.next_index >= s_burst.count)
                {
                    uint8_t done_payload[6];
                    uint32_t elapsed = HAL_GetTick() - s_burst.started_ms;
                    seq = s_burst.cmd_seq;
                    done_payload[0] = s_burst.count;
                    done_payload[1] = s_burst.guard_ms;
                    done_payload[2] = (uint8_t)(elapsed & 0xFFU);
                    done_payload[3] = (uint8_t)((elapsed >> 8) & 0xFFU);
                    done_payload[4] = (uint8_t)((elapsed >> 16) & 0xFFU);
                    done_payload[5] = (uint8_t)((elapsed >> 24) & 0xFFU);
                    burst_reset();
                    (void)bridge_send_frame(EVT_BURST_DONE, seq, done_payload, sizeof(done_payload));
                }
                else
                {
                    s_burst.waiting_guard = 1U;
                    s_burst.next_due_ms = HAL_GetTick() + s_burst.guard_ms;
                }
            }
            else if (s_pending_tx_valid)
            {
                seq = s_pending_tx_seq;
                s_pending_tx_valid = 0U;
                (void)bridge_send_frame(EVT_TX_DONE, seq, NULL, 0U);
            }
            break;

        case E22_RADIO_EVENT_RX_DONE:
            write_u16_le(&payload[0], (uint16_t)event->rssi_x10_dbm);
            write_u16_le(&payload[2], (uint16_t)event->snr_x10_db);
            write_u16_le(&payload[4], event->length);
            if (event->length > 0U)
            {
                memcpy(&payload[6], event->data, event->length);
            }
            len = (uint16_t)(6U + event->length);
            (void)bridge_send_frame(EVT_RX_PACKET, 0U, payload, len);
            break;

        case E22_RADIO_EVENT_TX_TIMEOUT:
            if (s_burst.active)
            {
                seq = s_burst.cmd_seq;
                burst_reset();
                bridge_send_error(seq, "BURST_TX_TIMEOUT");
            }
            else if (s_pending_tx_valid)
            {
                seq = s_pending_tx_seq;
                s_pending_tx_valid = 0U;
                bridge_send_error(seq, "TX_TIMEOUT");
            }
            else
            {
                bridge_send_error(0U, "TX_TIMEOUT");
            }
            break;

        case E22_RADIO_EVENT_RX_ERROR:
            /* CRC/header errors are expected RF events; do not spam Pi. */
            break;

        case E22_RADIO_EVENT_ERROR:
            bridge_send_error(0U, "RADIO_IRQ_FAIL");
            break;

        case E22_RADIO_EVENT_NONE:
        default:
            break;
    }
}
