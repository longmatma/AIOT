#include "pi_bridge.h"
#include "crc16.h"
#include <string.h>

#define BRIDGE_MAGIC0                 0xA5U
#define BRIDGE_MAGIC1                 0x5AU
#define BRIDGE_VERSION                0x01U
#define BRIDGE_MAX_PAYLOAD            512U
#define BRIDGE_MAX_BODY               (6U + BRIDGE_MAX_PAYLOAD + 2U)
#define BRIDGE_UART_RING_SIZE          1024U

#define CMD_PING                      0x01U
#define CMD_CONFIG_RADIO              0x02U
#define CMD_START_RX                  0x03U
#define CMD_TX_RAW                    0x04U
#define CMD_RADIO_RESET               0x05U

#define EVT_ACK                       0x80U
#define EVT_RX_PACKET                 0x81U
#define EVT_TX_DONE                   0x82U
#define EVT_ERROR                     0x83U
#define EVT_STATUS                    0x84U

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
    HAL_StatusTypeDef st = E22_Radio_StartRxContinuous();
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
    if (s_pending_tx_valid || E22_Radio_IsTxBusy())
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
    parser_reset();

    return HAL_UART_Receive_IT(s_huart, &s_uart_rx_byte, 1U);
}

void PiBridge_Process(void)
{
    uint8_t byte;
    while (ring_pop(&byte))
    {
        parser_feed(byte);
    }
}

void PiBridge_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart != NULL) && (huart == s_huart))
    {
        ring_push_isr(s_uart_rx_byte);
        (void)HAL_UART_Receive_IT(s_huart, &s_uart_rx_byte, 1U);
    }
}

void PiBridge_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart != NULL) && (huart == s_huart))
    {
        __HAL_UART_CLEAR_OREFLAG(s_huart);
        (void)HAL_UART_Receive_IT(s_huart, &s_uart_rx_byte, 1U);
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
            if (s_pending_tx_valid)
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
            if (s_pending_tx_valid)
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
