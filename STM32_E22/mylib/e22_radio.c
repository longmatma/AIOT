#include "e22_radio.h"
#include <string.h>

/*
 * E22-400M30S wiring fixed for this project (no CubeMX user labels):
 *   PB0  -> NSS
 *   PB1  -> NRST
 *   PB10 <- BUSY
 *   PB11 <- DIO1 (EXTI rising)
 *   PB12 -> RXEN
 *   PB13 -> TXEN
 *   PA5  -> SPI1_SCK
 *   PA6  <- SPI1_MISO
 *   PA7  -> SPI1_MOSI
 */

#define E22_NSS_PORT        GPIOB
#define E22_NSS_PIN         GPIO_PIN_0
#define E22_NRST_PORT       GPIOB
#define E22_NRST_PIN        GPIO_PIN_1
#define E22_BUSY_PORT       GPIOB
#define E22_BUSY_PIN        GPIO_PIN_10
#define E22_DIO1_PORT       GPIOB
#define E22_DIO1_PIN        GPIO_PIN_11
#define E22_RXEN_PORT       GPIOB
#define E22_RXEN_PIN        GPIO_PIN_12
#define E22_TXEN_PORT       GPIOB
#define E22_TXEN_PIN        GPIO_PIN_13

#define SX126X_CMD_SET_STANDBY             0x80U
#define SX126X_CMD_SET_RX                  0x82U
#define SX126X_CMD_SET_TX                  0x83U
#define SX126X_CMD_SET_RF_FREQUENCY        0x86U
#define SX126X_CMD_SET_PACKET_TYPE         0x8AU
#define SX126X_CMD_SET_MODULATION_PARAMS   0x8BU
#define SX126X_CMD_SET_PACKET_PARAMS       0x8CU
#define SX126X_CMD_SET_TX_PARAMS           0x8EU
#define SX126X_CMD_SET_BUFFER_BASE_ADDRESS 0x8FU
#define SX126X_CMD_SET_PA_CONFIG           0x95U
#define SX126X_CMD_SET_REGULATOR_MODE      0x96U
#define SX126X_CMD_SET_DIO3_TCXO_CTRL      0x97U
#define SX126X_CMD_CALIBRATE_IMAGE         0x98U
#define SX126X_CMD_CALIBRATE               0x89U
#define SX126X_CMD_SET_DIO_IRQ_PARAMS      0x08U
#define SX126X_CMD_GET_IRQ_STATUS          0x12U
#define SX126X_CMD_CLEAR_IRQ_STATUS        0x02U
#define SX126X_CMD_GET_RX_BUFFER_STATUS    0x13U
#define SX126X_CMD_GET_PACKET_STATUS       0x14U
#define SX126X_CMD_WRITE_REGISTER          0x0DU
#define SX126X_CMD_READ_REGISTER           0x1DU
#define SX126X_CMD_WRITE_BUFFER            0x0EU
#define SX126X_CMD_READ_BUFFER             0x1EU

#define SX126X_PACKET_TYPE_LORA            0x01U
#define SX126X_STANDBY_RC                  0x00U
#define SX126X_REGULATOR_DCDC              0x01U

#define SX126X_IRQ_TX_DONE                 0x0001U
#define SX126X_IRQ_RX_DONE                 0x0002U
#define SX126X_IRQ_HEADER_ERROR            0x0020U
#define SX126X_IRQ_CRC_ERROR               0x0040U
#define SX126X_IRQ_TIMEOUT                 0x0200U
#define SX126X_IRQ_USED (SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | \
                         SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR | \
                         SX126X_IRQ_TIMEOUT)

#define SX126X_LORA_BW_125                 0x04U
#define SX126X_LORA_BW_250                 0x05U
#define SX126X_LORA_BW_500                 0x06U
#define SX126X_LORA_CR_4_5                 0x01U
#define SX126X_LORA_CR_4_6                 0x02U
#define SX126X_LORA_CR_4_7                 0x03U
#define SX126X_LORA_CR_4_8                 0x04U
#define SX126X_LORA_HEADER_EXPLICIT        0x00U
#define SX126X_LORA_HEADER_IMPLICIT        0x01U
#define SX126X_LORA_CRC_OFF                0x00U
#define SX126X_LORA_CRC_ON                 0x01U
#define SX126X_LORA_IQ_NORMAL              0x00U
#define SX126X_LORA_IQ_INVERTED            0x01U
#define SX126X_RAMP_200_US                 0x04U

#define SX126X_REG_LORA_SYNC_WORD          0x0740U
#define SX126X_REG_IQ_POLARITY             0x0736U
#define SX126X_REG_TX_MODULATION           0x0889U
#define SX126X_REG_TX_CLAMP_CFG            0x08D8U
#define SX126X_REG_OCP                     0x08E7U
#define SX126X_REG_RX_GAIN                 0x08ACU

#define E22_PRIVATE_SYNCWORD_16             0x1424U
#define E22_PUBLIC_SYNCWORD_16              0x3444U

#define E22_SPI_TIMEOUT_MS                  100U
#define E22_BUSY_TIMEOUT_MS                 1000U
#define E22_TX_TIMEOUT_RTC_TICKS            320000UL /* 5 s, 15.625 us/tick */

static SPI_HandleTypeDef *s_hspi = NULL;
static E22_RadioConfig_t s_cfg;
static volatile uint8_t s_dio1_pending = 0U;
static uint8_t s_configured = 0U;
static uint8_t s_tx_busy = 0U;
static uint8_t s_rx_requested = 0U;

static void e22_nss_low(void)
{
    HAL_GPIO_WritePin(E22_NSS_PORT, E22_NSS_PIN, GPIO_PIN_RESET);
}

static void e22_nss_high(void)
{
    HAL_GPIO_WritePin(E22_NSS_PORT, E22_NSS_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef e22_wait_busy_low(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(E22_BUSY_PORT, E22_BUSY_PIN) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

static void e22_rf_standby(void)
{
    HAL_GPIO_WritePin(E22_RXEN_PORT, E22_RXEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E22_TXEN_PORT, E22_TXEN_PIN, GPIO_PIN_RESET);
}

static void e22_rf_rx(void)
{
    HAL_GPIO_WritePin(E22_TXEN_PORT, E22_TXEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E22_RXEN_PORT, E22_RXEN_PIN, GPIO_PIN_SET);
}

static void e22_rf_tx(void)
{
    HAL_GPIO_WritePin(E22_RXEN_PORT, E22_RXEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E22_TXEN_PORT, E22_TXEN_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef e22_write_command(uint8_t opcode, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef st;

    if (s_hspi == NULL)
    {
        return HAL_ERROR;
    }

    st = e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        return st;
    }

    e22_nss_low();
    st = HAL_SPI_Transmit(s_hspi, &opcode, 1U, E22_SPI_TIMEOUT_MS);
    if ((st == HAL_OK) && (length > 0U))
    {
        st = HAL_SPI_Transmit(s_hspi, (uint8_t *)data, length, E22_SPI_TIMEOUT_MS);
    }
    e22_nss_high();

    if (st != HAL_OK)
    {
        return st;
    }
    return e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
}

static HAL_StatusTypeDef e22_read_command(uint8_t opcode, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef st;
    uint8_t nop = 0x00U;

    if ((s_hspi == NULL) || ((length > 0U) && (data == NULL)))
    {
        return HAL_ERROR;
    }

    st = e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        return st;
    }

    e22_nss_low();
    st = HAL_SPI_Transmit(s_hspi, &opcode, 1U, E22_SPI_TIMEOUT_MS);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Transmit(s_hspi, &nop, 1U, E22_SPI_TIMEOUT_MS);
    }
    if ((st == HAL_OK) && (length > 0U))
    {
        st = HAL_SPI_Receive(s_hspi, data, length, E22_SPI_TIMEOUT_MS);
    }
    e22_nss_high();

    if (st != HAL_OK)
    {
        return st;
    }
    return e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
}

static HAL_StatusTypeDef e22_write_register(uint16_t address, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef st;
    uint8_t header[3];

    if ((s_hspi == NULL) || ((length > 0U) && (data == NULL)))
    {
        return HAL_ERROR;
    }

    st = e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        return st;
    }

    header[0] = SX126X_CMD_WRITE_REGISTER;
    header[1] = (uint8_t)(address >> 8);
    header[2] = (uint8_t)(address & 0xFFU);

    e22_nss_low();
    st = HAL_SPI_Transmit(s_hspi, header, sizeof(header), E22_SPI_TIMEOUT_MS);
    if ((st == HAL_OK) && (length > 0U))
    {
        st = HAL_SPI_Transmit(s_hspi, (uint8_t *)data, length, E22_SPI_TIMEOUT_MS);
    }
    e22_nss_high();

    if (st != HAL_OK)
    {
        return st;
    }
    return e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
}

static HAL_StatusTypeDef e22_read_register(uint16_t address, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef st;
    uint8_t header[4];

    if ((s_hspi == NULL) || ((length > 0U) && (data == NULL)))
    {
        return HAL_ERROR;
    }

    st = e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        return st;
    }

    header[0] = SX126X_CMD_READ_REGISTER;
    header[1] = (uint8_t)(address >> 8);
    header[2] = (uint8_t)(address & 0xFFU);
    header[3] = 0x00U;

    e22_nss_low();
    st = HAL_SPI_Transmit(s_hspi, header, sizeof(header), E22_SPI_TIMEOUT_MS);
    if ((st == HAL_OK) && (length > 0U))
    {
        st = HAL_SPI_Receive(s_hspi, data, length, E22_SPI_TIMEOUT_MS);
    }
    e22_nss_high();

    if (st != HAL_OK)
    {
        return st;
    }
    return e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
}

static HAL_StatusTypeDef e22_write_buffer(uint8_t offset, const uint8_t *data, uint8_t length)
{
    HAL_StatusTypeDef st;
    uint8_t header[2];

    if ((data == NULL) && (length > 0U))
    {
        return HAL_ERROR;
    }

    st = e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        return st;
    }

    header[0] = SX126X_CMD_WRITE_BUFFER;
    header[1] = offset;

    e22_nss_low();
    st = HAL_SPI_Transmit(s_hspi, header, sizeof(header), E22_SPI_TIMEOUT_MS);
    if ((st == HAL_OK) && (length > 0U))
    {
        st = HAL_SPI_Transmit(s_hspi, (uint8_t *)data, length, E22_SPI_TIMEOUT_MS);
    }
    e22_nss_high();

    if (st != HAL_OK)
    {
        return st;
    }
    return e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
}

static HAL_StatusTypeDef e22_read_buffer(uint8_t offset, uint8_t *data, uint8_t length)
{
    HAL_StatusTypeDef st;
    uint8_t header[3];

    if ((data == NULL) && (length > 0U))
    {
        return HAL_ERROR;
    }

    st = e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        return st;
    }

    header[0] = SX126X_CMD_READ_BUFFER;
    header[1] = offset;
    header[2] = 0x00U;

    e22_nss_low();
    st = HAL_SPI_Transmit(s_hspi, header, sizeof(header), E22_SPI_TIMEOUT_MS);
    if ((st == HAL_OK) && (length > 0U))
    {
        st = HAL_SPI_Receive(s_hspi, data, length, E22_SPI_TIMEOUT_MS);
    }
    e22_nss_high();

    if (st != HAL_OK)
    {
        return st;
    }
    return e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
}

static HAL_StatusTypeDef e22_clear_irq(uint16_t mask)
{
    uint8_t p[2] = {(uint8_t)(mask >> 8), (uint8_t)mask};
    return e22_write_command(SX126X_CMD_CLEAR_IRQ_STATUS, p, sizeof(p));
}

static HAL_StatusTypeDef e22_get_irq(uint16_t *mask)
{
    HAL_StatusTypeDef st;
    uint8_t p[2] = {0U, 0U};

    if (mask == NULL)
    {
        return HAL_ERROR;
    }

    st = e22_read_command(SX126X_CMD_GET_IRQ_STATUS, p, sizeof(p));
    if (st == HAL_OK)
    {
        *mask = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    }
    return st;
}

static HAL_StatusTypeDef e22_set_irq_routing(void)
{
    uint16_t mask = SX126X_IRQ_USED;
    uint8_t p[8] = {
        (uint8_t)(mask >> 8), (uint8_t)mask,
        (uint8_t)(mask >> 8), (uint8_t)mask,
        0x00U, 0x00U,
        0x00U, 0x00U
    };
    return e22_write_command(SX126X_CMD_SET_DIO_IRQ_PARAMS, p, sizeof(p));
}

static HAL_StatusTypeDef e22_set_packet_params(uint8_t payload_length)
{
    uint8_t header_type = s_cfg.explicit_header ? SX126X_LORA_HEADER_EXPLICIT : SX126X_LORA_HEADER_IMPLICIT;
    uint8_t crc = s_cfg.crc_enabled ? SX126X_LORA_CRC_ON : SX126X_LORA_CRC_OFF;
    uint8_t iq = s_cfg.iq_inverted ? SX126X_LORA_IQ_INVERTED : SX126X_LORA_IQ_NORMAL;
    uint8_t p[6] = {
        (uint8_t)(s_cfg.preamble_symbols >> 8),
        (uint8_t)(s_cfg.preamble_symbols & 0xFFU),
        header_type,
        payload_length,
        crc,
        iq
    };
    HAL_StatusTypeDef st;
    uint8_t reg;

    st = e22_write_command(SX126X_CMD_SET_PACKET_PARAMS, p, sizeof(p));
    if (st != HAL_OK)
    {
        return st;
    }

    /* SX126x datasheet IQ polarity workaround. */
    st = e22_read_register(SX126X_REG_IQ_POLARITY, &reg, 1U);
    if (st != HAL_OK)
    {
        return st;
    }
    if (s_cfg.iq_inverted)
    {
        reg &= (uint8_t)~0x04U;
    }
    else
    {
        reg |= 0x04U;
    }
    return e22_write_register(SX126X_REG_IQ_POLARITY, &reg, 1U);
}

static uint8_t e22_map_bandwidth(uint32_t bandwidth_hz)
{
    if (bandwidth_hz >= 400000UL)
    {
        return SX126X_LORA_BW_500;
    }
    if (bandwidth_hz >= 200000UL)
    {
        return SX126X_LORA_BW_250;
    }
    return SX126X_LORA_BW_125;
}

static uint8_t e22_map_coding_rate(uint8_t denominator)
{
    switch (denominator)
    {
        case 5: return SX126X_LORA_CR_4_5;
        case 6: return SX126X_LORA_CR_4_6;
        case 7: return SX126X_LORA_CR_4_7;
        case 8: return SX126X_LORA_CR_4_8;
        default: return 0U;
    }
}

static int8_t e22_map_module_power_to_sx1268(int8_t module_dbm)
{
    /* Ebyte E22-400M30S table at 433 MHz:
       SX1268 front-stage 18 dBm -> module ~29.96 dBm.
       Other points retained so the bridge can be extended later. */
    if (module_dbm >= 29) return 18;
    if (module_dbm >= 26) return 14;
    if (module_dbm >= 23) return 10;
    if (module_dbm >= 20) return 7;
    if (module_dbm >= 17) return 4;
    if (module_dbm >= 14) return 1;
    if (module_dbm >= 11) return -2;
    return -5;
}

static HAL_StatusTypeDef e22_set_sync_word(uint8_t sx127x_sync_word)
{
    uint16_t sync16;
    uint8_t p[2];

    if (sx127x_sync_word == 0x12U)
    {
        sync16 = E22_PRIVATE_SYNCWORD_16;
    }
    else if (sx127x_sync_word == 0x34U)
    {
        sync16 = E22_PUBLIC_SYNCWORD_16;
    }
    else
    {
        /* This bridge intentionally supports the two standard compatible words only. */
        return HAL_ERROR;
    }

    p[0] = (uint8_t)(sync16 >> 8);
    p[1] = (uint8_t)sync16;
    return e22_write_register(SX126X_REG_LORA_SYNC_WORD, p, sizeof(p));
}

static HAL_StatusTypeDef e22_set_frequency(uint32_t frequency_hz)
{
    uint64_t reg64 = ((uint64_t)frequency_hz << 25) / 32000000ULL;
    uint32_t reg = (uint32_t)reg64;
    uint8_t p[4] = {
        (uint8_t)(reg >> 24),
        (uint8_t)(reg >> 16),
        (uint8_t)(reg >> 8),
        (uint8_t)reg
    };
    uint8_t cal[2];
    HAL_StatusTypeDef st;

    if (frequency_hz > 460000000UL)
    {
        cal[0] = 0x75U;
        cal[1] = 0x81U;
    }
    else
    {
        cal[0] = 0x6BU;
        cal[1] = 0x6FU;
    }

    st = e22_write_command(SX126X_CMD_CALIBRATE_IMAGE, cal, sizeof(cal));
    if (st != HAL_OK)
    {
        return st;
    }
    return e22_write_command(SX126X_CMD_SET_RF_FREQUENCY, p, sizeof(p));
}

static HAL_StatusTypeDef e22_apply_500khz_workaround(uint8_t bw_code)
{
    uint8_t reg;
    HAL_StatusTypeDef st = e22_read_register(SX126X_REG_TX_MODULATION, &reg, 1U);
    if (st != HAL_OK)
    {
        return st;
    }

    if (bw_code == SX126X_LORA_BW_500)
    {
        reg &= (uint8_t)~0x04U;
    }
    else
    {
        reg |= 0x04U;
    }
    return e22_write_register(SX126X_REG_TX_MODULATION, &reg, 1U);
}

static HAL_StatusTypeDef e22_apply_pa_safety_workarounds(void)
{
    uint8_t reg;
    HAL_StatusTypeDef st;

    /* Better tolerance to antenna mismatch (SX126x errata/workaround). */
    st = e22_read_register(SX126X_REG_TX_CLAMP_CFG, &reg, 1U);
    if (st != HAL_OK)
    {
        return st;
    }
    reg |= 0x1EU;
    st = e22_write_register(SX126X_REG_TX_CLAMP_CFG, &reg, 1U);
    if (st != HAL_OK)
    {
        return st;
    }

    /* High-power SX1268 OCP setting used by Semtech-derived drivers. */
    reg = 0x38U;
    return e22_write_register(SX126X_REG_OCP, &reg, 1U);
}

static HAL_StatusTypeDef e22_set_rx_boosted(void)
{
    uint8_t reg = 0x96U;
    return e22_write_register(SX126X_REG_RX_GAIN, &reg, 1U);
}

static HAL_StatusTypeDef e22_hw_reset(void)
{
    e22_rf_standby();
    e22_nss_high();

    HAL_GPIO_WritePin(E22_NRST_PORT, E22_NRST_PIN, GPIO_PIN_SET);
    HAL_Delay(2U);
    HAL_GPIO_WritePin(E22_NRST_PORT, E22_NRST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20U);
    HAL_GPIO_WritePin(E22_NRST_PORT, E22_NRST_PIN, GPIO_PIN_SET);
    HAL_Delay(10U);

    return e22_wait_busy_low(E22_BUSY_TIMEOUT_MS);
}

HAL_StatusTypeDef E22_Radio_Init(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef st;
    uint8_t p[4];

    if (hspi == NULL)
    {
        return HAL_ERROR;
    }

    s_hspi = hspi;
    s_configured = 0U;
    s_tx_busy = 0U;
    s_rx_requested = 0U;
    s_dio1_pending = 0U;
    memset(&s_cfg, 0, sizeof(s_cfg));

    st = e22_hw_reset();
    if (st != HAL_OK)
    {
        return st;
    }

    p[0] = SX126X_STANDBY_RC;
    st = e22_write_command(SX126X_CMD_SET_STANDBY, p, 1U);
    if (st != HAL_OK)
    {
        return st;
    }

    /* E22-400M30S uses DIO3 internally to supply its 32 MHz TCXO at 2.2 V. */
    p[0] = 0x03U; /* 2.2 V */
    p[1] = 0x00U;
    p[2] = 0x01U;
    p[3] = 0x40U; /* 320 * 15.625 us = 5 ms */
    st = e22_write_command(SX126X_CMD_SET_DIO3_TCXO_CTRL, p, 4U);
    if (st != HAL_OK)
    {
        return st;
    }

    p[0] = SX126X_REGULATOR_DCDC;
    st = e22_write_command(SX126X_CMD_SET_REGULATOR_MODE, p, 1U);
    if (st != HAL_OK)
    {
        return st;
    }

    p[0] = 0x7FU; /* calibrate RC64k, RC13M, PLL, ADC, image blocks */
    st = e22_write_command(SX126X_CMD_CALIBRATE, p, 1U);
    if (st != HAL_OK)
    {
        return st;
    }

    return HAL_OK;
}

HAL_StatusTypeDef E22_Radio_Configure(const E22_RadioConfig_t *cfg)
{
    HAL_StatusTypeDef st;
    uint8_t p[8];
    uint8_t bw;
    uint8_t cr;
    int8_t sx_power;

    if ((cfg == NULL) || (s_hspi == NULL))
    {
        return HAL_ERROR;
    }
    if ((cfg->spreading_factor < 5U) || (cfg->spreading_factor > 12U))
    {
        return HAL_ERROR;
    }

    bw = e22_map_bandwidth(cfg->bandwidth_hz);
    cr = e22_map_coding_rate(cfg->coding_rate_denominator);
    if (cr == 0U)
    {
        return HAL_ERROR;
    }

    memcpy(&s_cfg, cfg, sizeof(s_cfg));
    if (s_cfg.preamble_symbols == 0U)
    {
        s_cfg.preamble_symbols = 8U;
    }

    p[0] = SX126X_STANDBY_RC;
    st = e22_write_command(SX126X_CMD_SET_STANDBY, p, 1U);
    if (st != HAL_OK) return st;

    p[0] = SX126X_PACKET_TYPE_LORA;
    st = e22_write_command(SX126X_CMD_SET_PACKET_TYPE, p, 1U);
    if (st != HAL_OK) return st;

    st = e22_set_frequency(s_cfg.frequency_hz);
    if (st != HAL_OK) return st;

    p[0] = 0x00U; /* TX base */
    p[1] = 0x00U; /* RX base */
    st = e22_write_command(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, p, 2U);
    if (st != HAL_OK) return st;

    p[0] = s_cfg.spreading_factor;
    p[1] = bw;
    p[2] = cr;
    p[3] = 0x00U; /* LDRO not needed for SF7/BW500; safe for current fixed setup */
    st = e22_write_command(SX126X_CMD_SET_MODULATION_PARAMS, p, 4U);
    if (st != HAL_OK) return st;

    st = e22_apply_500khz_workaround(bw);
    if (st != HAL_OK) return st;

    st = e22_set_packet_params(0xFFU);
    if (st != HAL_OK) return st;

    st = e22_set_sync_word(s_cfg.sync_word);
    if (st != HAL_OK) return st;

    /* High-power PA setup for SX1268 front stage. */
    p[0] = 0x04U; /* paDutyCycle */
    p[1] = 0x07U; /* hpMax */
    p[2] = 0x00U; /* SX1262/SX1268 */
    p[3] = 0x01U; /* paLut */
    st = e22_write_command(SX126X_CMD_SET_PA_CONFIG, p, 4U);
    if (st != HAL_OK) return st;

    st = e22_apply_pa_safety_workarounds();
    if (st != HAL_OK) return st;

    sx_power = e22_map_module_power_to_sx1268(s_cfg.module_tx_power_dbm);
    p[0] = (uint8_t)sx_power;
    p[1] = SX126X_RAMP_200_US;
    st = e22_write_command(SX126X_CMD_SET_TX_PARAMS, p, 2U);
    if (st != HAL_OK) return st;

    st = e22_set_rx_boosted();
    if (st != HAL_OK) return st;

    st = e22_set_irq_routing();
    if (st != HAL_OK) return st;

    st = e22_clear_irq(0xFFFFU);
    if (st != HAL_OK) return st;

    s_configured = 1U;
    s_tx_busy = 0U;
    s_rx_requested = 0U;
    e22_rf_standby();
    return HAL_OK;
}

HAL_StatusTypeDef E22_Radio_StartRxContinuous(void)
{
    HAL_StatusTypeDef st;
    uint8_t p[3] = {0xFFU, 0xFFU, 0xFFU};

    if (!s_configured)
    {
        return HAL_ERROR;
    }
    if (s_tx_busy)
    {
        return HAL_BUSY;
    }

    st = e22_set_packet_params(0xFFU);
    if (st != HAL_OK) return st;

    st = e22_clear_irq(0xFFFFU);
    if (st != HAL_OK) return st;

    e22_rf_rx();
    HAL_Delay(1U);

    st = e22_write_command(SX126X_CMD_SET_RX, p, sizeof(p));
    if (st == HAL_OK)
    {
        s_rx_requested = 1U;
    }
    return st;
}

HAL_StatusTypeDef E22_Radio_Send(const uint8_t *data, uint8_t length)
{
    HAL_StatusTypeDef st;
    uint8_t timeout[3];

    if ((!s_configured) || (data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }
    if (s_tx_busy)
    {
        return HAL_BUSY;
    }

    s_rx_requested = 1U; /* gateway returns to RX automatically after TX */

    st = e22_write_buffer(0U, data, length);
    if (st != HAL_OK) return st;

    st = e22_set_packet_params(length);
    if (st != HAL_OK) return st;

    st = e22_apply_500khz_workaround(e22_map_bandwidth(s_cfg.bandwidth_hz));
    if (st != HAL_OK) return st;

    st = e22_clear_irq(0xFFFFU);
    if (st != HAL_OK) return st;

    /* Ebyte requires >=2 ms after selecting TX RF switch before transmission. */
    e22_rf_tx();
    HAL_Delay(2U);

    timeout[0] = (uint8_t)(E22_TX_TIMEOUT_RTC_TICKS >> 16);
    timeout[1] = (uint8_t)(E22_TX_TIMEOUT_RTC_TICKS >> 8);
    timeout[2] = (uint8_t)E22_TX_TIMEOUT_RTC_TICKS;
    st = e22_write_command(SX126X_CMD_SET_TX, timeout, sizeof(timeout));
    if (st == HAL_OK)
    {
        s_tx_busy = 1U;
    }
    else
    {
        e22_rf_standby();
    }
    return st;
}

HAL_StatusTypeDef E22_Radio_ResetAndRestore(void)
{
    E22_RadioConfig_t cfg = s_cfg;
    uint8_t had_config = s_configured;
    HAL_StatusTypeDef st;

    st = E22_Radio_Init(s_hspi);
    if (st != HAL_OK)
    {
        return st;
    }

    if (had_config)
    {
        st = E22_Radio_Configure(&cfg);
        if (st != HAL_OK)
        {
            return st;
        }
        return E22_Radio_StartRxContinuous();
    }

    return HAL_OK;
}

void E22_Radio_NotifyDio1Irq(void)
{
    s_dio1_pending = 1U;
}

static int16_t e22_snr_raw_to_x10(uint8_t raw)
{
    int16_t v = (int8_t)raw;
    int16_t scaled = (int16_t)(v * 5); /* raw/4 dB => raw*2.5 in x10 */
    if (scaled >= 0)
    {
        return (int16_t)((scaled + 1) / 2);
    }
    return (int16_t)((scaled - 1) / 2);
}

bool E22_Radio_PollEvent(E22_RadioEvent_t *event_out)
{
    HAL_StatusTypeDef st;
    uint16_t irq;
    uint8_t status[3];
    uint8_t rx_status[2];

    if (event_out == NULL)
    {
        return false;
    }

    memset(event_out, 0, sizeof(*event_out));
    event_out->type = E22_RADIO_EVENT_NONE;

    if (!s_dio1_pending)
    {
        return false;
    }
    s_dio1_pending = 0U;

    st = e22_get_irq(&irq);
    if (st != HAL_OK)
    {
        event_out->type = E22_RADIO_EVENT_ERROR;
        return true;
    }
    event_out->irq_status = irq;
    (void)e22_clear_irq(irq);

    if ((irq & SX126X_IRQ_TX_DONE) != 0U)
    {
        s_tx_busy = 0U;
        e22_rf_standby();
        event_out->type = E22_RADIO_EVENT_TX_DONE;

        if (s_rx_requested)
        {
            (void)E22_Radio_StartRxContinuous();
        }
        return true;
    }

    if ((irq & SX126X_IRQ_RX_DONE) != 0U)
    {
        if ((irq & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) != 0U)
        {
            event_out->type = E22_RADIO_EVENT_RX_ERROR;
            if (s_rx_requested)
            {
                (void)E22_Radio_StartRxContinuous();
            }
            return true;
        }

        st = e22_read_command(SX126X_CMD_GET_RX_BUFFER_STATUS, rx_status, 2U);
        if (st != HAL_OK)
        {
            event_out->type = E22_RADIO_EVENT_ERROR;
            return true;
        }

        event_out->length = rx_status[0];
        if (event_out->length > 0U)
        {
            st = e22_read_buffer(rx_status[1], event_out->data, event_out->length);
            if (st != HAL_OK)
            {
                event_out->type = E22_RADIO_EVENT_ERROR;
                return true;
            }
        }

        st = e22_read_command(SX126X_CMD_GET_PACKET_STATUS, status, 3U);
        if (st != HAL_OK)
        {
            event_out->type = E22_RADIO_EVENT_ERROR;
            return true;
        }

        /* RSSI = -raw/2 dBm -> x10 = -raw*5. */
        event_out->rssi_x10_dbm = (int16_t)(-((int16_t)status[0] * 5));
        /* SNR = signed(raw)/4 dB. */
        event_out->snr_x10_db = e22_snr_raw_to_x10(status[1]);
        event_out->type = E22_RADIO_EVENT_RX_DONE;

        if (s_rx_requested)
        {
            (void)E22_Radio_StartRxContinuous();
        }
        return true;
    }

    if ((irq & SX126X_IRQ_TIMEOUT) != 0U)
    {
        if (s_tx_busy)
        {
            s_tx_busy = 0U;
            e22_rf_standby();
            event_out->type = E22_RADIO_EVENT_TX_TIMEOUT;
            if (s_rx_requested)
            {
                (void)E22_Radio_StartRxContinuous();
            }
            return true;
        }
    }

    if ((irq & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) != 0U)
    {
        event_out->type = E22_RADIO_EVENT_RX_ERROR;
        if (s_rx_requested)
        {
            (void)E22_Radio_StartRxContinuous();
        }
        return true;
    }

    return false;
}

bool E22_Radio_IsConfigured(void)
{
    return s_configured != 0U;
}

bool E22_Radio_IsTxBusy(void)
{
    return s_tx_busy != 0U;
}

const E22_RadioConfig_t *E22_Radio_GetConfig(void)
{
    return &s_cfg;
}
