/**
 * @ Author: Morran Smith
 * @ Create Time: 2019-06-01 09:51:05
 * @ Modified by: Morran Smith
 * @ Modified time: 2019-06-02 21:30:21
 * @ Description:
 */

#include "sx127x.h"
#include "sx127x_io.h"
#include "sx127x_private.h"
#include "sx127x_registers.h"

uint8_t sx127x_reset(sx127x_dev_t* dev)
{
    dev->common->reset_control(true);
    dev->common->delay(10);
    dev->common->reset_control(false);
    dev->common->delay(50);

    return 0;
}

void sx127x_dio_0_callback(sx127x_dev_t* dev)
{
    volatile uint8_t irq = sx127x_get_irq_flags(dev);

    switch (dev->settings.mode) {
    case MODE_TX:
        if (!(irq & FlagTxDone))
            break;

        sx127x_clear_irq_flags(dev, FlagTxDone);

        dev->callbacks->tx_done();

        break;
    case MODE_RXCONTINUOUS:
        if (irq & FlagPayloadCrcError) {
            sx127x_clear_irq_flags(dev, FlagPayloadCrcError);

            dev->callbacks->rx_crc_error();
            break;
        }

        if (irq & FlagRxTimeout) {
            sx127x_clear_irq_flags(dev, FlagRxTimeout);

            dev->callbacks->rx_timeout();
            break;
        }
        //no break
    case MODE_RXSINGLE: {

        if (!(irq & FlagRxDone))
            break;

        uint8_t size = sx127x_get_last_packet_size(dev);
        uint8_t buffer[size];

        sx127x_read_fifo(dev, buffer, size);

        sx127x_clear_irq_flags(dev, FlagRxDone);

        dev->callbacks->rx_done(buffer, size);

        break;
    }
    default:
        break;
    }
}

void sx127x_dio_1_callback(sx127x_dev_t* dev)
{

    if (dev->settings.mode != MODE_RXSINGLE)
        return;

    if (!(sx127x_get_irq_flags(dev) & FlagRxTimeout))
        return;

    sx127x_clear_irq_flags(dev, FlagRxTimeout);

    dev->callbacks->rx_timeout();
}

void sx127x_dio_2_callback(sx127x_dev_t* dev)
{
}

void sx127x_dio_3_callback(sx127x_dev_t* dev)
{

    if (!(dev->settings.mode == MODE_RXSINGLE || dev->settings.mode == MODE_RXCONTINUOUS))
        return;

    if (!(sx127x_get_irq_flags(dev) & FlagPayloadCrcError))
        return;

    dev->callbacks->rx_crc_error();

    sx127x_clear_irq_flags(dev, FlagPayloadCrcError);
}

void sx127x_dio_4_callback(sx127x_dev_t* dev)
{
}

void sx127x_dio_5_callback(sx127x_dev_t* dev)
{
}

uint8_t sx127x_init(sx127x_dev_t* dev, sx127x_radio_settings_t* settings)
{
    sx127x_reset(dev);
		
		if (sx127x_get_version(dev) != VERSION)
					return -1;
		
    if (sx127x_set_sleep(dev) != 0)
        return -1;

    if (sx127x_set_modulation_mode(dev, settings->modulation) != 0)
        return -1;

    if (sx127x_set_frequency(dev, settings->frequency) != 0)
        return -1;

    if (sx127x_set_pa_select(dev, settings->pa_select) != 0)
        return -1;

    if (sx127x_set_power(dev, settings->power) != 0)
        return -1;

    if (sx127x_set_band_width(dev, settings->band_width) != 0)
        return -1;

    if (sx127x_set_coding_rate(dev, settings->coding_rate) != 0)
        return -1;

    if (sx127x_set_spreading_factor(dev, settings->spreading_factor) != 0)
        return -1;

    if (sx127x_set_payload_crc_on(dev, settings->payload_crc_on) != 0)
        return -1;

    if (sx127x_set_preamble_length(dev, settings->preamble_length) != 0)
        return -1;

    if (sx127x_set_sync_word(dev, settings->sync_word) != 0)
        return -1;

    return 0;
}

uint8_t sx127x_load_current_parameters(sx127x_dev_t* dev)
{
    sx127x_get_modulation_mode(dev);
    sx127x_get_mode(dev);
    sx127x_get_pa_select(dev);
    sx127x_get_power(dev);
    sx127x_get_spreading_factor(dev);
    sx127x_get_band_width(dev);
    sx127x_get_coding_rate(dev);
    sx127x_get_payload_crc_on(dev);
    sx127x_get_preamble_length(dev);
    sx127x_get_frequency(dev);
    sx127x_get_sync_word(dev);

    return 0;
}

uint8_t sx127x_get_version(sx127x_dev_t* dev)
{
    return sx127x_read_register(dev->spi, RegVersion);
}

int sx127x_get_rssi(sx127x_dev_t* dev)
{
    return ((dev->settings.frequency < 520000000) ? -164 : -157) + (int)(int8_t)(sx127x_read_register(dev->spi, RegRssiValue));
}

int sx127x_get_last_packet_rssi(sx127x_dev_t* dev)
{
    if (sx127x_get_last_packet_snr(dev) >= 0)
        return ((dev->settings.frequency < 520000000) ? -164 : -157) + (16 / 15 * (int)(int8_t)(sx127x_read_register(dev->spi, RegPktRssiValue)));
    return ((dev->settings.frequency < 520000000) ? -164 : -157) + (int)(int8_t)(sx127x_read_register(dev->spi, RegPktRssiValue));
}

int sx127x_get_last_packet_snr(sx127x_dev_t* dev)
{
    return (int)(int8_t)(sx127x_read_register(dev->spi, RegPktSnrValue)) / 4;
}

uint8_t sx127x_set_sleep(sx127x_dev_t* dev)
{
    uint8_t reg = sx127x_read_register(dev->spi, RegOpMode);

    sx127x_write_register(dev->spi, RegOpMode, (reg & 0xF8) | MODE_SLEEP);

    dev->settings.mode = MODE_SLEEP;

    return 0;
}

uint8_t sx127x_set_standby(sx127x_dev_t* dev)
{
    uint8_t reg = sx127x_read_register(dev->spi, RegOpMode);

    sx127x_write_register(dev->spi, RegOpMode, (reg & 0xF8) | MODE_STDBY);

    dev->settings.mode = MODE_STDBY;

    return 0;
}

uint8_t sx127x_transmit(sx127x_dev_t* dev, uint8_t* buffer, uint8_t size, uint32_t delay)
{
    if (sx127x_set_dio_config(dev, (DIO_MODE_DISABLE << DIO_0_MAPPING) | (DIO_MODE_DISABLE << DIO_1_MAPPING) | (DIO_MODE_DISABLE << DIO_2_MAPPING) | (DIO_MODE_DISABLE << DIO_3_MAPPING) | (DIO_MODE_DISABLE << DIO_4_MAPPING) | (DIO_MODE_DISABLE << DIO_5_MAPPING)) != 0) // Отключение DIO
        return -1;

    if (sx127x_clear_irq_flags(dev, FlagTxDone) != 0) // На всякий случай очистра прерывания TxDone
        return -1;

    if (sx127x_set_irq_flags_mask(dev, FlagTxDone) != 0) // Включение прерывания по окончании отправки
        return -1;

    if (sx127x_set_standby(dev) != 0) // FIFO не доступен в режиме SLEEP
        return -1;

    if (sx127x_write_fifo(dev, buffer, size) != 0) // Загрузка FIFO
        return -1;

    if (sx127x_set_tx(dev) != 0) // Начало отправки
        return -1;

    while (!(sx127x_get_irq_flags(dev) & FlagTxDone)) {
        dev->common->delay(1);

        if (!delay--)
            return -1;
    }

    return 0;
}

uint8_t sx127x_transmit_it(sx127x_dev_t* dev, uint8_t* buffer, uint8_t size)
{
    if (sx127x_set_dio_config(dev, (DIO_MODE_1 << DIO_0_MAPPING) | (DIO_MODE_DISABLE << DIO_1_MAPPING) | (DIO_MODE_DISABLE << DIO_2_MAPPING) | (DIO_MODE_DISABLE << DIO_3_MAPPING) | (DIO_MODE_DISABLE << DIO_4_MAPPING) | (DIO_MODE_DISABLE << DIO_5_MAPPING)) != 0) // Включение DIO0
        return -1;

    if (sx127x_clear_irq_flags(dev, FlagTxDone) != 0) // На всякий случай очистра прерывания TxDone
        return -1;

    if (sx127x_set_irq_flags_mask(dev, FlagTxDone) != 0) // Включение прерывания по окончании отправки
        return -1;

    if (sx127x_set_standby(dev) != 0) // FIFO не доступен в режиме SLEEP
        return -1;

    if (sx127x_write_fifo(dev, buffer, size) != 0) // Загрузка FIFO
        return -1;

    if (sx127x_set_tx(dev) != 0) // Начало отправки
        return -1;

    return 0;
}

uint8_t sx127x_receive_single(sx127x_dev_t* dev, uint8_t* buffer, uint8_t* size)
{
    if (sx127x_set_dio_config(dev, (DIO_MODE_DISABLE << DIO_0_MAPPING) | (DIO_MODE_DISABLE << DIO_1_MAPPING) | (DIO_MODE_DISABLE << DIO_2_MAPPING) | (DIO_MODE_DISABLE << DIO_3_MAPPING) | (DIO_MODE_DISABLE << DIO_4_MAPPING) | (DIO_MODE_DISABLE << DIO_5_MAPPING)) != 0) // Отключение DIO
        return -1;

    if (sx127x_clear_irq_flags(dev, FlagPayloadCrcError | FlagRxDone | FlagRxTimeout) != 0) // На всякий случай очистра прерываний
        return -1;

    if (sx127x_set_irq_flags_mask(dev, FlagPayloadCrcError | FlagRxDone | FlagRxTimeout) != 0) // Включение прерывания по окончании отправки
        return -1;

    if (sx127x_set_standby(dev) != 0) // FIFO не доступен в режиме SLEEP
        return -1;

    if (sx127x_set_fifo_rx_pointer(dev, 0) != 0)
        return -1;

    if (sx127x_set_rx_single(dev) != 0)
        return -1;

    while (1) {
        uint8_t irq = sx127x_get_irq_flags(dev);

        if (irq & FlagRxDone) {
            if (!sx127x_get_crc_valid(dev)) {
                if (sx127x_clear_irq_flags(dev, FlagPayloadCrcError) != 0)
                    return -1;

                return 1;
            }

            *size = sx127x_get_last_packet_size(dev);
            if (sx127x_read_fifo(dev, buffer, *size) != 0)
                return -1;

            if (sx127x_clear_irq_flags(dev, FlagRxDone) != 0)
                return -1;

            return 0;
        } else if (irq & FlagRxTimeout) {
            if (sx127x_clear_irq_flags(dev, FlagRxTimeout) != 0)
                return -1;

            return 2;
        }
    }
}

uint8_t sx127x_receive_single_it(sx127x_dev_t* dev)
{
    if (sx127x_set_dio_config(dev, (DIO_MODE_0 << DIO_0_MAPPING) | (DIO_MODE_0 << DIO_1_MAPPING) | (DIO_MODE_DISABLE << DIO_2_MAPPING) | (DIO_MODE_2 << DIO_3_MAPPING) | (DIO_MODE_DISABLE << DIO_4_MAPPING) | (DIO_MODE_DISABLE << DIO_5_MAPPING)) != 0) // Включение DIO0, DIO1 и DIO3
        return -1;

    if (sx127x_clear_irq_flags(dev, FlagPayloadCrcError | FlagRxDone | FlagRxTimeout) != 0) // На всякий случай очистра прерываний
        return -1;

    if (sx127x_set_irq_flags_mask(dev, FlagPayloadCrcError | FlagRxDone | FlagRxTimeout) != 0) // Включение прерывания по окончании отправки
        return -1;

    if (sx127x_set_standby(dev) != 0) // FIFO не доступен в режиме SLEEP
        return -1;

    if (sx127x_set_fifo_rx_pointer(dev, 1) != 0)
        return -1;

    if (sx127x_set_rx_single(dev) != 0)
        return -1;

    return 0;
}

int8_t sx127x_receive_continuous(sx127x_dev_t* dev, uint8_t* buffer, uint8_t* size)
{
    if (dev->settings.mode != MODE_RXCONTINUOUS) {

        if (sx127x_set_dio_config(dev, (DIO_MODE_DISABLE << DIO_0_MAPPING) | (DIO_MODE_DISABLE << DIO_1_MAPPING) | (DIO_MODE_DISABLE << DIO_2_MAPPING) | (DIO_MODE_DISABLE << DIO_3_MAPPING) | (DIO_MODE_DISABLE << DIO_4_MAPPING) | (DIO_MODE_DISABLE << DIO_5_MAPPING)) != 0) // Отключение DIO

            if (sx127x_clear_irq_flags(dev, FlagPayloadCrcError | FlagRxDone | FlagRxTimeout) != 0) // На всякий случай очистра прерываний
                return -1;

        if (sx127x_set_irq_flags_mask(dev, FlagPayloadCrcError | FlagRxDone | FlagRxTimeout) != 0) // Включение прерывания по окончании отправки
            return -1;

        if (sx127x_set_standby(dev) != 0) // FIFO не доступен в режиме SLEEP
            return -1;

        if (sx127x_set_fifo_rx_pointer(dev, 0) != 0)
            return -1;

        if (sx127x_set_rx_continuos(dev) != 0)
            return -1;
    }

    while (1) {
        uint8_t irq = sx127x_get_irq_flags(dev);

        if (irq & FlagRxDone) {
            if (!sx127x_get_crc_valid(dev)) {
                if (sx127x_clear_irq_flags(dev, FlagPayloadCrcError) != 0)
                    return -1;

                return 1;
            }

            *size = sx127x_get_last_packet_size(dev);
            if (sx127x_read_fifo(dev, buffer, *size) != 0)
                return -1;

            if (sx127x_clear_irq_flags(dev, FlagRxDone) != 0)
                return -1;

            return 0;
        } else if (irq & FlagRxTimeout) {
            if (sx127x_clear_irq_flags(dev, FlagRxTimeout) != 0)
                return -1;

            return 2;
        }
    }
}

uint8_t sx127x_receive_continuous_it(sx127x_dev_t* dev)
{
    if (sx127x_set_dio_config(dev, (DIO_MODE_0 << DIO_0_MAPPING) | (DIO_MODE_0 << DIO_1_MAPPING) | (DIO_MODE_DISABLE << DIO_2_MAPPING) | (DIO_MODE_2 << DIO_3_MAPPING) | (DIO_MODE_DISABLE << DIO_4_MAPPING) | (DIO_MODE_DISABLE << DIO_5_MAPPING)) != 0) // Включение DIO0, DIO1 и DIO3
        return -1;

    if (sx127x_clear_irq_flags(dev, FlagPayloadCrcError | FlagRxDone | FlagRxTimeout) != 0) // На всякий случай очистра прерываний
        return -1;

    if (sx127x_set_irq_flags_mask(dev, FlagPayloadCrcError | FlagRxDone | FlagRxTimeout) != 0) // Включение прерывания по окончании отправки
        return -1;

    if (sx127x_set_standby(dev) != 0) // FIFO не доступен в режиме SLEEP
        return -1;

    if (sx127x_set_fifo_rx_pointer(dev, 0) != 0)
        return -1;

    if (sx127x_set_rx_continuos(dev) != 0)
        return -1;

    return 0;
}
