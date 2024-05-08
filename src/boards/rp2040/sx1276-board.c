/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation
 *
 * \remark    This is based on
 *            https://github.com/Lora-net/LoRaMac-node/blob/master/src/boards/B-L072Z-LRWAN1/sx1276-board.c
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 */

#include <stddef.h>

#include "hardware/gpio.h"
#include "pico/util/queue.h"

#include "delay.h"
#include "sx1276-board.h"

#include "radio/radio.h"

#define RADIO_DIO0_GPIO_IRQ     GPIO_IRQ_EDGE_RISE
#define RADIO_DIO1_GPIO_IRQ     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL

void SX1276ServiceIrqHandlers( void );

const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime,
    .IrqProcess = SX1276ServiceIrqHandlers, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

static DioIrqHandler** irq_handlers;
static queue_t dio0_irq_queue;
static queue_t dio1_irq_queue;

void __not_in_flash_func(dio_gpio_callback)(void)
{
    uint events;
    if ((events = gpio_get_irq_event_mask(SX1276.DIO0.pin)) & RADIO_DIO0_GPIO_IRQ) {
        gpio_acknowledge_irq(SX1276.DIO0.pin, RADIO_DIO0_GPIO_IRQ);
        queue_try_add(&dio0_irq_queue, &events);
    }
    if ((events = gpio_get_irq_event_mask(SX1276.DIO1.pin)) & RADIO_DIO1_GPIO_IRQ) {
        gpio_acknowledge_irq(SX1276.DIO1.pin, RADIO_DIO1_GPIO_IRQ);
        queue_try_add(&dio1_irq_queue, &events);
    }
}

void SX1276ServiceIrqHandlers( void )
{
    uint events;
    if(queue_try_remove(&dio0_irq_queue, &events)) {
        irq_handlers[0](NULL);
    }
    if(queue_try_remove(&dio1_irq_queue, &events)) {
        irq_handlers[1](NULL);
    }
}

void SX1276SetAntSwLowPower( bool status )
{
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    return true;
}

void SX1276SetBoardTcxo( uint8_t state )
{
}

uint32_t SX1276GetDio1PinState( void )
{
    return GpioRead(&SX1276.DIO1);
}

void SX1276SetAntSw( uint8_t opMode )
{
}

void SX1276Reset( void )
{
    GpioInit( &SX1276.Reset, SX1276.Reset.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 ); // RST

    DelayMs (1);

    GpioInit( &SX1276.Reset, SX1276.Reset.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 ); // RST

    DelayMs (6);
}

void SX1276IoInit( void )
{
    GpioInit( &SX1276.Spi.Nss, SX1276.Spi.Nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 ); // CS
    GpioInit( &SX1276.Reset, SX1276.Reset.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );     // RST

    GpioInit( &SX1276.DIO0, SX1276.DIO0.pin, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );        // IRQ / DIO0
    GpioInit( &SX1276.DIO1, SX1276.DIO1.pin, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );        // DI01
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
    irq_handlers = irqHandlers;
    queue_init(&dio0_irq_queue, sizeof(uint), 4);
    queue_init(&dio1_irq_queue, sizeof(uint), 4);

    gpio_set_irq_enabled(SX1276.DIO0.pin, RADIO_DIO0_GPIO_IRQ, true);
    gpio_set_irq_enabled(SX1276.DIO1.pin, RADIO_DIO1_GPIO_IRQ, true);
    irq_add_shared_handler(IO_IRQ_BANK0, dio_gpio_callback, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);

}

void SX1276SetRfTxPower( int8_t power )
{
    SX1276Write( REG_PACONFIG, 0xff ); //max power ?
    SX1276Write( REG_PADAC, 0x87 ); // Enables the +20dBm option on PA_BOOST pin
}

uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    return 0;
}
