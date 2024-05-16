/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "hardware/gpio.h"

#include "gpio-board.h"

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
    obj->pin = pin;

    if( pin == NC )
    {
        return;
    }

    gpio_init(pin);

    if (mode == PIN_INPUT)
    {
        gpio_set_dir(pin, GPIO_IN);
    }
    else if (mode == PIN_OUTPUT)
    {
        gpio_set_dir(pin, GPIO_OUT);
    }

    if (config == PIN_PUSH_PULL)
    {
        if (type == PIN_NO_PULL)
        {
            gpio_disable_pulls(pin);
        }
        else if (type == PIN_PULL_UP)
        {
            gpio_pull_up(pin);
        }
        else if (type == PIN_PULL_DOWN)
        {
            gpio_pull_down(pin);
        }
    }

    if( mode == PIN_OUTPUT )
    {
        GpioMcuWrite( obj, value );
    }
}

__always_inline void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    gpio_put(obj->pin, value);
}

__always_inline uint32_t GpioMcuRead( Gpio_t *obj )
{
    return gpio_get(obj->pin);
}



void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    assert(false);
    // uint32_t event_mask = 0;
    // if(irqMode & IRQ_RISING_EDGE) event_mask |= GPIO_IRQ_EDGE_RISE;
    // if(irqMode & IRQ_FALLING_EDGE) event_mask |= GPIO_IRQ_EDGE_FALL;
    // if(irqMode & IRQ_RISING_FALLING_EDGE) event_mask |= GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_FALL;

    // gpio_set_irq_enabled_with_callback(obj->pin, GPIO_IRQ_EDGE_RISE, true, irqHandler);
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    gpio_set_irq_enabled(obj->pin, GPIO_IRQ_EDGE_RISE, false);
}
