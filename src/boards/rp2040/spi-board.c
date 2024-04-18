/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "spi-board.h"

spi_inst_t *getPicoSpi(SpiId_t spiId)
{
    return (spiId == SPI_1) ? spi0 : spi1;
}

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
    spi_inst_t *spi = getPicoSpi(spiId);
    printf("Intializing SPI%d\n", spi_get_index(spi));
    spi_init(spi, 15000);
    spi_set_format((spiId == 0) ? spi0 : spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(miso, GPIO_FUNC_SPI);
    gpio_set_function(sclk, GPIO_FUNC_SPI);

    obj->SpiId = spiId;
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    spi_inst_t *spi = getPicoSpi(obj->SpiId);
    const uint8_t outDataB = (outData & 0xff);
    uint8_t inDataB = 0x00;

    spi_write_read_blocking(spi, &outDataB, &inDataB, 1);

    return inDataB;
}
