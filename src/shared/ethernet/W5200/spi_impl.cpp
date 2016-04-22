/*
 * Copyright (C) 2015  Silvano Seva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "spi_impl.h"

using namespace miosix;

void Spi_init()
{
    eth::sck::mode(Mode::ALTERNATE);
    eth::sck::alternateFunction(5);
    
    eth::miso::mode(Mode::ALTERNATE);
    eth::miso::alternateFunction(5);
    
    eth::mosi::mode(Mode::ALTERNATE);
    eth::mosi::alternateFunction(5);

    eth::cs::mode(Mode::OUTPUT);
    
    
    RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
    RCC_SYNC();

    //APB bus frequency is 90MHz, so leaving the BR[2:0] bits
    //in CR1 makes the SPI clock frequency to be 45MHz
    
    SPI5->CR1 = SPI_CR1_SSM //CS handled in software
              | SPI_CR1_SSI //Internal CS high
              | SPI_CR1_SPE //SPI enabled
              | SPI_CR1_MSTR; //Master mode
}

unsigned char Spi_sendRecv(unsigned char data)
{
    SPI5->DR = data;
    while ((SPI5->SR & SPI_SR_RXNE) == 0); //Wait
    return SPI5->DR;
}

void Spi_CS_high()
{
    eth::cs::high();
}

void Spi_CS_low()
{
    eth::cs::low();
}
