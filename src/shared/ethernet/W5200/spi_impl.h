/*
 * This is a little set of functions that create an interface layer between
 * the driver and the target hardware / system SPI bus handling mechanism
 * Please edit spi_impl.cpp and NOT this file
 * 
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

#ifndef SPI_IMPL_H
#define SPI_IMPL_H

#include <Common.h>

void Spi_init();

unsigned char Spi_sendRecv(unsigned char data);

void Spi_CS_high();

void Spi_CS_low();

#endif // SPI_IMPL_H
