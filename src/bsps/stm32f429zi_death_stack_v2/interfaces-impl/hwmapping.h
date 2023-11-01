/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HWMAPPING_H
#define HWMAPPING_H

#include "interfaces/gpio.h"

namespace miosix
{

namespace interfaces
{

namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

namespace spi2
{
using sck  = Gpio<GPIOB_BASE, 13>;
using miso = Gpio<GPIOB_BASE, 14>;
using mosi = Gpio<GPIOB_BASE, 15>;
}  // namespace spi2

// USB UART
namespace uart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace uart1

// GPS UART
namespace uart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace uart2

namespace uart3
{
using tx = Gpio<GPIOB_BASE, 10>;
using rx = Gpio<GPIOB_BASE, 11>;
}  // namespace uart3

namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

// CAN1
namespace can
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can

// Servo motors timers
namespace timers
{
using tim4ch1 = Gpio<GPIOD_BASE, 12>;  // Servo 1
using tim8ch2 = Gpio<GPIOC_BASE, 7>;   // Servo 2
}  // namespace timers

using camMosfet = Gpio<GPIOC_BASE, 14>;

}  // namespace interfaces

namespace sensors
{

namespace ads1118
{
using cs = Gpio<GPIOB_BASE, 1>;
}  // namespace ads1118

namespace bmx160
{
using cs   = Gpio<GPIOA_BASE, 8>;
using intr = Gpio<GPIOE_BASE, 5>;
}  // namespace bmx160

namespace lsm9ds1
{
using cs_a_g   = Gpio<GPIOC_BASE, 1>;
using cs_m     = Gpio<GPIOC_BASE, 3>;
using intr_a_g = Gpio<GPIOB_BASE, 12>;
}  // namespace lsm9ds1

namespace lis3mdl
{
using cs = Gpio<GPIOG_BASE, 6>;
}  // namespace lis3mdl

namespace ms5803
{
using cs = Gpio<GPIOD_BASE, 7>;
}  // namespace ms5803

}  // namespace sensors

namespace inputs
{
using vbat      = Gpio<GPIOF_BASE, 7>;
using expulsion = Gpio<GPIOE_BASE, 4>;
}  // namespace inputs

namespace actuators
{

namespace servos
{
using servo1 = interfaces::timers::tim4ch1;
using servo2 = interfaces::timers::tim8ch2;
}  // namespace servos

namespace nosecone
{

using th_cut_input = Gpio<GPIOE_BASE, 6>;  // Input thermal cutters

namespace thermal_cutter_1
{
using enable      = Gpio<GPIOG_BASE, 2>;
using cutter_sens = Gpio<GPIOF_BASE, 6>;  // ADC3 CH4
}  // namespace thermal_cutter_1

namespace thermal_cutter_2
{
using enable      = Gpio<GPIOD_BASE, 11>;
using cutter_sens = Gpio<GPIOF_BASE, 8>;  // ADC3 CH6
}  // namespace thermal_cutter_2

}  // namespace nosecone

}  // namespace actuators

namespace aux
{
using sense_aux_1 = Gpio<GPIOE_BASE, 2>;
using aux_pd_pu   = Gpio<GPIOC_BASE, 5>;
using aux_spi1_cs = Gpio<GPIOG_BASE, 7>;
}  // namespace aux

namespace leds
{
using led_red1  = Gpio<GPIOG_BASE, 7>;
using led_red2  = Gpio<GPIOG_BASE, 10>;
using led_blue1 = Gpio<GPIOG_BASE, 14>;
using led_ring  = Gpio<GPIOE_BASE, 3>;

/**
 * These are connected to the enable pin of the thermal cutters and the cs of
 * the lis3mdl magnetometer.
 */
// using led_blue2  = Gpio<GPIOG_BASE, 2>;
// using led_green1 = Gpio<GPIOG_BASE, 6>;
// using led_green2 = Gpio<GPIOD_BASE, 11>;
}  // namespace leds

namespace xbee
{
using cs    = Gpio<GPIOF_BASE, 9>;
using attn  = Gpio<GPIOF_BASE, 10>;
using reset = Gpio<GPIOC_BASE, 13>;
}  // namespace xbee

}  // namespace miosix

#endif  // HWMAPPING_H
