/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Silvano Seva, Alberto Nidasio
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

namespace i2c
{
using scl = Gpio<GPIOB_BASE, 8>;
using sda = Gpio<GPIOB_BASE, 9>;
}  // namespace i2c

namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

namespace can
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can
}  // namespace interfaces

namespace sensors
{

namespace adis16405
{
using cs   = Gpio<GPIOA_BASE, 8>;
using dio1 = Gpio<GPIOB_BASE, 4>;
using dio2 = Gpio<GPIOD_BASE, 6>;
using dio3 = Gpio<GPIOD_BASE, 4>;
using rst  = Gpio<GPIOD_BASE, 5>;
using ckIn = Gpio<GPIOA_BASE, 15>;
}  // namespace adis16405

namespace ad7994
{
using ab                      = Gpio<GPIOB_BASE, 1>;
using nconvst                 = Gpio<GPIOG_BASE, 9>;
static constexpr uint8_t addr = 0x22 << 1;
}  // namespace ad7994

namespace lm75b_analog
{
static constexpr uint8_t addr = 0x48 << 1;
}  // namespace lm75b_analog

namespace lm75b_imu
{
static constexpr uint8_t addr = 0x49 << 1;
}  // namespace lm75b_imu

namespace mpu9250
{
using cs   = Gpio<GPIOC_BASE, 3>;
using intr = Gpio<GPIOE_BASE, 5>;
}  // namespace mpu9250

namespace ms5803
{
using cs = Gpio<GPIOD_BASE, 7>;
}  // namespace ms5803

namespace lsm6ds3h
{
using cs   = Gpio<GPIOC_BASE, 1>;
using int1 = Gpio<GPIOB_BASE, 12>;
using int2 = Gpio<GPIOG_BASE, 3>;
}  // namespace lsm6ds3h

}  // namespace sensors

namespace inputs
{
using vbat      = Gpio<GPIOF_BASE, 7>;
using lp_dtch   = Gpio<GPIOC_BASE, 6>;  // launchpad detach
using btn_open  = Gpio<GPIOG_BASE, 11>;
using btn_close = Gpio<GPIOG_BASE, 13>;
}  // namespace inputs

namespace nosecone
{
using motEn   = Gpio<GPIOG_BASE, 14>;
using motP1   = Gpio<GPIOC_BASE, 7>;   // Pwm motore 1
using motP2   = Gpio<GPIOB_BASE, 0>;   // Pwm motore 2
using rogP1   = Gpio<GPIOD_BASE, 12>;  // Pwm rogallina 1
using rogP2   = Gpio<GPIOD_BASE, 13>;  // Pwm rogallina 2
using nc_dtch = Gpio<GPIOB_BASE, 7>;   // Nosecone detach

using motor_act = Gpio<GPIOE_BASE, 3>;   // Motor actuation
using motor_3v  = Gpio<GPIOC_BASE, 15>;  // Motor actuation 3v
using motor_gnd = Gpio<GPIOE_BASE, 2>;   // Motor actuation gnd
}  // namespace nosecone

namespace actuators
{

using tcPwm = Gpio<GPIOE_BASE, 6>;  // Pwm thermal cutters

namespace thCut2
{
using ena   = Gpio<GPIOD_BASE, 11>;
using csens = Gpio<GPIOF_BASE, 8>;
}  // namespace thCut2

namespace thCut1
{
using ena   = Gpio<GPIOG_BASE, 2>;
using csens = Gpio<GPIOF_BASE, 6>;
}  // namespace thCut1
}  // namespace actuators

namespace misc
{
using buzz = Gpio<GPIOD_BASE, 3>;

using aux1 = Gpio<GPIOE_BASE, 2>;
using aux2 = Gpio<GPIOE_BASE, 3>;
using aux3 = Gpio<GPIOE_BASE, 4>;
using aux4 = Gpio<GPIOC_BASE, 14>;
using aux5 = Gpio<GPIOC_BASE, 15>;

}  // namespace misc

using led1 = Gpio<GPIOC_BASE, 4>;

/* NOTE: Direct access to these leds is possible
 * only when the STM board is detached from
 * the rest of the stack
 */
/*
using led2  = Gpio<GPIOG_BASE, 2>;
using led3  = Gpio<GPIOD_BASE, 11>;
using led4  = Gpio<GPIOB_BASE, 1>;
using led1  = Gpio<GPIOG_BASE, 3>;
*/

namespace xbee
{
using cs           = Gpio<GPIOF_BASE, 9>;
using attn         = Gpio<GPIOF_BASE, 10>;
using reset        = Gpio<GPIOC_BASE, 13>;
using sleep_req    = Gpio<GPIOC_BASE, 2>;
using sleep_status = Gpio<GPIOA_BASE, 4>;
}  // namespace xbee
}  // namespace miosix

#endif  // HWMAPPING_H
