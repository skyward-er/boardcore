/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Radu Raul
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

#pragma once
#include <sensors/Sensor.h>
#include <sensors/H3LIS331DL/H3LIS331DLData.h>

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>

#include <miosix.h>

#define SETBITS(var, bitpos, mask, value) var |= (value << bitpos) & mask

namespace Boardcore {

  class H3LIS331DL : Sensor<H3LIS331DLData> {
    /* Class Data Types */
  public:
    enum class Registers {
      WHO_AM_I = 0x07,
      CTRL_REG1 = 0x20,
      CTRL_REG2 = 0x21,
      CTRL_REG3 = 0x22,
      CTRL_REG4 = 0x23,
      CTRL_REG5 = 0x24,
      OUT_X = 0x28,
      OUT_Y = 0x2a,
      OUT_Z = 0x2c
    }

    enum class FullScaleRange {
      FS_100 = 0,
      FS_200 = 1,
      FS_400 = 3
    }

    enum class OutputDataRate {
      ODR_LP_0_5 = 0,
      ODR_LP_1 = 1,
      ODR_LP_2 = 2,
      ODR_LP_5 = 3,
      ODR_LP_10 = 4,
      ODR_50 = 5,
      ODR_100 = 6,
      ODR_400 = 7,
      ODR_1000 = 8
    }

    enum class BlockDataUpdate {
      BDU_CONTINUOS_UPDATE = 0,
      BDU_WAIT_UNTIL_READ = 1
    }

    /* Class Members */
  private:
    static const WHO_AM_I_ID = 0x32;

    SPISlave spi;
    FullScaleRange fs;
    OutputDataRate odr;
    BlockDataUpdate bdu;
    bool initialized;

    /* Class Methods */
  public:


    H3LIS331DL(
      SPIBusInterface& spiBus,
      miosix::GpioPin cs,
      OutputDataRate odr,
      BlockDataUpdate bdu
    );

    H3LIS331DL(
      SPIBusInterface& spiBus,
      SPIBusConfig cfg,
      miosix::GpioPin cs,
      OutputDataRate odr,
      BlockDataUpdate bdu
    );

    bool init();

    H3LIS331DLData sampleImpl() override;

  }
}
