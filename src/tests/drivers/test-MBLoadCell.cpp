/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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
 * IMPLIED, INCLUDING BUT NOT LI
 * MITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Common.h>

#include "sensors/MBLoadCell/MBLoadCell.h"
#include "string.h"
#include "utils/ButtonHandler.h"

using namespace Boardcore;
using namespace miosix;

using button              = miosix::Gpio<GPIOA_BASE, 0>;  ///< user button
const uint8_t btn_user_id = 1;

/**
 * @brief callback function that is called when the button is pressed. When the
 * button is pressed for a long time, resets the minimum and maximum values of
 * the recorded weights
 */
void buttonCallback(uint8_t btn_id, ButtonPress btn_press, MBLoadCell *loadcell)
{
    if (btn_id == btn_user_id && btn_press == ButtonPress::DOWN)
        printf(
            "### TIMESTAMP: %.3f [s], EX MAX: %.2f [Kg], EX MIN: %.2f [Kg] "
            "###\n",
            (double)TimestampTimer::getTimestamp() / 1000000,
            loadcell->getMaxWeight().data, loadcell->getMinWeight().data);

    if (btn_id == btn_user_id && btn_press == ButtonPress::LONG)
        loadcell->resetMaxMinWeights();
}

int main()
{
    // enabling the timestamps
    TimestampTimer::enableTimestampTimer();

    /*
     * use of CONT_MOD_TD: transmits net and gross weight
     * - use of serial port 3: in stm32f407vg TX=PB10, RX=PB11 (can't be used in
     * stm32f407vg, is the default serial port)
     * - use of serial port 2: in stm32f407vg TX=PA2, RX=PA3
     * - use of serial port 1: in stm32f407vg TX=PA9, RX=PA10
     */
    MBLoadCell loadCell(LoadCellModes::CONT_MOD_T, 2, 115200);

    // binding the load cell instance to the callback to be called
    std::function<void(uint8_t, ButtonPress)> callback =
        std::bind(buttonCallback, std::placeholders::_1, std::placeholders::_2,
                  &loadCell);

    // instanciating the button
    ButtonHandler<button> btn_handler(btn_user_id, callback);
    btn_handler.start();

    // initializing the load cell
    if (!loadCell.init())
    {
        TRACE("Initialization of the load cell failed\n");
        return 1;
    }

    loadCell.asciiRequest(LoadCellValuesEnum::SET_SETPOINT_1, 5);
    while (true)
    {
        loadCell.sample();
        loadCell.printData();
    }

    return 0;
}
