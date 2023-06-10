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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <diagnostic/PrintLogger.h>
#include <sensors/MBLoadCell/MBLoadCell.h>
#include <string.h>
#include <utils/ButtonHandler/ButtonHandler.h>

// #define PRINT_ALL_SAMPLES // To be defined if we want to print all the
// samples

using namespace Boardcore;
using namespace miosix;
using namespace std;
using namespace placeholders;

using button            = miosix::Gpio<GPIOA_BASE, 0>;  ///< User button
const uint8_t btnUserId = 1;

/**
 * @brief Callback function that is called when the button is pressed. When the
 * button is pressed for a long time, resets the minimum and maximum values of
 * the recorded weights
 */
void buttonCallback(ButtonEvent btnPress, MBLoadCell *loadcell)
{
    if (btnPress == ButtonEvent::PRESSED)
        TRACE(
            "## MAX: %.2f [Kg] (ts: %.3f)\t##\tMIN: %.2f [Kg] (ts: %.3f) ##\n",
            loadcell->getMaxWeight().load,
            loadcell->getMaxWeight().loadTimestamp / 1000000.0,
            loadcell->getMinWeight().load,
            loadcell->getMinWeight().load / 1000000.0);

    if (btnPress == ButtonEvent::LONG_PRESS)
        loadcell->resetMaxMinWeights();
}

int main()
{
    // In order to disable DEBUG prints of the button press events
    Logging::getStdOutLogSink().setLevel(LOGL_WARNING);
    USART usart(USART1, 115200);

    /**
     * Use of CONT_MOD_TD: transmits net and gross weight
     * - use of serial port 3: in stm32f407vg TX=PB10, RX=PB11 (can't be used in
     * stm32f407vg, is the default serial port)
     * - use of serial port 2: in stm32f407vg TX=PA2, RX=PA3
     * - use of serial port 1: in stm32f407vg TX=PA9, RX=PA10
     */
    MBLoadCell loadCell(usart, LoadCellModes::CONT_MOD_T);

    // Instanciating the button
    ButtonHandler::getInstance().registerButtonCallback(
        button::getPin(), bind(buttonCallback, _1, &loadCell));

    // Initializing the load cell
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
