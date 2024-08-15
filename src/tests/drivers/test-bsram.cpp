/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Niccolò Betto
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

#include <arch/common/drivers/stm32_bsram.h>
#include <miosix.h>

#include <iostream>

int PRESERVE preservedVariable;

std::string to_string(miosix::ResetReason reason)
{
    switch (reason)
    {
        case miosix::ResetReason::UNKNOWN:
            return "UNKNOWN";
        case miosix::ResetReason::LOW_POWER:
            return "LOW_POWER";
        case miosix::ResetReason::WINDOW_WATCHDOG:
            return "WINDOW_WATCHDOG";
        case miosix::ResetReason::INDEPENDENT_WATCHDOG:
            return "INDEPENDENT_WATCHDOG";
        case miosix::ResetReason::SOFTWARE:
            return "SOFTWARE";
        case miosix::ResetReason::POWER_ON:
            return "POWER_ON";
        case miosix::ResetReason::PIN:
            return "PIN";
    }
}

void printInfo()
{
    std::cout << "preservedVariable: " << preservedVariable << "\n";
    std::cout << "resetReason: " << to_string(miosix::lastResetReason())
              << "\n";
    std::cout.flush();
}

int main()
{
    while (true)
    {
        printInfo();

        int option;
        std::cout << "Select option"
                  << " (1 = set preserved variable; 2 = software reset): ";
        std::cin >> option;

        switch (option)
        {
            case 1:
                int userInput;
                std::cout << "Input the new value of the preserved variable"
                          << std::endl;
                std::cin >> userInput;

                miosix::BSRAM::enableWrite();
                preservedVariable = userInput;
                miosix::BSRAM::disableWrite();

                break;

            case 2:
                miosix::reboot();
                break;

            default:
                std::cout << "Invalid option: " << option << std::endl;
                break;
        }
    }

    return 0;
}
