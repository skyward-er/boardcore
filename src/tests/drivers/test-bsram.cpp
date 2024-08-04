/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <drivers/bsram/BSRAM.h>

#include <iostream>

using namespace Boardcore;
using namespace miosix;

class BSRAMImpl : public BSRAM
{
public:
    BSRAMImpl() : BSRAM()
    {
        switch (lastResetReason())
        {
            case ResetReason::RST_POWER_ON:
                setTemp(0);
                pwrOn = true;
                break;
            case ResetReason::RST_PIN:
                btnRst = true;
                break;
            case ResetReason::RST_SW:
                swRst = true;
                break;
            default:
                other = true;
                break;
        }
    }

    void printStuff()
    {
        std::cout << "temp: " << getTemp() << std::endl;
        std::cout << "resetReason: " << (int)lastResetReason() << std::endl;
        std::cout << "pwrOn: " << pwrOn << std::endl;
        std::cout << "btnRst: " << btnRst << std::endl;
        std::cout << "swRst: " << swRst << std::endl;
        std::cout << "other: " << other << std::endl;
    }

    static void setTemp(int tempNew)
    {
        enableWrite();
        temp = tempNew;
        disableWrite();
    }

    static int getTemp() { return temp; }

private:
    bool pwrOn  = false;
    bool btnRst = false;
    bool swRst  = false;
    bool other  = false;

    static int PRESERVE temp;
};

int PRESERVE BSRAMImpl::temp;

int main()
{
    BSRAMImpl bsram;
    int option;

    while (true)
    {
        bsram.printStuff();
        std::cout << "select option (1 = set number; 2 = software reset): ";
        std::cin >> option;

        switch (option)
        {
            case 1u:
                int temp;
                std::cout << "Input the new value of temp" << std::endl;
                std::cin >> temp;
                bsram.setTemp(temp);
                break;
            case 2u:
                reboot();
                break;
            default:
                std::cout << "Invalid option: " << option << std::endl;
                break;
        }
    }

    return 0;
}
