/* Copyright (c) 2015-2019 Skyward Experimental Rocketry
 * Authors: Benedetta Margrethe Cattani
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Common.h>
#include <interfaces-impl/hwmapping.h>
using namespace miosix;
using namespace interfaces;
using namespace actuators;

using rena   = Gpio<GPIOG_BASE, 2>;

void enable_reverse(){
  hbridger::in::high();
  hbridgel::in::low();
  rena::high();
  hbridgel::ena::high();
}

void enable_direct(){
  hbridger::in::low();
  hbridgel::in::high();
  rena::high();
  hbridgel::ena::high();
}

void disable(){
  hbridger::in::low();
  hbridgel::in::low();
  rena::low();
  hbridgel::ena::low();
}


int main()
{
    rena::mode(Mode::OUTPUT);
    hbridger::in::mode(Mode::OUTPUT);
    hbridgel::in::mode(Mode::OUTPUT);


    while (true)
    {
printf("Serial is working!\n");

      Thread::sleep(1000);
      disable();
      printf("Disabled\n");
      Thread::sleep(2000);
      enable_direct();
      printf("Direct\n");
      Thread::sleep(3000);
      enable_reverse();
      printf("Reverse\n");
      Thread::sleep(3000);
      disable();
      printf("Disabled\n");


    }
}
