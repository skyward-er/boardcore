/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor, Emilio Corigliano
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
#include <cstdio>
#include <cstdint>
#include <thread>
#include <interfaces-impl/bsp_impl.h>
#include <miosix.h>


using namespace miosix;

void banner() {
    printf(R"(
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡀⠀⠀⠀⠀⠀⠀⠀⢀⣠⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⢶⣿⢃⣠⣴⣶⣶⣿⣛⢯⣹⣾⠁⠀⠀⠀⠀⠀⢀⣀⣤⣴⣶⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⣿⣿⣿⣻⣾⣿⣞⣯⣷⣽⣾⣿⣁⣀⣠⢤⡶⣶⠿⣻⣶⣟⡿⣿⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣟⣯⣿⣶⣯⣿⣿⣿⡁⠀⣴⠿⣟⣿⣟⣿⣻⢷⢯⣞⡏⠀⠀
⠀⠀⠀⢠⡶⢒⣖⣲⡶⢶⡖⣶⣒⢶⡲⣞⣫⠭⣭⣭⣽⣭⣯⣽⣿⣿⣿⣿⣿⣿⣿⠿⢿⡿⠿⠿⠿⣌⠿⢿⣿⣿⣿⣿⡾⣽⢯⣟⣿⠃⠀⠀
⠀⠀⠀⠀⢻⣯⣛⢧⣛⢧⣛⡶⣹⣎⢷⣝⡮⣟⢷⡾⣹⢯⡟⠉⠉⠛⢿⣿⣿⣯⣟⣿⣫⣽⡿⣃⣔⣬⣿⣷⣯⣿⣿⣿⣿⣽⣻⣾⡏⠀⠀⠀
⠀⠀⠀⠀⠀⠹⣟⡾⣭⡟⣾⣹⢗⣾⣫⢾⡽⣽⢾⡽⢯⡟⠀⠀⠀⢀⢈⣿⣿⣿⣿⣿⠿⢻⣷⣼⣿⣿⣟⣿⣟⣿⣿⣿⣿⣿⣿⠏⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠙⣿⣷⣿⣷⣯⣟⡾⣽⣳⢿⡽⣾⡽⡟⠀⠀⢀⣰⠞⠟⠙⢻⣷⣿⣸⣷⣾⣟⣺⣽⣟⣻⣿⣿⣿⣿⣿⣿⣿⠋⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⣿⣿⣿⣿⣷⣯⣿⣽⣳⢿⠃⠀⠀⢀⠀⢠⣤⣦⣄⣇⠉⠛⠛⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠻⣿⣿⣿⣿⣿⣉⠉⠉⠻⠀⠀⠠⢀⣾⢿⠋⢁⣿⣿⠀⠀⠀⠀⣿⣧⠙⢿⣹⣏⢠⣛⠟⠁⠀⠀⢀⣴⣷⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⠿⣿⣿⣿⣓⣈⣰⣦⣤⣀⣼⠋⠀⠀⢭⣽⣿⡇⠀⣀⣀⣿⣿⡇⡸⣿⣿⡤⠉⠒⠤⣀⠀⠠⡄⠀⣀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⢹⠏⠀⠛⣞⣧⣾⠍⠢⣀⠀⠈⠻⠟⣇⢉⣡⡶⣭⠯⡀⠐⠛⠙⢇⠀⠐⢲⠏⠀⠀⠈⠲⠞⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⠁⢀⠄⠂⢸⠛⠁⣠⣲⡆⢔⠾⠋⠀⠈⠢⣉⠀⢀⣀⣈⡆⠀⠀⢸⣀⠔⠋⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠚⠤⣌⡁⠀⠀⠈⢆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠹⣿⣿⣿⡿⠀⠀⣜⣩⠟⠃⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠼⣛⡉⠀⠀⠓⢄⡀⠀⠀⠀⠀⠀⠀⢀⡀⠀⠀⡹⠋⠀⣠⠞⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠐⠒⠚⣿⣶⠦⣄⣀⡀⠘⠿⠽⠶⢾⣷⣀⠖⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣀⡀⣀⣀⣠⡿⠍⣛⠻⡿⠿⣟⣲⣶⣶⡶⠿⣿⣖⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠼⠛⢻⠿⠻⢿⣿⣷⣿⣶⣆⠬⡯⣑⣿⣿⣿⣿⣹⣍⣧⣾⠈⠉⠉⠒⠒⠠⣄⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠋⠀⠀⠐⠒⠠⣔⠁⠀⠉⣿⣿⣿⢛⡖⠾⠿⢿⡿⠿⣾⠟⠁⠀⠀⠀⠀⠀⠸⠀⠱⡄⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠰⡅⠀⠀⠀⠀⠀⠀⠀⠉⠒⠄⠘⢿⣿⠇⠈⠁⠒⠺⣤⣢⡼⠀⠀⠀⠀⠀⠀⠀⠸⠀⠀⠘⣆⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠹⡄⠐⠒⠠⠤⠀⣀⠀⠀⠀⠀⠈⣏⠀⠀⠀⠀⠀⠈⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⢆⠀⠀⠙⣆⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡰⠁⠀⠘⣆⠀⠀⠀⠀⠀⠉⠑⠢⡀⢀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⠀⠀⠀⠈⢆⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡰⠁⠀⠀⠀⠈⢣⡀⠠⢄⣀⠀⠀⠀⢠⠞⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠄⠀⠀⠀⠀⢣⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠰⠧⣀⣀⠀⢀⡤⠋⠀⠀⠀⠀⠈⢁⠒⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡆⠀⢀⣠⢾⡍⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠞⣻⡿⠏⠀⠀⠀⠐⠢⠄⢀⡀⣰⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡷⠖⠋⣁⡼⣇⠀ 

 Welcome to the
   ________  __   _    _____      ______          __             _ __     
  / ____/ / / /  | |  / /__ \    /_  __/__  _____/ /________  __(_) /____ 
 / /   / / / /   | | / /__/ /     / / / _ \/ ___/ __/ ___/ / / / / __/ _ \
/ /___/ /_/ /    | |/ // __/     / / /  __(__  ) /_(__  ) /_/ / / /_/  __/
\____/\____/     |___//____/    /_/  \___/____/\__/____/\__,_/_/\__/\___/ 
)");
}

void led_task() {
    ledOff();
    
    const unsigned int DELAY = 300;
    while(1) {
        led2Off();
        led1On();
        Thread::sleep(DELAY);
        led1Off();
        led2On();
        Thread::sleep(DELAY);
        led2Off();
        led3On();
        Thread::sleep(DELAY);
        led3Off();
        led4On();
        Thread::sleep(DELAY);
        led4Off();
        led3On();
        Thread::sleep(DELAY);
        led3Off();
        led2On();
        Thread::sleep(DELAY);
    }
}

void xram_test() {
    volatile uint16_t* const START = (volatile uint16_t*)0xd0000000;
    volatile uint16_t* const END = START + (16 * 1024 * 1024);

    volatile uint16_t *iter = START;
    // First clear the whole RAM
    while(iter != END) {
        *iter = 0;
        iter++;
    }

    __DMB(); // Flush cache
    
    printf("*** Starting basic XRAM test...\n");
    bool ok = true;

    iter = START;
    while(iter != END) {
        // Set a marker value at this address
        *iter = 0xffff;
        __DMB(); // Flush cache

        if(*iter != 0xffff) {
            ok = false;
            printf("Readback failed: %p\n", iter);
        }

        // Reset the value back to 0
        *iter = 0;
        __DMB(); // Flush cache

        iter++;
    } 

    if(ok) {
        printf("*** XRAM basic test succesfull!\n");
    } else {
        printf("*** XRAM basic test failed!\n");
    }

    printf("*** Starting XRAM mirroring test...\n");
    ok = true;

    // Then check for mirroring for every short
    iter = START;
    while(iter != END) {
        // Set a marker value at this address
        *iter = 0xdead;
        __DMB(); // Flush cache

        // Generate possible alias addresses
        for(size_t i = 1; i < sizeof(iter); i++) {
            // Change just one bit, and see if it alias
            volatile uint16_t *other = (volatile uint16_t *)((size_t)iter ^ (1 << i));

            // Skip invalid addresses
            if(other < START || other >= END) {
                continue;
            }

            // Check if they somehow alias
            if(*iter == *other) {
                ok = false;
                printf("Alias: %p -> %p\n", iter, other);
            }
            
            __DMB(); // Flush cache
        }

        // Reset the value back to 0
        *iter = 0;
        __DMB(); // Flush cache
        
        iter++;
    }

    if(ok) {
        printf("*** XRAM mirror test succesfull!\n");
    } else {
        printf("*** XRAM mirror test failed!\n");
    }
}

int main() {
    banner();

    // Spawn the led blinker task
    std::thread t(led_task);

    while(1) {
        printf(
            "\nSelect test to run:\n"
            "- 1 Single pin test\n"
            "- 2 Automatic SD test\n"
            "- 3 Automatic RAM test\n"
            "- 4 Automatic ELC guys can't properly solder(tm) test\n"
            "- 5 Semi-automatic pin test\n"
            "\n"
            "Command: "
        );

        int cmd;
        scanf("%d", &cmd);

        switch(cmd) {
        case 3:
            xram_test();
            break;
        }
    }

    return 0;
}