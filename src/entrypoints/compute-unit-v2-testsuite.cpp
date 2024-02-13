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
    
    const unsigned int DELAY = 200;
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

    
    // First clear the whole RAM
    for(volatile uint16_t *iter = START; iter != END; iter++) {
        *iter = 0;
    }

    __DMB(); // Flush cache
    
    printf("*** Starting basic XRAM test...\n");
    bool ok = true;

    for(volatile uint16_t *iter = START; iter != END; iter++) {
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
    } 

    if(ok) {
        printf("*** XRAM basic test succesfull!\n");
    } else {
        printf("*** XRAM basic test failed!\n");
    }

    printf("*** Starting XRAM mirroring test...\n");
    ok = true;

    // Generate addresses to test every line
    for(int i = 1; i < 25; i++) {
        volatile uint16_t *other = (volatile uint16_t*)((size_t)START | (1 << i));

        // Write something
        *other = 0xdead;
        __DMB(); // Flush cache

        for(volatile uint16_t *iter = START; iter != END; iter++) {
            // Skip the written address
            if(iter == other) {
                continue;
            }

            // Check for mirroring
            if(*iter == 0xdead) {
                printf("Mirroring %p -> %p\n", other, iter);
                ok = false;
            }
        }

        // Reset the value
        *other = 0;
        __DMB(); // Flush cache
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
            "- 1 Automatic SD test\n"
            "- 2 Automatic RAM test\n"
            "- 3 Automatic ELC guys can't properly solder(tm) test\n"
            "- 4 Semi-automatic pin test\n"
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