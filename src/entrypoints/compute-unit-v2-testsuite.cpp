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
#include <interfaces-impl/bsp_impl.h>
#include <miosix.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

using namespace miosix;

struct PinDef
{
    GpioPin pin;
    const char *gpio_name;
    const char *conan_name;
    // ADRIANOOOOOOOOOOOOOOOO
    const char *breakout_v1_name;
};

std::array<PinDef, 53> PIN_DEFS{{
    {Gpio<GPIOA_BASE, 0>::getPin(), "PA0", "PIN57", "PIN13"},
    {Gpio<GPIOA_BASE, 1>::getPin(), "PA1", "PIN55", "PIN15"},
    {Gpio<GPIOA_BASE, 2>::getPin(), "PA2", "PIN62", "PIN8"},
    {Gpio<GPIOA_BASE, 3>::getPin(), "PA3", "PIN60", "PIN10"},
    {Gpio<GPIOA_BASE, 4>::getPin(), "PA4", "PIN58", "PIN12"},
    {Gpio<GPIOA_BASE, 5>::getPin(), "PA5", "PIN56", "PIN14"},
    {Gpio<GPIOA_BASE, 6>::getPin(), "PA6", "PIN54", "PIN16"},
    {Gpio<GPIOA_BASE, 7>::getPin(), "PA7", "PIN52", "PIN18"},
    {Gpio<GPIOA_BASE, 8>::getPin(), "PA8", "PIN33", "PIN37"},
    // {Gpio<GPIOA_BASE, 9>::getPin(), "PA9", "PIN36", "PIN"}, Miosix serial
    // {Gpio<GPIOA_BASE, 10>::getPin(), "PA10", "PIN34", "PIN"}, Miosix serial
    {Gpio<GPIOA_BASE, 11>::getPin(), "PA11", "PIN27", "PIN43"},
    {Gpio<GPIOA_BASE, 12>::getPin(), "PA12", "PIN30", "PIN40"},
    // {Gpio<GPIOA_BASE, 13>::getPin(), "PA13", NULL, NULL}, SWDIO
    // {Gpio<GPIOA_BASE, 14>::getPin(), "PA14", NULL, NULL}, SWCLK
    {Gpio<GPIOA_BASE, 15>::getPin(), "PA15", "PIN32", "PIN38"},
    {Gpio<GPIOB_BASE, 0>::getPin(), "PB0", "PIN53", "PIN17"},
    {Gpio<GPIOB_BASE, 1>::getPin(), "PB1", "PIN51", "PIN19"},
    {Gpio<GPIOB_BASE, 2>::getPin(), "PB2", "PIN65", "PIN5"},
    {Gpio<GPIOB_BASE, 3>::getPin(), "PB3", "PIN15", "PIN55"},
    {Gpio<GPIOB_BASE, 4>::getPin(), "PB4", "PIN17", "PIN53"},
    // {Gpio<GPIOB_BASE, 5>::getPin(), "PB5", NULL, NULL}, RAM
    // {Gpio<GPIOB_BASE, 6>::getPin(), "PB6", NULL, NULL}, RAM
    {Gpio<GPIOB_BASE, 7>::getPin(), "PB7", "PIN14", "PIN56"},
    {Gpio<GPIOB_BASE, 8>::getPin(), "PB8", "PIN13", "PIN57"},
    {Gpio<GPIOB_BASE, 9>::getPin(), "PB9", "PIN11", "PIN59"},
    // {Gpio<GPIOB_BASE, 10>::getPin(), "PB10", NULL}, Flash
    {Gpio<GPIOB_BASE, 11>::getPin(), "PB11", "PIN49", "PIN21"},
    {Gpio<GPIOB_BASE, 12>::getPin(), "PB12", "PIN47", "PIN23"},
    {Gpio<GPIOB_BASE, 13>::getPin(), "PB13", "PIN45", "PIN25"},
    {Gpio<GPIOB_BASE, 14>::getPin(), "PB14", "PIN43", "PIN27"},
    {Gpio<GPIOB_BASE, 15>::getPin(), "PB15", "PIN46", "PIN24"},
    // {Gpio<GPIOC_BASE, 0>::getPin(), "PC0", NULL, NULL}, RAM
    {Gpio<GPIOC_BASE, 1>::getPin(), "PC1", "PIN63", "PIN7"},
    // {Gpio<GPIOC_BASE, 2>::getPin(), "PC2", NULL, NULL}, LED
    {Gpio<GPIOC_BASE, 3>::getPin(), "PC3", "PIN61", "PIN9"},
    {Gpio<GPIOC_BASE, 4>::getPin(), "PC4", "PIN50", "PIN20"},
    {Gpio<GPIOC_BASE, 5>::getPin(), "PC5", "PIN48", "PIN22"},
    {Gpio<GPIOC_BASE, 6>::getPin(), "PC6", "PIN37", "PIN33"},
    {Gpio<GPIOC_BASE, 7>::getPin(), "PC7", "PIN35", "PIN35"},
    // {Gpio<GPIOC_BASE, 8>::getPin(), "PC8", NULL, NULL}, SD Card
    // {Gpio<GPIOC_BASE, 9>::getPin(), "PC9", NULL, NULL}, SD Card
    // {Gpio<GPIOC_BASE, 10>::getPin(), "PC10", NULL, NULL}, SD Card
    // {Gpio<GPIOC_BASE, 11>::getPin(), "PC11", NULL, NULL}, SD Card
    // {Gpio<GPIOC_BASE, 12>::getPin(), "PC12", NULL, NULL}, SD Card
    // {Gpio<GPIOC_BASE, 13>::getPin(), "PC13", NULL, NULL}, LED
    // {Gpio<GPIOC_BASE, 14>::getPin(), "PC14", NULL, NULL}, LED
    // {Gpio<GPIOC_BASE, 15>::getPin(), "PC15", NULL, NULL}, LED
    // {Gpio<GPIOD_BASE, 0>::getPin(), "PD0", NULL, NULL}, RAM
    // {Gpio<GPIOD_BASE, 1>::getPin(), "PD1", NULL, NULL}, RAM
    // {Gpio<GPIOD_BASE, 2>::getPin(), "PD2", NULL, NULL}, SD Card
    {Gpio<GPIOD_BASE, 3>::getPin(), "PD3", "PIN28", "PIN42"},
    {Gpio<GPIOD_BASE, 4>::getPin(), "PD4", "PIN26", "PIN44"},
    {Gpio<GPIOD_BASE, 5>::getPin(), "PD5", "PIN22", "PIN48"},
    {Gpio<GPIOD_BASE, 6>::getPin(), "PD6", "PIN24", "PIN46"},
    {Gpio<GPIOD_BASE, 7>::getPin(), "PD7", "PIN23", "PIN47"},
    // {Gpio<GPIOD_BASE, 8>::getPin(), "PD8", NULL, NULL}, RAM
    // {Gpio<GPIOD_BASE, 9>::getPin(), "PD9", NULL, NULL}, RAM
    // {Gpio<GPIOD_BASE, 10>::getPin(), "PD10", NULL, NULL}, RAM
    {Gpio<GPIOD_BASE, 11>::getPin(), "PD11", "PIN44", "PIN26"},
    {Gpio<GPIOD_BASE, 12>::getPin(), "PD12", "PIN41", "PIN29"},
    {Gpio<GPIOD_BASE, 13>::getPin(), "PD13", "PIN39", "PIN31"},
    // {Gpio<GPIOD_BASE, 14>::getPin(), "PD14", NULL, NULL}, RAM
    // {Gpio<GPIOD_BASE, 15>::getPin(), "PD15", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 0>::getPin(), "PE0", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 1>::getPin(), "PE1", NULL, NULL}, RAM
    {Gpio<GPIOE_BASE, 2>::getPin(), "PE2", "PIN7", "PIN63"},
    {Gpio<GPIOE_BASE, 3>::getPin(), "PE3", "PIN12", "PIN58"},
    {Gpio<GPIOE_BASE, 4>::getPin(), "PE4", "PIN9", "PIN61"},
    {Gpio<GPIOE_BASE, 5>::getPin(), "PE5", "PIN5", "PIN65"},
    {Gpio<GPIOE_BASE, 6>::getPin(), "PE6", "PIN3", "PIN67"},
    // {Gpio<GPIOE_BASE, 7>::getPin(), "PE7", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 8>::getPin(), "PE8", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 9>::getPin(), "PE9", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 10>::getPin(), "PE10", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 11>::getPin(), "PE11", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 12>::getPin(), "PE12", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 13>::getPin(), "PE13", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 14>::getPin(), "PE14", NULL, NULL}, RAM
    // {Gpio<GPIOE_BASE, 15>::getPin(), "PE15", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 0>::getPin(), "PF0", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 1>::getPin(), "PF1", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 2>::getPin(), "PF2", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 3>::getPin(), "PF3", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 4>::getPin(), "PF4", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 5>::getPin(), "PF5", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 6>::getPin(), "PF6", NULL, NULL}, Flash
    // {Gpio<GPIOF_BASE, 7>::getPin(), "PF7", NULL, NULL}, Flash
    // {Gpio<GPIOF_BASE, 8>::getPin(), "PF8", NULL, NULL}, Flash
    // {Gpio<GPIOF_BASE, 9>::getPin(), "PF9", NULL, NULL}, Flash
    // {Gpio<GPIOF_BASE, 10>::getPin(), "PF10", NULL, NULL}, Flash
    // {Gpio<GPIOF_BASE, 11>::getPin(), "PF11", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 12>::getPin(), "PF12", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 13>::getPin(), "PF13", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 14>::getPin(), "PF14", NULL, NULL}, RAM
    // {Gpio<GPIOF_BASE, 15>::getPin(), "PF15", NULL, NULL}, RAM
    // {Gpio<GPIOG_BASE, 0>::getPin(), "PG0", NULL, NULL}, RAM
    // {Gpio<GPIOG_BASE, 1>::getPin(), "PG1", NULL, NULL}, RAM
    // {Gpio<GPIOG_BASE, 2>::getPin(), "PG2", NULL, NULL}, RAM
    {Gpio<GPIOG_BASE, 3>::getPin(), "PG3", "PIN42", "PIN28"},
    // {Gpio<GPIOG_BASE, 4>::getPin(), "PG4", NULL}, RAM
    // {Gpio<GPIOG_BASE, 5>::getPin(), "PG5", NULL}, RAM
    {Gpio<GPIOG_BASE, 6>::getPin(), "PG6", "PIN40", "PIN30"},
    {Gpio<GPIOG_BASE, 7>::getPin(), "PG7", "PIN38", "PIN32"},
    // {Gpio<GPIOG_BASE, 8>::getPin(), "PG8", NULL, NULL}, RAM
    {Gpio<GPIOG_BASE, 9>::getPin(), "PG9", "PIN19", "PIN51"},
    {Gpio<GPIOG_BASE, 10>::getPin(), "PG10", "PIN25", "PIN45"},
    {Gpio<GPIOG_BASE, 11>::getPin(), "PG11", "PIN21", "PIN49"},
    {Gpio<GPIOG_BASE, 12>::getPin(), "PG12", "PIN20", "PIN50"},
    {Gpio<GPIOG_BASE, 13>::getPin(), "PG13", "PIN18", "PIN52"},
    {Gpio<GPIOG_BASE, 14>::getPin(), "PG14", "PIN16", "PIN54"},
    // {Gpio<GPIOG_BASE, 15>::getPin(), "PG15", NULL, NULL} RAM
}};

void banner(bool furry)
{
    if (furry)
    {
        puts(R"(
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
        )");
    }
    else
    {
        puts(R"(
                                                 /
                                            /  /  /
                                        /    /      /
                                     /    /   /  /
                                      /  _=^|  /    /
                                 /  /  <^   |   /
                                     //     | /   /  /
                                 /   /      /
                                   >/      /    /   /
                               /  //      /  /         /
                                 </      /^>_   /  /
                              /  /      /    ^>_     /   /
                                /      /^<_     ^>_    /
                               /      / /  ^<_     ^>_     /
                               ^<_   /        ^<_     ^>_            ___----____
                                  ^</            ^<_     ^>_   _-<^^^
                                                    ^<_     ^<^
                                                       ^<_  /   /
                                                          ^/   /^>^>_  ********
                                                          '<__/  /^> ^>_  ___***
                                                              ^-/  /^>__^>_  ^^^
                                                                ^-/  /  ! /
                                                                  ^-/^_/_/
               _____________._____________________
              /,   O ..    ==   *          O ___  /
             /,         ,@@    @@@@@@@@@@   /()/ /
            === %%%%    """   @@@@@@@@@@   /"'/ /
           ==='%%%%      +   @@@@@@@@@@   /"'/ /
          /%     .   .%  .  @@@@@@@@@@   /"'/ /
         /%     . %..%  . ,  ,  ,  ,    /__/ /
        /____O_______________________O______/
                                      L__V
        )");
    }

    puts(R"(
 Welcome to the
   ________  __   _    _____      ______          __             _ __     
  / ____/ / / /  | |  / /__ \    /_  __/__  _____/ /________  __(_) /____ 
 / /   / / / /   | | / /__/ /     / / / _ \/ ___/ __/ ___/ / / / / __/ _ \
/ /___/ /_/ /    | |/ // __/     / / /  __(__  ) /_(__  ) /_/ / / /_/  __/
\____/\____/     |___//____/    /_/  \___/____/\__/____/\__,_/_/\__/\___/ 
    )");
}

void led_task()
{
    ledOff();

    const unsigned int DELAY = 200;
    while (1)
    {
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

void sd_test()
{
    // Write 1MB of bytes
    const size_t COUNT           = 256 * 1024;
    const uint32_t INITIAL_VALUE = 0xdeadbeef;

    puts("*** Starting SD test...");
    bool ok = true;

    FILE *f = fopen("/sd/test.bin", "wb");
    if (f == NULL)
    {
        puts("Failed to open test.bin");
        ok = false;
    }

    if (ok)
    {
        uint32_t value = INITIAL_VALUE;
        for (size_t i = 0; i < COUNT; i++)
        {
            size_t result = fwrite(&value, sizeof(value), 1, f);
            if (result != 1)
            {
                printf("Failed to write word %d\n", i);

                // We had a problem
                ok = false;
                break;
            }

            // Update value with xorshift
            value = (value << 8) ^ (value >> 8);
        }

        fclose(f);
    }

    if (ok)
    {
        // Reopen file
        f = fopen("/sd/test.bin", "rb");
        if (f == NULL)
        {
            puts("Failed to reopen test.bin");
            ok = false;
        }
    }

    if (ok)
    {
        uint32_t value = INITIAL_VALUE;
        for (size_t i = 0; i < COUNT; i++)
        {
            uint32_t actual = 0;
            size_t result   = fread(&actual, sizeof(actual), 1, f);
            if (result != 1)
            {
                printf("Failed to write word %d\n", i);
                ok = false;
                break;
            }

            if (actual != value)
            {
                printf(
                    "Failed to validate word %d, expected: %lu, actual: %lu\n",
                    i, value, actual);
                ok = false;
                break;
            }

            // Update value with xorshift
            value = (value << 8) ^ (value >> 8);
        }

        fclose(f);
    }

    if (ok)
    {
        puts("*** SD test succesfull!");
    }
    else
    {
        puts("*** SD test failed!");
    }
}

void xram_test()
{
    volatile uint16_t *const START = (volatile uint16_t *)0xd0000000;
    volatile uint16_t *const END   = START + (16 * 1024 * 1024);

    // First clear the whole RAM
    for (volatile uint16_t *iter = START; iter != END; iter++)
    {
        *iter = 0;
    }

    __DMB();  // Flush cache

    puts("*** Starting basic XRAM test...");
    bool ok = true;

    for (volatile uint16_t *iter = START; iter != END; iter++)
    {
        // Set a marker value at this address
        *iter = 0xffff;
        __DMB();  // Flush cache

        if (*iter != 0xffff)
        {
            ok = false;
            printf("Readback failed: %p\n", iter);
        }

        // Reset the value back to 0
        *iter = 0;
        __DMB();  // Flush cache
    }

    if (ok)
    {
        puts("*** XRAM basic test succesfull!");
    }
    else
    {
        puts("*** XRAM basic test failed!");
    }

    puts("*** Starting XRAM mirroring test...");
    ok = true;

    // Generate addresses to test every line
    for (int i = 1; i < 25; i++)
    {
        volatile uint16_t *other =
            (volatile uint16_t *)((size_t)START | (1 << i));

        // Write something
        *other = 0xdead;
        __DMB();  // Flush cache

        for (volatile uint16_t *iter = START; iter != END; iter++)
        {
            // Skip the written address
            if (iter == other)
            {
                continue;
            }

            // Check for mirroring
            if (*iter == 0xdead)
            {
                printf("Mirroring %p -> %p\n", other, iter);
                ok = false;
            }
        }

        // Reset the value
        *other = 0;
        __DMB();  // Flush cache
    }

    if (ok)
    {
        puts("*** XRAM mirror test succesfull!");
    }
    else
    {
        puts("*** XRAM mirror test failed!");
    }
}

void pin_continuity_test()
{
    for (auto &a : PIN_DEFS)
    {
        a.pin.mode(Mode::INPUT_PULL_DOWN);
    }

    puts("*** Starting pin continuity test...");
    bool ok = true;

    // Ok now start testing
    for (auto &a : PIN_DEFS)
    {
        a.pin.mode(Mode::OUTPUT);
        a.pin.high();
        for (auto &b : PIN_DEFS)
        {
            if (a.pin.getNumber() == b.pin.getNumber() &&
                a.pin.getPort() == b.pin.getPort())
            {
                continue;
            }

            if (b.pin.value() == 1)
            {
                printf("Shorted pins %s %s\n", a.gpio_name, b.gpio_name);
                ok = false;
            }
        }

        a.pin.low();
        a.pin.mode(Mode::INPUT_PULL_DOWN);
    }

    if (ok)
    {
        puts("*** Pin continuity test succesfull!");
    }
    else
    {
        puts("*** Pin continuity test failed!");
    }
}

void pin_semi_test()
{
    for (auto &a : PIN_DEFS)
    {
        a.pin.mode(Mode::OUTPUT);
        a.pin.low();
    }

    puts("*** Starting semi-automatic pin test...");

    std::atomic<bool> running(true);
    std::atomic<int> cur_idx(0);

    // Parallel test pulsing pin
    std::thread t(
        [&]()
        {
            while (running)
            {
                int idx = cur_idx;

                PIN_DEFS[idx].pin.high();
                Thread::sleep(10);
                PIN_DEFS[idx].pin.low();
                Thread::sleep(10);
            }
        });

    // Clean any pending newlines
    while (getchar() != '\n')
        ;
    puts("Press enter to continue...");

    for (size_t i = 0; i < PIN_DEFS.size(); i++)
    {
        auto &a = PIN_DEFS[i];

        // Pause waiting for keys
        while (getchar() != '\n')
            ;

        cur_idx = i;
        printf(
            "Pulsing %s, conan: %s, breakout v1: %s, press enter to "
            "continue...\n",
            a.gpio_name, a.conan_name, a.breakout_v1_name);
    }

    running = false;
    t.join();

    puts("*** Test finished!");
}

int main()
{
    banner(true);

    // Spawn the led blinker task
    std::thread t(led_task);

    while (1)
    {
        printf(
            "\nSelect test to run:\n"
            "- 1 Automatic SD test\n"
            "- 2 Automatic RAM test\n"
            "- 3 Automatic pin continuity test\n"
            "- 4 Semi-automatic pin test\n"
            "\n"
            "Command: ");

        int cmd;
        scanf("%d", &cmd);

        switch (cmd)
        {
            case 1:
                sd_test();
                break;

            case 2:
                xram_test();
                break;

            case 3:
                pin_continuity_test();
                break;

            case 4:
                pin_semi_test();
                break;
        }
    }

    return 0;
}