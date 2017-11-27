/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Nuno Barcellos
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

#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdio>
#include "miosix.h"

using namespace std;
using namespace miosix;

// GAMMA868 configuration status
struct config
{
    int local_addr[3] = {127, 127, 127};
    int dest_addr[3]  = {127, 127, 127};
    int lora_mode     = 6;
    int lora_pow      = 15;
    int handshake     = 0;
    int baudrate      = 4;
};

// PIN CONFIG
typedef Gpio<GPIOG_BASE, 13> greenLed;
typedef Gpio<GPIOG_BASE, 14> redLed;
typedef Gpio<GPIOA_BASE, 0> button;
typedef Gpio<GPIOB_BASE, 2> learnSwitch;
typedef Gpio<GPIOB_BASE, 0> learnAck;

// Needed to check if learn mode is really active.
FastMutex learnMutex;
ConditionVariable learnCond;
int learnMode = 0;

// Serial port
int fd = -1;

// Functions
int initialize();
int enterLearnMode();
void confirmLearnMode(void *arg);
void timer(void *arg);
void printConfig();
int writeConfig(config conf);

void waitForButton();
void waitForOk();

/*
 * GAMMA868 Configuration Software:
 * This software is intended for stm32f429i discovery boards.
 *
 * Discovery board should be connected as follows
 * - Default USART(default @19200 baud)     Anything with a keyboard.
 * - Auxtty USART(must be @9600 baud)       Gamma868 SERIAL(pin 18: rx, 17:tx)
 * - learnSwitch (see pin config)           Gamma868 LRN SW(pin 5)
 * - learnAck (see pin config)              Gamma868 LRN LED(pin 6)
 *
 * Gamma module must be in SERIAL MODEM mode (see documentation).
 */
int main()
{

    printf("\n----- GAMMA868 CONFIGURATOR -----\n");
    int init = initialize();  // Initialize
    if (init < 0)
    {
        printf("Exiting program.");
        return -1;
    }

    while (1)
    {
        printf("Press button to start dsgamma configuration.\n");
        waitForButton();
        int lrn = enterLearnMode();  // Learn Mode

        if (lrn < 0)
        {
            printf("Check that the device is correctly connected and retry.\n");
        }
        else
        {
            // Get current configuration
            Thread::sleep(100);
            printConfig();

            // Write new configuration
            // TODO check values before saving them
            struct config newConf;

            printf("LOCAL ADDRESS (3 bytes, 0-127 each):\n");
            scanf("%d %d %d", &newConf.local_addr[0], &newConf.local_addr[1],
                  &newConf.local_addr[2]);
            printf("DESTINATION ADDRESS (3 bytes, 0-127 each):\n");
            scanf("%d %d %d", &newConf.dest_addr[0], &newConf.dest_addr[1],
                  &newConf.dest_addr[2]);
            printf("LORA MODE (1-6):\n");
            scanf("%d", &newConf.lora_mode);
            printf("LORA POWER (0-15):\n");
            scanf("%d", &newConf.lora_pow);
            printf("HANDSHAKE (0-1):\n");
            scanf("%d", &newConf.handshake);
            printf("BAUDRATE (0-4):\n");
            scanf("%d", &newConf.baudrate);

            writeConfig(newConf);  // TODO catch error

            // Get current configuration
            Thread::sleep(100);
            printConfig();

            // Wait for button to close learn mode
            printf("Press button to end configuration.\n");
            waitForButton();
            write(fd, "#Q", 2);
            printf("Configuration ended.\n\n");
        }

        // Clean up
        learnMode = 0;
        greenLed::low();
        Thread::sleep(500);
    }
}

/*
 * Configures discovery gpio and serial port to communicate with the gamma868
 * module.
 */
int initialize()
{
    // Discovery gpio setup
    {
        FastInterruptDisableLock dLock;
        greenLed::mode(Mode::OUTPUT);
        redLed::mode(Mode::OUTPUT);
        button::mode(Mode::INPUT);
        learnSwitch::mode(Mode::OUTPUT);
        learnAck::mode(Mode::INPUT);
    }
    learnSwitch::high();  // Learn switch is active low

    // Serial port setup
    printf("Opening serial port /dev/auxtty on discovery board ... ");
    fd = open("/dev/auxtty", O_RDWR);
    if (fd < 0)
    {
        printf("Failed!\n");
        return -1;
    }
    else
    {
        printf("Ok\n");
        return 0;
    }
}

/*
 * Puts the gamma868 in "learn mode" (configuration mode).
 */
int enterLearnMode()
{
    // Enter learn mode
    printf("Entering learn mode ... ");
    fflush(stdout);
    learnSwitch::low();

    // Create learn mode confirmation and timeout thread
    Thread *checkLearnModeThread, *timeoutThread;
    checkLearnModeThread = Thread::create(confirmLearnMode, STACK_MIN);
    timeoutThread        = Thread::create(timer, STACK_MIN);

    if (checkLearnModeThread == NULL || timeoutThread == NULL)
    {
        printf("Failed: learn mode control threads not created.\n");
        return -1;
    }

    // Wait for confirm (or timeout)
    {
        Lock<FastMutex> l(learnMutex);
        while (learnMode == 0)
            learnCond.wait(l);
    }

    learnSwitch::high();  // Stop "pushing" the button

    if (learnMode == -1)
    {
        printf("Failed!\n");
        return -2;
    }
    else
    {
        printf("Ok\n");
        greenLed::high();
        return 0;
    }
}

/*
 * Prints gamma868 configuration.
 */
void printConfig()
{
    // TODO timeout
    char config[15];
    write(fd, "#?", 2);
    read(fd, config, 15);
    printf("Current configuration: \n");
    for (int i = 2; i < 13; i++)
    {
        printf("%02X ", config[i]);  // Prints hex values
    }
    printf("\n");
}

/*
 * Sends configuration to the gamma868 module.
 */
int writeConfig(struct config conf)
{
    // TODO check values before writing

    char conf_addr[8] = "#A";
    for (int i = 0; i < 3; i++)
    {
        conf_addr[2 + i] = (char)conf.local_addr[i];
        conf_addr[5 + i] = (char)conf.dest_addr[i];
    }
    write(fd, conf_addr, 8);
    waitForOk();

    char conf_baud[3] = "#B";
    conf_baud[2]      = (char)conf.baudrate;
    write(fd, conf_baud, 3);
    waitForOk();

    char conf_handshake[3] = "#H";
    conf_handshake[2]      = (char)conf.handshake;
    write(fd, conf_handshake, 3);
    waitForOk();

    char conf_lora[4] = "#C";
    conf_lora[2]      = (char)conf.lora_mode;
    conf_lora[3]      = (char)conf.lora_pow;
    write(fd, conf_lora, 4);
    waitForOk();

    return 0;
}

/*
 * Checks how many times the gamma868 LRN LED flashes: 2 flashes confirm
 * that the device has entered learn mode.
 * Runs in a separate Thread.
 */
void confirmLearnMode(void *arg)
{
    int times = 0;
    int learn = learnAck::value();

    while (1)
    {
        Thread::sleep(100);
        if (learnMode != 0)
            break;  // Breaks if timer has set learnMode to -1

        int curState = learnAck::value();
        if (curState != learn)
        {  // Count how many times the led changes state
            learn = curState;
            times++;
        }

        if (times == 5)
        {  // Set learnMode flag to 1 (with mutex).
            {
                Lock<FastMutex> l(learnMutex);
                learnMode = 1;
                learnCond.signal();
            }
            break;
        }
    }
}

/*
 * Waits for 5 seconds : if learn mode wasn't confirmed  after this time,
 * signal an error.
 * Runs in a separate Thread.
 */
void timer(void *arg)
{
    for (int i = 0; i < 50; i++)
    {
        if (learnMode > 0)
            break;  // If the learnMode confirm arrives, stop the timer.

        if (i == 49)
        {
            {
                Lock<FastMutex> l(learnMutex);
                learnMode = -1;
                learnCond.signal();
            }
        }
        Thread::sleep(100);  // 100ms x 50 cycles = 5 sec
    }
}

/*
 * Waits for the discovery user button to be pressed (blocking).
 */
void waitForButton()
{
    while (1)
    {
        if (button::value() == 1)
            break;  // Wait for button
    }
}

/*
 * Waits until an "OK" is received on the serial port (blocking).
 */
void waitForOk()
{
    char reply[3];
    read(fd, reply, 3);
    printf("%s\n", reply);
    Thread::sleep(100);
}