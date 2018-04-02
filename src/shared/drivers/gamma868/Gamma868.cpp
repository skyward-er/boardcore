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

#include "Gamma868.h"

using namespace miosix;
/*
 * A serial port attached to the Gamma868 RX and TX pins is expected
 * to be passed to the object in order to communicate with the device.
 *
 * NOTE: The serial port has to be already open and at the baudrate
 * at which the module has been configured (default is 9600 baud).
 *
 * NOTE: The object also uses 2 other pins, gammaSwitch and gammaLed, which are
 * defined in the gamma_config.h file.
 */
Gamma868::Gamma868(const char *serialPath)
{
    fd = open(serialPath, O_RDWR);
    if (fd < 0)
        printf("Cannot open %s\n", serialPath);
    gammaLed::mode(Mode::INPUT);
    gammaSwitch::mode(Mode::OUTPUT);
    gammaSwitch::high();
}

/*
 * Immediately sends command (blocking).
 */
bool Gamma868::send(const uint8_t* pkt, uint32_t pkt_len)
{
    // Send to gamma
    {
        Lock<FastMutex> l(gammaMutex);
        write(fd, pkt, pkt_len);
    }

    return true;
}

/*
 * Reads from the gamma868 serial (blocking).
 */
void Gamma868::receive(uint8_t* pkt, uint32_t pkt_len)
{
    {
        Lock<FastMutex> l(gammaMutex);
        read(fd, pkt, pkt_len);  // Read all the bufLen chars
    }

}

/*
 * Set a new configuration to gamma.
 * Returns true if the configuration was set right.
 */
bool Gamma868::config(Configuration newConf)
{
    if (enterLearnMode())
    {
        // Get current configuration
        Thread::sleep(100);
        printConfig();

        // Writes the new configuration
        printf("Writing new configuration..\n");
        writeConfig(newConf);  // TODO catch error

        // Get current configuration
        Thread::sleep(100);
        printConfig();

        // Exits from learn mode
        if(exitLearnMode() == true)
        {
            printf("Configuration ended ok.\n\n");
            return true;
        }
        else
        {
            printf("Configuration failed.\n\n");
            return false;
        }
    }
    else
    {
        printf("Check that the device is correctly connected and retry.\n");
        return false;
    }
}

/*
 * Puts the gamma868 in "learn mode" (configuration mode).
 */
bool Gamma868::enterLearnMode()
{
    // Enter learn mode
    printf("Entering learn mode ... ");
    fflush(stdout);
    gammaSwitch::low();

    // Create learn mode confirmation and timeout thread
    Thread *checkLearnModeThread, *timeoutThread;
    checkLearnModeThread = Thread::create(&Gamma868::static_confirmLearnMode,
                                  STACK_DEFAULT_FOR_PTHREAD, MAIN_PRIORITY,
                                  reinterpret_cast<void *>(this));
    timeoutThread        = Thread::create(&Gamma868::static_timer,
                                  STACK_DEFAULT_FOR_PTHREAD, MAIN_PRIORITY,
                                  reinterpret_cast<void *>(this));

    if (checkLearnModeThread == NULL || timeoutThread == NULL)
    {
        printf("Failed: learn mode control threads not created.\n");
        return false;
    }

    // Wait for confirm (or timeout)
    {
        Lock<FastMutex> l(learnMutex);
        while (learnMode == 0)
            learnCond.wait(l);
    }

    gammaSwitch::high();  // Stop "pushing" the button

    if (learnMode == -1)
    {
        printf("Failed!\n");
        return false;
    }
    else
    {
        printf("Ok\n");
        return true;
    }
}

/*
 * Checks how many times the gamma868 LRN LED flashes: 2 flashes confirm
 * that the device has entered learn mode.
 * Runs in a separate Thread.
 */
void Gamma868::confirmLearnMode()
{
    int times = 0;
    int learn = gammaLed::value();

    while (1)
    {
        Thread::sleep(100);
        if (learnMode != 0)
            break;  // Breaks if timer has set learnMode to -1

        int curState = gammaLed::value();
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
 * Puts the gamma868 out of "learn mode" (configuration mode).
 */
bool Gamma868::exitLearnMode()
{
    // Enter learn mode
    printf("Exiting learn mode ... ");
    
    write(fd, "#Q", 2);

    return true;
}

/*
 * Waits for 5 seconds : if learn mode wasn't confirmed  after this time,
 * signal an error.
 * Runs in a separate Thread.
 */
void Gamma868::timer()
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
 * Prints gamma868 configuration.
 */
void Gamma868::printConfig()
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
int Gamma868::writeConfig(struct Configuration conf)
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
 * Waits until an "OK" is received on the serial port (blocking).
 */
void Gamma868::waitForOk()
{
    char reply[3];
    read(fd, reply, 3);
    printf("%s\n", reply);
    Thread::sleep(100);
}