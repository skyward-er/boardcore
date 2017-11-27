/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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

#include <drivers/gamma868/Gamma868.h>
#include <stdio.h>
#include <string.h>
#include "miosix.h"

using namespace std;
using namespace miosix;

#define SENDER 1
#define ECHO_RECEIVER 2
#define NORMAL 0

/* DISCOVERY F429I*/
typedef Gpio<GPIOG_BASE, 13> greenLed;
typedef Gpio<GPIOG_BASE, 14> redLed;
typedef Gpio<GPIOA_BASE, 0> button;

Gamma868 gamma("/dev/auxtty");  // create gamma object

// RTT calculation
long long sendTime = 0;
int nTentativi     = 0;
int tot            = 0;
int state          = NORMAL;
int charSent       = 0;

void btnClick(void *arg);
void stdInput();
void handleCommand(char *cmd);
void handleData(char *data);
void receiver(void *arg);

int main()
{
    gamma.start();  //!!!!!IMPORTANT!!!!!!!!

    // Discovery gpio setup
    {
        FastInterruptDisableLock dLock;
        greenLed::mode(Mode::OUTPUT);
        redLed::mode(Mode::OUTPUT);
        button::mode(Mode::INPUT);
    }

    // STACK_DEFAULT_FOR_PTHREAD is needed for printf())
    Thread::create(btnClick, STACK_DEFAULT_FOR_PTHREAD);
    Thread::create(receiver, STACK_DEFAULT_FOR_PTHREAD);
    stdInput();  // Runs in a loop

    return 0;
}

/*
 * Handles the button click (send an echo command).
 */
void btnClick(void *arg)
{

    char msg[CMD_LEN];

    while (1)
    {

        // Wait for button
        while (1)
        {
            if (button::value() == 1)
                break;
        }

        //---------------Send COMMAND----------------
        printf("Writing CMD... \n");
        msg[0] = (char)10;
        gamma.sendCmd(CMD_LEN, msg);
        //------------------------------------------
        printf("Ok \n");

        Thread::sleep(200);
    }
}

/*
 * Handles standard input (sends the string).
 */
void stdInput()
{

    char msg[100 + 1];

    while (1)
    {

        // Wait for button
        scanf("%s", msg);

        // Save current time
        if (state == SENDER && msg[0] == 'a')
        {
            for (int i = 0; i < 10; i++)
            {
                for (int j = 0; j < MIN_TO_SEND; j++)
                {
                    msg[j] = (char)j;
                }
                gamma.send(MIN_TO_SEND, msg);
                sendTime = miosix::getTick();
                nTentativi++;
                Thread::sleep(1000);
            }
        }

        //----------------SEND DATA----------------
        // gamma.send(strlen(msg), msg);
        //-----------------------------------------

        printf("Ok \n");
    }
}

/*
 * Continuously reads from the device.
 */
void receiver(void *arg)
{

    char inputBuf[DATA_LEN];

    while (1)
    {
        printf("Reading: \n");
        bool cmdReceived = gamma.receive(DATA_LEN, inputBuf);

        if (cmdReceived)
            handleCommand(inputBuf);
        else
            handleData(inputBuf);
    }
}

void handleCommand(char *cmd)
{
    printf("Command received!\n");
    char c = (char)20;
    switch ((int)cmd[0])
    {
        case 0:
            state = NORMAL;
            break;
        case 10:
            state = ECHO_RECEIVER;
            gamma.sendCmd(1, &c);
            break;
        case 20:
            printf("ACK\n");
            state = SENDER;
            break;
        default:
            printf("Unknown command\n");
    }
}

void handleData(char *data)
{

    long long arrivalTime = miosix::getTick();
    printf("Received: ");
    for (int i = 0; i < DATA_LEN; i++)
    {
        printf("%c", data[i]);
    }
    printf("\n");

    if (state == ECHO_RECEIVER)
    {
        gamma.send(DATA_LEN, data);
    }
    else if (state == 1)
    {
        int rtt = arrivalTime - sendTime;
        printf("RTT: %d\n", rtt);

        printf("--------------RESULT------------");
        tot += rtt;

        // Print Delay calculation
        if (nTentativi > 0)
            printf("Tentativi: %d  Media: %d \n", nTentativi,
                   tot / (2 * nTentativi));
    }
}