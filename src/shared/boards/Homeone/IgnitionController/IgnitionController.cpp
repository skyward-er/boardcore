/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Matteo Michele Piazzolla
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

#include <boards/Homeone/IgnitionController/IgnitionController.h>
#include <events/EventBroker.h>

#include <boards/Homeone/Events.h>
#include <boards/Homeone/Topics.h>

#include <boards/Homeone/CanAbstraction.h>

#include <cstdlib>

#include <iostream>

using namespace std;

uint8_t can_get_status_msg[] = {'G','E','T','S','T','A','T','X'};
uint8_t can_abort_ignition_msg[] = {'A','B','O','R','T','N','O','W'};

uint8_t CAN_IGNITION = 0xAF;
uint8_t CAN_IGNITION_STATUS = 0xFA;


namespace HomeoneBoard
{
namespace IGN
{

IgnitionController::IgnitionController() : FSM(&IgnitionController::state_idle)
{
    sEventBroker->subscribe(this, TOPIC_COMMANDS);
    sEventBroker->subscribe(this, TOPIC_IGNITION);
    
    CanAbstraction canAbst(LINK_IGNITION);
    ignitionPublisher = canAbst.getPublisher(CAN_IGNITION);
    ignitionStatusSub = canAbst.getSubscriber(CAN_IGNITION);
    //ignitionStatusSub = canAbst.getSubscriber(CAN_IGNITION_STATUS);
    
    cout << "init ignition controller" << endl;
}

void IgnitionController::state_idle(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            cout << "idle entry" << endl;
            sEventBroker->postDelayed({EV_IGN_GET_STATUS}, TOPIC_IGNITION, 1000);
            break;

        case EV_EXIT:
            break;

        case EV_IGN_ABORT:
            cout << "Abort Message" << endl;
            ignitionPublisher->publish(can_abort_ignition_msg, sizeof(can_abort_ignition_msg));
            break;

        case EV_IGN_GET_STATUS:
            transition(&IgnitionController::state_get_status);
            break;

        case EV_IGN_LAUNCH:{
            cout << "Launch code Message" << endl;
            StartLaunchEvent launch_evt = static_cast<const StartLaunchEvent&>(e);
            uint64_t launchCode = launch_evt.launchCode;

            //TODO check endinaess
            ignitionPublisher->publish((uint8_t*) &launchCode, sizeof(can_abort_ignition_msg));
            break;
        }
        default:
            printf("Unknown event received.\n");
            break;
    }
}

void IgnitionController::state_get_status(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            cout << " requesting status..." << endl;
            ignitionPublisher->publish(can_get_status_msg, sizeof(can_get_status_msg));
            transition(&IgnitionController::state_wait_response);
            break;
            
        case EV_EXIT:
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void IgnitionController::state_wait_response(const Event& e)
{
    static uint8_t retry_counter = 0;

    switch (e.sig)
    {
        case EV_ENTRY:
            cout << "entry wait "<< endl;
            sEventBroker->post(Event{EV_IGN_WAIT}, TOPIC_IGNITION);

            break;
            
        case EV_EXIT:
            break;

        case EV_IGN_WAIT:{
                cout << "trying to receive" << endl;
                if(retry_counter > MAX_RETRY){
                    cout << "too many attempt" << endl;
                    retry_counter = 0;
                    transition(&IgnitionController::state_idle);
                }
                else{
                    retry_counter++;
                    if(ignitionStatusSub->haveMessage() == true){
                        cout << "msg received" << endl;
                        uint8_t can_buffer[8];
                        bool res = ignitionStatusSub->receive(&can_buffer, 8);
                        if(res == true){
                            updateInternalState(can_buffer);
                            transition(&IgnitionController::state_idle);
                        }
                    }
                    else{
                        sEventBroker->post(Event{EV_IGN_WAIT}, TOPIC_IGNITION);
                    }
                }
            }
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void IgnitionController::updateInternalState(uint8_t *can_msg){
    cout << "updating status" << endl;
    cout << "u1_abort_cmd " << ignition_board_status.u1_abort_cmd << endl;
    cout << "u1_abort_timeout " << ignition_board_status.u1_abort_timeout << endl;
    cout << "u1_wrong_code " << ignition_board_status.u1_wrong_code << endl;
    cout << "u1_launch_done " << ignition_board_status.u1_launch_done << endl;
    cout << "u2_abort_cmd " << ignition_board_status.u2_abort_cmd << endl;
    cout << "u2_abort_timeout " << ignition_board_status.u2_abort_timeout << endl;
    cout << "u2_wrong_code " << ignition_board_status.u2_wrong_code << endl;
    cout << "u2_launch_done " << ignition_board_status.u2_launch_done << endl;
    memcpy((void*) &this->ignition_board_status, (void*)can_msg, sizeof(IgnitionBoardStatus));
}



}
}
