/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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
 * OUT OF OR IN CONNECTION\ WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "NoseconeStatus.h"


namespace NoseconeBoard 
{


/* Getter */
void getNoseconeStatus(CanInterfaces::NoseconeBoardStatus* status) 
{
    status->motor_active          = status_g.motor_active;        
    status->motor_last_direction  = status_g.motor_last_direction;
    status->homeone_not_connected = status_g.homeone_not_connected;

    status->close_received = status_g.close_received;
    status->close_timeout  = status_g.close_timeout;
    status->close_stop     = status_g.close_stop;
    status->close_limit    = status_g.close_limit;
    status->open_received  = status_g.open_received;
    status->open_timeout   = status_g.open_timeout;
    status->open_stop      = status_g.open_stop;
    status->open_limit     = status_g.open_limit;

    status->max_current_sensed  = status_g.max_current_sensed;
    status->min_current_sensed  = status_g.min_current_sensed;
    status->last_current_sensed = status_g.last_current_sensed;
}

} /* namespace NoseconeBoard */