/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2017  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "UdpManager.h"

using namespace std;
using namespace miosix;

void _evt_mgmt_thread(void *args) {
    
//     udpComm->evtQueue.run();
}

UdpManager::UdpManager() :count(0) {
    
//      Thread::create(_evt_mgmt_thread,1024);    
}

UdpManager::~UdpManager() {
}

void UdpManager::start()
{    
    puts("starting");
    function<void ()> fun = bind(&UdpManager::gumby,this);    
    evtQueue.post(fun);
    evtQueue.run();
}


void UdpManager::gumby()
{
    Thread::sleep(500);
    printf("I'm the gumby number %d\n",count++);
    function<void ()> fun = bind(&UdpManager::gumby,this);    
    evtQueue.post(fun);
}
