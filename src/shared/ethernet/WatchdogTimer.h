/*
 * Simple watchdog timer with 1ms resolution that calls a function when expires
 * Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#ifndef WATCHDOGTIMER_H
#define WATCHDOGTIMER_H

#include <Common.h>
#include <Singleton.h>
#include <functional>

class WatchdogTimer : Singleton< WatchdogTimer > {
      friend class Singleton<WatchdogTimer>;
public:
    
    /**
     * Start the timer
     */
    void start();
    
    /**
     * Stop the timer
     */
    void stop();
    
    /**
     * Clear timer's internal counter
     */
    void clear();
    
    /**
     * Set timeout period: callback after @param duration milliseconds after
     * counter is started
     */
    void setDuration(uint16_t duration);
    
    /**
     * Set a function that will be called when timer expires.
     * WARNING: the callback will be executed inside an interrupt routine, so
     * it must be written properly!! It cannot malloc, printf and other things 
     * like that
     */
    void setCallback(std::function<void ()> callback){ 
        irqCallback = callback; }
    
    /**
     * @return true if timer expired. Calling start() and clear() reset the
     * flag to false
     */
    bool expired() { return triggered; }
    
private:
    WatchdogTimer();
    ~WatchdogTimer();
    
    float ms_to_tick;   //conversion factor, number of counter ticks in a ms
    bool triggered;
    std::function<void ()> irqCallback;
    
    friend void Irq_impl();
    
    WatchdogTimer(const WatchdogTimer& other);
    WatchdogTimer& operator=(const WatchdogTimer& other);
    bool operator==(const WatchdogTimer& other);
};

#endif // WATCHDOGTIMER_H
