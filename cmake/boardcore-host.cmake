# Copyright (c) 2021 Skyward Experimental Rocketry
# Authors: Damiano Amatruda
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

add_library(boardcore-host STATIC EXCLUDE_FROM_ALL
    # Debug
    ${SBS_BASE}/src/shared/utils/Debug.cpp
    ${SBS_BASE}/src/shared/diagnostic/CpuMeter.cpp
    ${SBS_BASE}/src/shared/diagnostic/PrintLogger.cpp

    # Actuators
    ${SBS_BASE}/src/shared/actuators/Servo/Servo.cpp

    # Drivers
    ${SBS_BASE}/src/shared/drivers/timer/TimestampTimer.cpp

    # Events
    ${SBS_BASE}/src/shared/events/EventBroker.cpp

    # Logger
    ${SBS_BASE}/src/shared/logger/Logger.cpp

    # Radio
    ${SBS_BASE}/src/shared/radio/Xbee/APIFrameParser.cpp

    # Scheduler
    ${SBS_BASE}/src/shared/scheduler/TaskScheduler.cpp

    # Sensors
    ${SBS_BASE}/src/shared/sensors/SensorManager.cpp
    ${SBS_BASE}/src/shared/sensors/SensorSampler.cpp

    # Utils
    ${SBS_BASE}/src/shared/utils/AeroUtils/AeroUtils.cpp
    ${SBS_BASE}/src/shared/utils/SkyQuaternion/SkyQuaternion.cpp
    ${SBS_BASE}/src/shared/utils/Stats/Stats.cpp
    ${SBS_BASE}/src/shared/utils/TestUtils/TestHelper.cpp
)
add_library(SkywardBoardcore::Boardcore::host ALIAS boardcore-host)
target_include_directories(boardcore-host PUBLIC ${SBS_BASE}/src/shared)
target_link_libraries(boardcore-host PUBLIC
    Miosix::Miosix::host
    TSCPP::TSCPP
    Eigen3::Eigen
    fmt::fmt-header-only
    Catch2::Catch2
    Mavlink::Mavlink
)
