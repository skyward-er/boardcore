#define CATCH_CONFIG_RUNNER
#define CATCH_CONFIG_NO_POSIX_SIGNALS
#define CATCH_CONFIG_NO_CPP11_GENERATED_METHODS
#define CATCH_CONFIG_NO_CPP11_TYPE_TRAITS

#include <miosix.h>
#include <catch.hpp>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "boards/Homeone/Events.h"
#include "boards/Homeone/FlightModeManager/FlightModeManager.h"
#include "boards/Homeone/Topics.h"
#include "drivers/HardwareTimer.h"
#include "events/EventBroker.h"

using namespace HomeoneBoard;

using miosix::Thread;
using std::string;
using std::vector;

using Timer = HardwareTimer<uint32_t, 2>;

template <typename TimerType>
class ExecutionProfiler
{
public:
    ExecutionProfiler(TimerType& timer) : hw_timer(timer) {}

    void start(string msg)
    {
        current_msg = msg;
        printf("%s START\n", msg.c_str());
#ifdef DEBUG
        printf("DEBUG is defined. Execution times may not be correct.\n",
               msg.c_str());
#endif
        hw_timer.stop();
        hw_timer.start();
    }

    void tick()
    {
        auto tick = hw_timer.tick();
        printf("%s\nTime: %.3f (%.3f us)\n", current_msg.c_str(),
               hw_timer.toMilliSeconds(tick), hw_timer.toMicroSeconds(tick));
    }

    void stop()
    {
        auto tick = hw_timer.stop();
        printf("%s\nTotal execution time: %.3f ms (%.3f us)\n",
               current_msg.c_str(), hw_timer.toMilliSeconds(tick),
               hw_timer.toMicroSeconds(tick));
    }

private:
    string current_msg = "";
    TimerType& hw_timer;
};

int main()
{
    int result = Catch::Session().run();

    for (;;)
    {
        printf("end.\n");
        Thread::sleep(10000);
    }
}

void checkTransition(FlightModeManager& mgr, FMMState previous,
                     FMMState expected)
{
    for (int i = 0; i < 500; i++)
    {
        if (mgr.getStatus().state != previous)
            REQUIRE(mgr.getStatus().state == expected);

        usleep(100);
    }
    FAIL("Transition timeout\n");  // Fail the test. The state machine
                                   // didn't change the state
}

TEST_CASE("FlightModeManager", "[fmm]")
{
    Timer& tim = Timer::instance();
    tim.setPrescaler(83);

    ExecutionProfiler<Timer> ep{tim};

    FlightModeManager& mgr = *FlightModeManager::getInstance();
    mgr.start();
    checkTransition(mgr, FMMState::UNDEFINED, FMMState::INIT);

    ep.start("Profiling init error.");
    sEventBroker->post({EV_INIT_ERROR}, TOPIC_FLIGHT_EVENTS);
    // ep.tick();
    checkTransition(mgr, FMMState::INIT, FMMState::ERROR);
    ep.stop();
}