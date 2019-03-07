#include <miosix.h>
#include <map>

#include "events/FSM.h"
#include "events/HSM.h"

using miosix::FastMutex;
using miosix::Lock;
using std::map;

/*
 * How long should we wait for the state machine to handle the event?
 * Value in milliseconds
 */
static const int SM_EVENT_HANDLE_UNCERTAINTY = 1;

/**
 * @brief Test if a specific transition occurs in a Finite State Machine
 * in response to an event.
 * Requires protected-level access to the FSM object, usually achieved by
 * "#define protected public"
 *
 * @tparam FSM_type Finite State Machine class to be tested
 * @param fsm FSM object reference
 * @param ev The event that should trigger the transition
 * @param expected_state Expected state after transition (state function
 * pointer)
 * @return true If the state machine is in the expected state after posting the
 * event
 * @return false If the machine is not in the expected state after posting the
 * event
 */
template <class FSM_type>
bool testFSMTransition(FSM_type& fsm, const Event& ev,
                       void (FSM_type::*expected_state)(const Event&))
{
    fsm.handleEvent(ev);
    return fsm.testState(expected_state);
}

/**
 * @brief Test if a specific transition occurs in a Finite State Machine
 * in response to an event, posted on a specific topic.
 * Once the event is posted, the state machine will process it asynchronously,
 * so we have to wait a little bit for it to happen. The wait time is defined in
 * SM_EVENT_HANDLE_UNCERTAINTY. If the state machine takes longer than this
 * time to process the event, the test will likely fail, as a transition will
 * not have occured.
 *
 * @tparam FSM_type Finite State Machine class to be tested
 * @param fsm FSM object reference
 * @param ev The event that should trigger the transition
 * @param topic Topic to post the event on
 * @param expected_state Expected state after transition (state function
 * pointer)
 * @param broker Eventbroker instance (Defaults to the singleton instance)
 * @return true If the state machine is in the expected state after posting the
 * event
 * @return false If the machine is not in the expected state after posting the
 * event
 */
template <class FSM_type>
bool testFSMAsyncTransition(FSM_type& fsm, const Event& ev, uint8_t topic,
                            void (FSM_type::*expected_state)(const Event&),
                            EventBroker& broker = *sEventBroker)
{
    broker.post(ev, topic);
    // Wait for the event to be handled
    miosix::Thread::sleep(SM_EVENT_HANDLE_UNCERTAINTY);
    return fsm.testState(expected_state);
}

/**
 * @brief Test if a specific transition occurs in a Hierarchical State Machine
 * in response to an event.
 * Requires protected-level access to the HSM object, usually achieved by
 * "#define protected public"
 *
 * @tparam HSM_type Hierarchical State Machine class to be tested
 * @param fsm FSM object reference
 * @param ev The event that should trigger the transition
 * @param expected_state Expected state after transition (state function
 * pointer)
 * @return true If the state machine is in the expected state after posting the
 * event
 * @return false If the machine is not in the expected state after posting the
 * event
 */
template <class HSM_type>
bool testHSMTransition(HSM_type& hsm, const Event& ev,
                       State (HSM_type::*expected_state)(const Event&))
{
    hsm.handleEvent(ev);
    return hsm.testState(expected_state);
}

/**
 * @brief Test if a specific transition occurs in a Hierarchical State Machine
 * in response to an event, posted on a specific topic.
 * Once the event is posted, the state machine will process it asynchronously,
 * so we have to wait a little bit for it to happen. The wait time is defined in
 * SM_EVENT_HANDLE_UNCERTAINTY. If the state machine takes longer than this
 * time to process the event, the test will likely fail, as a transition will
 * not have occured.
 *
 * @tparam HSM_type Hierarchical State Machine class to be tested
 * @param fsm FSM object reference
 * @param ev The event that should trigger the transition
 * @param topic Topic to post the event on
 * @param expected_state Expected state after transition (state function
 * pointer)
 * @param broker Eventbroker instance (Defaults to the singleton instance)
 * @return true If the state machine is in the expected state after posting the
 * event
 * @return false If the machine is not in the expected state after posting the
 * event
 */
template <class HSM_type>
bool testHSMAsyncTransition(HSM_type& hsm, const Event& ev, uint8_t topic,
                            State (HSM_type::*expected_state)(const Event&),
                            EventBroker& broker = *sEventBroker)
{
    broker.post(ev, topic);
    // Wait for the event to be handled
    miosix::Thread::sleep(SM_EVENT_HANDLE_UNCERTAINTY);
    return hsm.testState(expected_state);
}

/**
 * @brief Helper class to count how many events are sent to the topic(s) it is
 * registered to.
 *
 * Useful if you want to check wether or not events are being
 * effectively posted
 */
class EventCounter : public EventHandler
{
public:
    /**
     * @brief Construct a new Event Counter object
     * 
     * @param broker EventBroker to listen events to
     */
    EventCounter(EventBroker& broker) : broker(broker)
    {

    }

    ~EventCounter()
    {
        broker.unsubscribe(this);
    }

    /**
     * @brief Subscribes to a topic in the EventBroker
     * 
     * @param topic 
     */
    void subscribe(uint8_t topic)
    {
        broker.subscribe(this, topic);
    }

    // Override postEvent not to put events in a queue, just count the events it
    // receives.
    void postEvent(const Event& ev) override
    {
        Lock<FastMutex> l(mutex);

        ++map_counter[ev.sig];
        ++total_count;

        last_event = ev.sig;
    }

    /**
     * @brief Returns the number of times a specific event has been received
     */
    unsigned int getCount(const Event& ev) { return getCount(ev.sig); }

    /**
     * @brief Returns the number of times a specific event has been received
     */
    unsigned int getCount(uint8_t ev_sig)
    {
        Lock<FastMutex> l(mutex);

        if (map_counter.count(ev_sig) == 1)
        {
            return map_counter.at(ev_sig);
        }

        return 0;
    }

    /**
     * @brief Returns how many events have been received in total
     */
    unsigned int getTotalCount() { return total_count; }

    /**
     * @brief Returns the signature of the last event received (ev.sig)
     */
    uint8_t getLastEvent() { return last_event; }

protected:
    // Do nothing
    void handleEvent(const Event& ev) override
    {
        // Avoid unused argument warning
        (void)ev;
    };

private:
    EventBroker& broker;

    FastMutex mutex;
    // Count how many times we have received each event
    map<uint8_t, unsigned int> map_counter;

    // How many events we have received in total
    unsigned int total_count = 0;
    uint8_t last_event;
};