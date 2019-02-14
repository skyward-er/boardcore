#include "events/FSM.h"
#include "events/HSM.h"

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