// This is the header file for the AvState class, which is a part of the flightControl module.
// The AvState class is a finite state machine (FSM) that represents the state of the rocket.
// The class has a constructor and a destructor, as well as a number of functions that transition from one state to others possible states.

#ifndef AVSTATE_H
#define AVSTATE_H

#include <iostream>
#include <list>
#include <string>
#include "Application/Data/data.hpp"
#include "Application/Data/fsm.hpp"


using namespace flight_computer;
// Path: AV-Firehorn-Rpi/include/flightControl/AvState.h
// Compare this snippet from AV-Firehorn-Rpi/src/flightControl/FSM.cpp:
//     // this function allows to get the current state of the FSM
// functions that transition from one state to others possible states
class AvState
{
public:
    // constructor
    explicit AvState();
    // destructor
    ~AvState();
    /**
     * @brief Return the FSM's current state.
     * @return The active @c State enum value.
     */
    State getCurrentState();

    /**
     * @brief Evaluate the current sensor/event snapshot and advance the FSM.
     *
     * Calls the appropriate @c fromXxx() transition function for the active
     * state, then stores the resulting state as the new current state.
     *
     * @param dump Snapshot of all avionics data at the current tick.
     */
    void update(const DataDump &dump);

    /**
     * @brief Convert a @c State enum value to a human-readable string.
     * @param state The state to convert.
     * @return String representation of @p state (e.g. @c "BURN", @c "ASCENT").
     */
    std::string stateToString(State state);

private:
    State fromInit(DataDump const &dump);
    State fromCalibration(DataDump const &dump);
    State fromFilling(DataDump const &dump);
    State fromArmed(DataDump const &dump);
    State fromPressurization(DataDump const &dump);
    State fromIgnition(DataDump const &dump);
    State fromBurn(DataDump const &dump);
    State fromAscent(DataDump const &dump);
    State fromDescent(DataDump const &dump);
    State fromLanded(DataDump const &dump);

    State fromAbortOnGround(DataDump const &dump);
    State fromAbortInFlight(DataDump const &dump);


    State currentState;

    // HAL_GetTick() timestamps for the timers published into
    // GOATStore::flightEventTimersStore every tick (see update()) --
    // flight_duration/ascent_duration/descent_duration, consumed by
    // fromAscent()/fromDescent()'s duration-based exit conditions.
    uint32_t liftoff_entry_ms_ = 0;
    uint32_t ascent_entry_ms_  = 0;
    uint32_t descent_entry_ms_ = 0;
    bool     has_lifted_off_   = false;
};

#endif
