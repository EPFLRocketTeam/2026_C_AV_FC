
#ifndef APP_FSM_H
#define APP_FSM_H

namespace flight_computer {
    enum State {
        /* =============== On ground =============== */
        INIT,
        CALIBRATION,
        MANUAL,
        ARMED,
        READY,

        /* =============== In flight =============== */
        THRUSTSEQUENCE,
        LIFTOFF,
        ASCENT,
        LANDED,
        DESCENT,

        /* ============== Error States ============== */
        ERRORGROUND,
        ERRORFLIGHT
    };
};

#endif /* APP_FSM_H */
