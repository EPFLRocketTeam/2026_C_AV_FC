
#ifndef APP_FSM_H
#define APP_FSM_H

namespace flight_computer {
    enum State {
        /* =============== On ground =============== */
        INIT,
        CALIBRATION,
		FILLING,

        ARMED,
        PRESSURIZATION,
		IGNITION,

        /* =============== In flight =============== */
        BURN,
        ASCENT,
		DESCENT,
        LANDED,


        /* ============== Error States ============== */
        ABORT_ON_GROUND,
        ABORT_IN_FLIGHT,

		/* ============== Other ============== */
		MANUAL,
    };
};

#endif /* APP_FSM_H */
