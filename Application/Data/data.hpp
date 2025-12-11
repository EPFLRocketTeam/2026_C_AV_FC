
#ifndef APP_DATA_H
#define APP_DATA_H

#include "Application/Data/fsm.hpp"

namespace flight_computer {
    class Data {
    public:
        State state;

        Data ();
    };
    
    class DataManager {
    public:
        static Data read ();

        static void setState (State state);
    };
};

#endif /* APP_DATA_H */
