
#include "Application/Data/data.hpp"

using namespace flight_computer;

Data::Data () {
    state = State::INIT;
}

static Data globalData = Data();

Data DataManager::read () {
    return globalData;
}
void DataManager::setState (State state) {
    globalData.state = state;
}
