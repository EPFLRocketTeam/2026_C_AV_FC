
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

StateStore::StateStore()
: state_(State::INIT)
{}

void StateStore::set(const State& value)
{
    state_ = value;
}

const State& StateStore::get() const
{
    return state_;
}
