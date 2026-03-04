
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

State StateStore::get() const
{
    return state_;
}

GOATStore::GOATStore() {
	stateStore = StateStore{};
	gpsStore   = GpsStore{};
}

DataDump GOATStore::get() const {
    return {
        stateStore.get(),
        gpsStore.get(),
    };
}
