
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

State* StateStore::get_ref() 
{
    return &state_;
}

GOATStore::GOATStore() {
	stateStore = StateStore{};
	gpsStore   = GpsStore{};
}

const DataDump& GOATStore::get() const {
    data_.av_state = &stateStore.get();
    data_.gps_state = &gpsStore.get();
    return data_;
}

void GOATStore::set(const DataDump& value) {
        stateStore.set(*value.av_state);
        gpsStore.set(*value.gps_state);
}

DataDump* GOATStore::get_ref() 
{
    return &data_;
}
