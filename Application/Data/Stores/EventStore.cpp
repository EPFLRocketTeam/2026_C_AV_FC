#include "Application/Data/data.hpp"

using namespace flight_computer;

Event::Event()
:   command_updated(false),
    calibrated(false),
    dpr_eth_ready(false),
    dpr_eth_pressure_ok(false),
    dpr_lox_ready(false),
    dpr_lox_pressure_ok(false),
    prb_ready(false),
    trb_ready(false),
    ignited(false),
    seperated(false),
    chute_unreefed(false),
    ignition_failed(false),
    catastrophic_failure(false)
{}

EventStore::EventStore() {}

bool EventStore::get_command_updated() const { return data_.command_updated; }
void EventStore::set_command_updated(bool value) { data_.command_updated = value; }

bool EventStore::get_calibrated() const { return data_.calibrated; }
void EventStore::set_calibrated(bool value) { data_.calibrated = value; }

bool EventStore::get_dpr_eth_ready() const { return data_.dpr_eth_ready; }
void EventStore::set_dpr_eth_ready(bool value) { data_.dpr_eth_ready = value; }

bool EventStore::get_dpr_eth_pressure_ok() const { return data_.dpr_eth_pressure_ok; }
void EventStore::set_dpr_eth_pressure_ok(bool value) { data_.dpr_eth_pressure_ok = value; }

bool EventStore::get_dpr_lox_ready() const { return data_.dpr_lox_ready; }
void EventStore::set_dpr_lox_ready(bool value) { data_.dpr_lox_ready = value; }

bool EventStore::get_dpr_lox_pressure_ok() const { return data_.dpr_lox_pressure_ok; }
void EventStore::set_dpr_lox_pressure_ok(bool value) { data_.dpr_lox_pressure_ok = value; }

bool EventStore::get_prb_ready() const { return data_.prb_ready; }
void EventStore::set_prb_ready(bool value) { data_.prb_ready = value; }

bool EventStore::get_trb_ready() const { return data_.trb_ready; }
void EventStore::set_trb_ready(bool value) { data_.trb_ready = value; }

bool EventStore::get_ignited() const { return data_.ignited; }
void EventStore::set_ignited(bool value) { data_.ignited = value; }

bool EventStore::get_seperated() const { return data_.seperated; }
void EventStore::set_seperated(bool value) { data_.seperated = value; }

bool EventStore::get_chute_unreefed() const { return data_.chute_unreefed; }
void EventStore::set_chute_unreefed(bool value) { data_.chute_unreefed = value; }

bool EventStore::get_ignition_failed() const { return data_.ignition_failed; }
void EventStore::set_ignition_failed(bool value) { data_.ignition_failed = value; }

bool EventStore::get_catastrophic_failure() const { return data_.catastrophic_failure; }
void EventStore::set_catastrophic_failure(bool value) { data_.catastrophic_failure = value; }
