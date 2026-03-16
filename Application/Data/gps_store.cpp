#include "Application/Data/data.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"

using namespace flight_computer;

GpsStore::GpsStore()
{data_ = {};}


// --------- Helpers par champ ---------

void GpsStore::setLon(int32_t lon)
{
    data_.lon = lon;
}

int32_t GpsStore::getLon() const
{
    return data_.lon;
}

void GpsStore::setLat(int32_t lat)
{
    data_.lat = lat;
}

int32_t GpsStore::getLat() const
{
    return data_.lat;
}

void GpsStore::setHAcc(uint32_t hAcc)
{
    data_.hAcc = hAcc;
}

uint32_t GpsStore::getHAcc() const
{
    return data_.hAcc;
}

void GpsStore::setNumSV(uint8_t numSV)
{
    data_.numSV = numSV;
}

uint8_t GpsStore::getNumSV() const
{
    return data_.numSV;
}

void GpsStore::setFixType(GpsFixType type)
{
    data_.fixType = type;
}

GpsFixType GpsStore::getFixType() const
{
    return data_.fixType;
}

void GpsStore::setValidity(GpsValidityFlags valid)
{
    data_.valid = valid;
}

GpsValidityFlags GpsStore::getValidity() const
{
    return data_.valid;
}

void GpsStore::setStatusFlags(GpsStatusFlags flags)
{
    data_.flags = flags;
}

GpsStatusFlags GpsStore::getStatusFlags() const
{
    return data_.flags;
}
