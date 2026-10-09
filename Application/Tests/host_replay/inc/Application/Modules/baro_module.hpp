#pragma once
#include "Application/Data/ring_buffer.hpp"
#include "Drivers/BMP390/BMP390.h"
extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData1;
extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData2;
extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData3;
extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData4;
