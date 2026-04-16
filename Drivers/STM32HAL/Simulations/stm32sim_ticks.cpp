#include "Drivers/STM32HAL/Simulations/stm32sim_ticks.hpp"

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>

static std::atomic<uint32_t> g_tick{0};
static std::atomic<bool> g_running{false};
static std::thread g_thread;

static void tick_thread()
{
    using namespace std::chrono;

    while (g_running.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(milliseconds(1));
        g_tick.fetch_add(1, std::memory_order_relaxed);
    }
}

void stm32sim_ticks_init()
{
    if (g_running.load())
    {
        throw std::logic_error(
            "stm32sim_ticks_init() called twice without deinit()");
    }

    g_tick.store(0);
    g_running.store(true);
    g_thread = std::thread(tick_thread);
}

void stm32sim_ticks_deinit()
{
    if (!g_running.load())
    {
        throw std::logic_error(
            "stm32sim_ticks_deinit() called without init()");
    }

    g_running.store(false);

    if (g_thread.joinable())
        g_thread.join();
}

uint32_t HAL_GetTick()
{
    return g_tick.load();
}

void HAL_Delay(uint32_t Delay)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < Delay)
    {
        std::this_thread::yield();
    }
}
