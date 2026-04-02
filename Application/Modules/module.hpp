#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace modules {

// =============================================================================
// Internal DataBuffer duck-typing trait (C++17)
//
// Fires a static_assert at Module instantiation time if BufferT does not
// expose the required surface:  append(const T&), get(size_t), size(),
// empty(), full().
//
// Usage inside Module:
//   static_assert(detail::is_data_buffer<BufferT, DataT>::value, "...");
// =============================================================================
namespace detail {

template <typename B, typename T, typename = void>
struct is_data_buffer : std::false_type {};

template <typename B, typename T>
struct is_data_buffer<
    B, T,
    std::void_t<
        // append(const T&)
        decltype(std::declval<B>().append(std::declval<const T &>())),
        // get(size_t) -> const T* (or pointer-convertible)
        decltype(std::declval<const B>().get(std::declval<size_t>())),
        // size() -> size_t (or integral)
        decltype(std::declval<const B>().size()),
        // empty() -> bool (or bool-convertible)
        decltype(std::declval<const B>().empty()),
        // full() -> bool (or bool-convertible)
        decltype(std::declval<const B>().full())>>
    : std::true_type {};

}  // namespace detail

// =============================================================================
// Module<DriverT, BufferT, N>
//
// Abstract base class for all sensor modules.
//
// Owns N non-owning driver pointers (callers keep the driver objects alive)
// and N in-place BufferT instances. No heap allocation; all storage is
// part of the Module object itself.
//
// Subclasses implement:
//   bool init()                 – one-time hardware initialisation
//   void update(uint32_t tick)  – called every scheduler tick
//
// Template parameters:
//   DriverT   Abstract or concrete driver type (e.g. InvIMU_Interface,
//             UbxGpsInterface). The Module stores raw pointers to DriverT.
//   BufferT   Buffer type for sensor data. Must expose:
//               append(const DataT&), get(size_t) -> const DataT*,
//               size(), empty(), full().
//             Both RingBuffer<DataT, Cap> and ArrayBuffer<DataT, Cap>
//             satisfy this requirement.
//   N         Number of sensor instances managed by this module (>= 1).
//
// Memory note:
//   Module subclasses can be large (e.g. 3 x RingBuffer<IMUData,100> ≈ 12 KB).
//   Declare them as static or file-scope variables, never as local variables
//   on the call stack.
// =============================================================================
template <typename DriverT, typename BufferT, size_t N>
class Module {
    static_assert(N > 0, "Module must manage at least one driver instance");

  public:
    /**
     * @brief Construct a Module from an array of N driver pointers.
     *
     * The caller retains ownership of the driver objects; this Module stores
     * raw pointers only. All N pointers must remain valid for the lifetime of
     * this Module.
     *
     * @param drivers  C-array of exactly N DriverT pointers.
     */
    explicit Module(DriverT *(&drivers)[N], BufferT *(&buffers)[N]) {
        for (size_t i = 0; i < N; ++i) {
            drivers_[i] = drivers[i];
            buffers_[i] = buffers[i];
        }
    }

    virtual ~Module() = default;

    // Modules are not copyable or movable: they contain large in-place buffer
    // arrays and represent unique hardware interfaces.
    Module(const Module &)            = delete;
    Module &operator=(const Module &) = delete;
    Module(Module &&)                 = delete;
    Module &operator=(Module &&)      = delete;

    // -------------------------------------------------------------------------
    // Interface to implement in subclasses
    // -------------------------------------------------------------------------

    /**
     * @brief One-time hardware initialisation for all N drivers.
     *
     * Called once before the main loop starts. Implementations should
     * initialise each driver and return false if any critical driver fails.
     *
     * @return true if all required drivers initialised successfully.
     */
    virtual bool init() = 0;

    /**
     * @brief Poll all N drivers and fill their respective buffers.
     *
     * Called every scheduler tick. Implementations should read new data from
     * each driver and append it to the corresponding buffers_[i].
     *
     * @param tick_ms  Elapsed system time in milliseconds (from HAL_GetTick
     *                 or equivalent).
     */
    virtual void update(uint32_t tick_ms) = 0;

  protected:
    DriverT *drivers_[N]{};  ///< Non-owning pointers to the N driver instances.
    BufferT  *buffers_[N]{};

    /** Compile-time constant exposing N to subclasses (for loop bounds). */
    static constexpr size_t kNumSensors = N;
};

}  // namespace modules
