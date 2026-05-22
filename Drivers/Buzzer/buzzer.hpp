#pragma once
#include <cstdint>
#include <stdio.h>

namespace buzzer {

inline void push_buzzer (size_t* durations, bool* is_on, int& offset, size_t duration, bool status) {
    durations[offset] = duration;
    is_on[offset] = status;

    offset ++;
}

/**
 * Preambule
 *  - repeat 3 times
 *    - 100ms on, 100ms off
 *  - 10 seconds off
 */
const size_t NB_BLOCKS_PREAMBULE = 3 * 2 + 1;
inline void setup_preambule (size_t* durations, bool* is_on, int& offset) {
    for (int i = 0; i < 3; i ++) {
        push_buzzer(durations, is_on, offset, 100, true);
        push_buzzer(durations, is_on, offset, 100, false);
    }
    push_buzzer(durations, is_on, offset, 10 * 1000, false);
}

/**
 * Per module
 *  - 250ms on
 *  - 750ms off
 *  - repeat 5 times
 *    - 100ms (on if module okay else off)
 *    - 100ms off
 *  - 1s off
 */
const size_t NB_BLOCKS_PER_MODULE = 2 + 5 * 2 + 1;
inline void setup_module (size_t* durations, bool* is_on, int& offset, bool module_ok) {
    push_buzzer(durations, is_on, offset, 250, true);
    push_buzzer(durations, is_on, offset, 750, false);

    for (int i = 0; i < 5; i ++) {
        push_buzzer(durations, is_on, offset, 100, module_ok);
        push_buzzer(durations, is_on, offset, 100, false);
    }
    
    push_buzzer(durations, is_on, offset, 1000, false);
}

/**
 * Post
 *  - 10 seconds off
 */
const size_t NB_BLOCKS_POST = 1;
inline void setup_post (size_t* durations, bool* is_on, int& offset) {
    push_buzzer(durations, is_on, offset, 10 * 1000, false);
}

/**
 * Compute the number of steps inside the buzzer.
 * 
 * @param N_els number of modules to show
 * @return the number of steps
 */
constexpr size_t number_steps (size_t N_els) {
    return NB_BLOCKS_PREAMBULE + N_els * NB_BLOCKS_PER_MODULE + NB_BLOCKS_POST;
}

template<size_t N_els>
struct Buzzer {
private:
    bool started = false;

    size_t  offset = 0;
    ssize_t block_start_time = 0;

    size_t block_duration[number_steps(N_els)];
    bool   block_on[number_steps(N_els)];

    void (*enable_buzzer) (bool is_on);
    
    void end_block () {
        block_start_time += block_duration[offset];
        offset ++;
        
        if (offset == number_steps(N_els)) {
            enable_buzzer(false);
            return ;
        }

        enable_buzzer(block_on[offset]);
    }
public:
    void tick (ssize_t current_time) {
        if (!started || offset == number_steps(N_els)) {
            return ;
        }

        if (current_time >= block_duration[offset] + block_start_time) {
            end_block();
        }
    }

    bool is_finished () {
        return offset == number_steps(N_els);
    }

    template<typename... Args>
    void start(ssize_t current_time, void (*enable_buzzer)(bool), Args... args) {
        static_assert(sizeof...(Args) == N_els, "Wrong number of args");

        this->enable_buzzer = enable_buzzer;
        started = true;

        const bool temp[N_els] = { static_cast<bool>(args)... };
        
        int offset = 0;
        setup_preambule(block_duration, block_on, offset);

        for (int i_el = 0; i_el < N_els; i_el ++) {
            setup_module(block_duration, block_on, offset, temp[i_el]);
        }

        setup_post(block_duration, block_on, offset);

        /*printf("Number elements: %d\r\n", number_steps(N_els));
        for (int i = 0; i < number_steps(N_els); i ++) {
        	printf("%d %d\r\n", block_duration[i], block_on[i]);
        }*/

        block_start_time = current_time;
        enable_buzzer(block_on[0]);
    }
};

};
