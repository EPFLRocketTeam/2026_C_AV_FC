#pragma once
#include <cstdint>
#include <stdio.h>

namespace buzzer {

/*
 * Each module has a unique acoustic signature:
 *   group long pulses (250 ms) + idx short pulses (80 ms) + status tone
 *
 *   Module        group  idx   Heard as
 *   Buzzer Test     0     1    ■
 *   Superloop       0     2    ■■
 *   IMU 1           1     1    ▬ ■
 *   IMU 2           1     2    ▬ ■■
 *   IMU 3           1     3    ▬ ■■■
 *   IMU 4           1     4    ▬ ■■■■
 *   BMP 1           2     1    ▬▬ ■
 *   BMP 2           2     2    ▬▬ ■■
 *   BMP 3           2     3    ▬▬ ■■■
 *   BMP 4           2     4    ▬▬ ■■■■
 *   GPS             0     3    ■■■
 *   Kalman          0     4    ■■■■
 *   Ready           3     0    ▬▬▬  (fanfare)
 *
 * Status: OK = 500 ms long tone.  FAIL = two 100 ms quick beeps.
 */
struct ModuleId {
    uint8_t group;       // 0-3: number of long (250 ms) identity pulses
    uint8_t idx;         // 0-4: number of short (80 ms) identity pulses
    bool    finale = false; // play "shave and a haircut" instead
};

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

/*
 * Per-module pattern — exactly 20 blocks regardless of group/idx:
 *   3 group slots  x 2 blocks =  6   (250 ms on / 150 ms off, or 1 ms pad)
 *   4 index slots  x 2 blocks =  8   ( 80 ms on /  60 ms off, or 1 ms pad)
 *   1 pre-status gap          =  1   (300 ms off)
 *   4 status blocks           =  4   (OK: long tone; FAIL: two quick beeps)
 *   1 inter-module gap        =  1   (500 ms off)
 *                              ---
 *                               20
 */
const size_t NB_BLOCKS_PER_MODULE = 3*2 + 4*2 + 1 + 4 + 1; // 20
inline void setup_module (size_t* d, bool* on, int& ofs, uint8_t group, uint8_t idx, bool ok) {
    // group identity: long pulses
    for (int i = 0; i < 3; i++) {
        if (i < (int)group) {
            push_buzzer(d, on, ofs, 250, true);
            push_buzzer(d, on, ofs, 150, false);
        } else {
            push_buzzer(d, on, ofs, 1, false);
            push_buzzer(d, on, ofs, 1, false);
        }
    }
    // index identity: short pulses
    for (int i = 0; i < 4; i++) {
        if (i < (int)idx) {
            push_buzzer(d, on, ofs, 80, true);
            push_buzzer(d, on, ofs, 60, false);
        } else {
            push_buzzer(d, on, ofs, 1, false);
            push_buzzer(d, on, ofs, 1, false);
        }
    }
    // pre-status gap
    push_buzzer(d, on, ofs, 300, false);
    // status — always 4 blocks so total block count stays constant
    if (ok) {
        push_buzzer(d, on, ofs, 500, true);
        push_buzzer(d, on, ofs,   1, false);
        push_buzzer(d, on, ofs,   1, false);
        push_buzzer(d, on, ofs, 400, false);
    } else {
        push_buzzer(d, on, ofs, 100, true);
        push_buzzer(d, on, ofs, 100, false);
        push_buzzer(d, on, ofs, 100, true);
        push_buzzer(d, on, ofs, 400, false);
    }
    // inter-module gap
    push_buzzer(d, on, ofs, 500, false);
}

/**
 * Compute the number of steps inside the buzzer.
 * 
 * @param N_els number of modules to show
 * @return the number of steps
 */
// "Shave and a haircut, two bits!" — exactly NB_BLOCKS_PER_MODULE (20) blocks.
inline void setup_finale (size_t* d, bool* on, int& ofs) {
	 push_buzzer(d, on, ofs, 200, true);  // Shave
	 push_buzzer(d, on, ofs,  80, false);
	 push_buzzer(d, on, ofs, 100, true);  // and
	 push_buzzer(d, on, ofs,  80, false);
	 push_buzzer(d, on, ofs, 100, true);  // a
	 push_buzzer(d, on, ofs,  80, false);
	 push_buzzer(d, on, ofs, 200, true);  // hair-
	 push_buzzer(d, on, ofs,  80, false);
	 push_buzzer(d, on, ofs, 200, true);  // -cut
	 push_buzzer(d, on, ofs, 350, false); // (pause)
	 push_buzzer(d, on, ofs, 100, true);  // two
	 push_buzzer(d, on, ofs,  80, false);
	 push_buzzer(d, on, ofs, 300, true);  // bits!
	 push_buzzer(d, on, ofs, 500, false); // end silence
	 for (int p = 0; p < 6; p++) push_buzzer(d, on, ofs, 1, false); // pad to 20
}

constexpr size_t number_steps (size_t N_els) {
    return NB_BLOCKS_PREAMBULE + N_els * NB_BLOCKS_PER_MODULE;
}

template<size_t N_els>
struct Buzzer {
private:
    bool started = false;

    size_t  offset = 0;
    ssize_t block_start_time = 0;

    size_t block_duration[number_steps(N_els)];
    bool   block_on[number_steps(N_els)];

    ModuleId m_ids[N_els];

    void (*enable_buzzer) (bool is_on);

    void end_block () {
        block_start_time += (ssize_t)block_duration[offset];
        offset ++;

        if (offset == number_steps(N_els)) {
            enable_buzzer(false);
            return ;
        }

        enable_buzzer(block_on[offset]);
    }
public:
    Buzzer() {
        for (size_t i = 0; i < N_els; i++) m_ids[i] = {0, 0};
    }

    // Call once before start() to assign each module's acoustic identity.
    void configure(const ModuleId* ids) {
        for (size_t i = 0; i < N_els; i++) m_ids[i] = ids[i];
    }

    void tick (ssize_t current_time) {
        if (!started || offset == number_steps(N_els)) {
            return ;
        }

        if (current_time >= (ssize_t)block_duration[offset] + block_start_time) {
            end_block();
        }
    }

    template<typename... Args>
    void start(ssize_t current_time, void (*enable_buzzer)(bool), Args... args) {
        static_assert(sizeof...(Args) == N_els, "Wrong number of args");

        this->enable_buzzer = enable_buzzer;
        started = true;
        offset  = 0;

        const bool temp[N_els] = { static_cast<bool>(args)... };

        int ofs = 0;
        setup_preambule(block_duration, block_on, ofs);

        for (size_t i = 0; i < N_els; i++) {
            if (m_ids[i].finale)
                setup_finale(block_duration, block_on, ofs);
            else
                setup_module(block_duration, block_on, ofs,
                             m_ids[i].group, m_ids[i].idx, temp[i]);
        }

        block_start_time = current_time;
        enable_buzzer(block_on[0]);
    }
};

};
