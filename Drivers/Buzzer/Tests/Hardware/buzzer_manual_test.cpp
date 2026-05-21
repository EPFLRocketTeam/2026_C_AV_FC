
extern "C" {
    #include "../../../STM32HAL/stm32hal.h"
    #include "Core/Inc/main.h"
    #include "./buzzer_manual_test.h"
}

#include "../../buzzer.hpp"

void manual_test_buzzer_set_buzzer (bool status) {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, status ? GPIO_PIN_SET : GPIO_PIN_RESET);

    printf("[BUZZER] Set at time %d: %d\r\n", HAL_GetTick(), status);
}
void manual_test_buzzer () {
    bool started = false;
    
    buzzer::Buzzer<4> buzzer;
    while (HAL_GetTick() < 100 * 1000) {
        buzzer.tick(HAL_GetTick());
        if (!started && HAL_GetTick() > 10 * 1000) {
            buzzer.start(
                HAL_GetTick(),
                manual_test_buzzer_set_buzzer,
                1, 1, 0, 1
            );

            started = true;
        }
    }
}
