
#ifndef STM32_SIM_DEF
#define STM32_SIM_DEF

#define SIMULATOR_NAMESPACE simulator_fc
#define USING_SIMULATOR_NAMESPACE using namespace simulator_fc

typedef enum
{
    HAL_OK       = 0x00,
    HAL_ERROR    = 0x01,
    HAL_BUSY     = 0x02,
    HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

namespace SIMULATOR_NAMESPACE {
    enum SimStatus {
        SIM_OK,
        SIM_ERROR
    };
};

#endif /* STM32_SIM_DEF */
