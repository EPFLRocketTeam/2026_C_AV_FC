/*
 *  bmp390.h - Driver for the Bosch BMP390 pressure sensor
 *
 *  Created by Elie "Fayorg" Baier on the 10th of December, 2025.
*/

#define BMP390_CHIP_ID              0x60U
#define BMP390_I2C_ADDR_SDO_LOW     (0x76U << 1U)  /**< SDO → GND,   HAL 8-bit addr */
#define BMP390_I2C_ADDR_SDO_HIGH    (0x77U << 1U)  /**< SDO → VDDIO, HAL 8-bit addr */

/**
 * @brief Result codes for BMP390 operations
 */
typedef enum {
    BMP390_OK = 0,
    BMP390_E_NULLPTR,
    BMP390_E_INVALID_PARAM,
    BMP390_E_BUS,
    BMP390_E_TIMEOUT,
    BMP390_E_CHIP_ID,
    BMP390_E_CONFIG,
} BMP390_Result;

/**
 * @brief Power modes for BMP390
 */
typedef enum {
    BMP390_POWERMODE_SLEEP = 0x00U,
    BMP390_POWERMODE_FORCED = 0x01U,
    BMP390_POWERMODE_NORMAL = 0x03U,
} BMP390_PowerMode;

/**
 * @brief Oversampling settings for BMP390
 */
typedef enum {
    BMP390_OSR_X1  = 0x00U,
    BMP390_OSR_X2  = 0x01U,
    BMP390_OSR_X4  = 0x02U,
    BMP390_OSR_X8  = 0x03U,
    BMP390_OSR_X16 = 0x04U,
    BMP390_OSR_X32 = 0x05U,
} BMP390_Oversampling;

/**
 * @brief IIR filter settings for BMP390
 */
typedef enum {
    BMP390_IIR_OFF = 0x00U,
    BMP390_IIR_1   = 0x01U,
    BMP390_IIR_3   = 0x02U,
    BMP390_IIR_7   = 0x03U,
    BMP390_IIR_15  = 0x04U,
    BMP390_IIR_31  = 0x05U,
    BMP390_IIR_63  = 0x06U,
    BMP390_IIR_127 = 0x07U,
} BMP390_IIRFilter;

/**
 * @brief Output data rates for BMP390
 */
typedef enum {
    BMP390_ODR_200_HZ = 0x00U,
    BMP390_ODR_100_HZ = 0x01U,
    BMP390_ODR_50_HZ = 0x02U,
    BMP390_ODR_25_HZ = 0x03U,
    BMP390_ODR_12P5_HZ = 0x04U,
    BMP390_ODR_6P25_HZ = 0x05U,
    BMP390_ODR_3P125_HZ = 0x06U,
    BMP390_ODR_1P5625_HZ = 0x07U,
    BMP390_ODR_0P78125_HZ = 0x08U,
    BMP390_ODR_0P390625_HZ = 0x09U,
    BMP390_ODR_0P1953125_HZ = 0x0AU,
    BMP390_ODR_0P09765625_HZ = 0x0BU,
    BMP390_ODR_0P048828125_HZ = 0x0CU,
    BMP390_ODR_0P0244140625_HZ = 0x0DU,
    BMP390_ODR_0P01220703125_HZ = 0x0EU,
    BMP390_ODR_0P006103515625_HZ = 0x0FU,
    BMP390_ODR_0P0030517578125_HZ = 0x10U,
    BMP390_ODR_0P00152587890625_HZ = 0x11U,
} BMP390_OutputDataRate;

/**
 * @brief Calibration data for BMP390
 */
typedef struct {
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;
    float t_lin;
} BMP390_CalibrationData;

/**
 * @brief Configuration for BMP390
 */
typedef struct {
    bool pressure_enable;
    bool temperature_enable;
    BMP390_Oversampling pressure_oversampling;
    BMP390_Oversampling temperature_oversampling;
    BMP390_OutputDataRate output_data_rate;
    BMP390_IIRFilter iir_filter;
    BMP390_PowerMode power_mode;
} BMP390_Config;

typedef struct {
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    float pressure_pa;
    float temperature_deg_c;
} BMP390_Measurement;

typedef struct {
    SPI_HandleTypeDef *spi4;
    GPIO_TypeDef *chip_select_port;
    uint16_t chip_select_pin;
    uint32_t timeout_ms;
    BMP390_CalibrationData calibration;
    BMP390_Config config;
    uint8_t chip_id;
    bool initialized;
} BMP390_Device;

class BMP390 {
    public:
        static constexpr uint8_t CHIP_ID = 0x60U;

        BMP390(PI_HandleTypeDef *spi4_handle, GPIO_TypeDef *chip_select_port, uint16_t chip_select_pin, uint32_t timeout_ms);

        BMP390_Result Init();
        BMP390_Result SoftReset();
        BMP390_Result Configure(const BMP390_Config *config);
        BMP390_Result SetPowerMode(BMP390_PowerMode power_mode);
        BMP390_Result TriggerForcedMeasurement();
        BMP390_Result ReadMeasurement(BMP390_Measurement *measurement);
        BMP390_Result PerformForcedMeasurement(BMP390_Measurement *measurement);

        uint8_t ChipId() const;
        bool IsInitialized() const;
        const CalibrationData &GetCalibration() const;
        const Config &GetConfig() const;
    
    private:
        static constexpr uint16_t CALIB_DATA_LENGTH = 21U;
        static constexpr uint16_t SPI_STACK_LIMIT = 66U;

        bool HasValidHardware() const;
        void Select() const;
        void Deselect() const;

        Result ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length);
        BMP390_Result WriteRegisters(uint8_t reg, const uint8_t *data, uint16_t length);
        BMP390_Result WritePowerControl(bool pressure_enable, bool temperature_enable, PowerMode mode);
        BMP390_Result ReadChipId(uint8_t &chip_id);
        BMP390_Result ReadCalibration();
        BMP390_Result ReadRaw(uint32_t &raw_pressure, uint32_t &raw_temperature);

        SPI_HandleTypeDef *m_spi4;
        GPIO_TypeDef *m_chip_select_port;
        uint16_t m_chip_select_pin;
        uint32_t m_timeout_ms;
        CalibrationData m_calibration;
        Config m_config;
        uint8_t m_chip_id;
        bool m_initialized;
}