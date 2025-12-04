/*
 * ubx_gps_interface.hpp
 *
 *  Created on: Nov 25, 2025
 *      Author: Maxime Rochat
 */

#ifndef INC_UBX_GPS_INTERFACE_HPP_
#define INC_UBX_GPS_INTERFACE_HPP_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

// ============================================================================
// CONSTANTS & SCALING FACTORS
// ============================================================================

/** Protocol Constants */
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07

// --- Configuration Keys for UART1 (See M10 Interface Docs Sec 4.9) ---

/** CFG-UART1OUTPROT-NMEA: Disable NMEA (Key ID: 0x10740002) */
#define CFG_KEY_UART1OUTPROT_NMEA 0x10740002

/** CFG-MSGOUT-UBX_NAV_PVT_UART1: Enable PVT (Key ID: 0x20910007) */
#define CFG_KEY_MSGOUT_NAV_PVT_UART1 0x20910007

/** CFG-RATE-MEAS: Measurement Rate in ms (Key ID: 0x30210001) */
#define CFG_KEY_RATE_MEAS 0x30210001

/** Scaling Factors (See Docs Section 3.15.11) */
#define UBX_SCALE_LAT_LON 1e-7
#define UBX_SCALE_HEADING 1e-5
#define UBX_SCALE_ALTITUDE 1
#define UBX_SCALE_SPEED 1

// ============================================================================
// ENUMS
// ============================================================================

enum class GpsStatus {
  OK = 0,
  ERROR_UART,
  ERROR_TIMEOUT,
  ERROR_CHECKSUM,
  ERROR_CONFIG,
};

/** GNSS Fix Type */
enum class GpsFixType {
  NO_FIX = 0,
  DEAD_RECKONING_ONLY = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  GNSS_DEAD_RECKONING = 4,
  TIME_ONLY = 5
};

/** Power Save Mode State */
enum class GpsPowerSaveMode {
  NOT_ACTIVE = 0,
  ENABLED = 1,
  ACQUISITION = 2,
  TRACKING = 3,
  POWER_OPTIMIZED = 4,
  INACTIVE = 5
};

/** Carrier Phase Solution Status */
enum class GpsCarrierPhaseStatus {
  NONE = 0,
  FLOAT = 1,
  FIXED = 2
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct GpsValidityFlags {
  bool validDate;
  bool validTime;
  bool fullyResolved;
  bool validMag;
};

struct GpsStatusFlags {
  bool gnssFixOK;
  bool diffSoln;
  GpsPowerSaveMode psmState;
  bool headVehValid;
  GpsCarrierPhaseStatus carrSoln;
};

/**
 * The main PVT (Position Velocity Time) Data Structure
 * Corresponds to UBX-NAV-PVT payload (92 bytes)
 */
struct GpsPvtData {
  // --- Time ---
  uint32_t iTOW;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  int32_t nano;
  uint32_t tAcc;

  // --- Validity ---
  GpsValidityFlags valid;

  // --- Fix Info ---
  GpsFixType fixType;
  GpsStatusFlags flags;
  uint8_t numSV;

  // --- Position ---
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;

  // --- Velocity ---
  int32_t velN;
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;

  // --- DOP ---
  uint16_t pDOP;
};

struct GpsBasicFixData {
  // --- Validity ---
  GpsValidityFlags valid;

  // --- Fix Info ---
  GpsFixType fixType;
  GpsStatusFlags flags;
  uint8_t numSV;

  // --- Position ---
  int32_t lon;
  int32_t lat;
  uint32_t hAcc;
};

// ============================================================================
// GPS INTERFACE CLASS
// ============================================================================

class UbxGpsInterface {
public:
  /**
   * @brief Construct a new GPS Interface
   * @param huart UART handle for GPS communication
   * @param rate_ms Measurement rate in milliseconds (default: 1000ms = 1Hz)
   */
  UbxGpsInterface(UART_HandleTypeDef *huart, uint16_t rate_ms = 1000);

  /**
   * @brief Initialize the GPS and configure it to output PVT messages
   *
   * Configures:
   * 1. CFG-UART1OUTPROT-NMEA = 0 (Disable NMEA)
   * 2. CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1 (Enable Binary PVT)
   * 3. Navigation Rate = configured rate
   *
   * @return GpsStatus::OK on success, error code otherwise
   */
  GpsStatus init();

  /**
   * @brief Set the GPS measurement rate
   * @param period_ms Measurement period in milliseconds (e.g., 1000 for 1Hz)
   * @return GpsStatus::OK on success, error code otherwise
   */
  GpsStatus setRate(uint16_t period_ms);

  /**
   * @brief Get the current measurement rate
   * @return Current rate in milliseconds
   */
  uint16_t getRate() const { return rate_ms_; }

  /**
   * @brief Listen to UART stream for a PVT message
   *
   * Blocking call that scans UART RX stream for UBX-NAV-PVT header.
   *
   * @param[out] pvt_data Pointer to struct where data will be written
   * @param[in] timeout_ms Time to wait before giving up
   * @return GpsStatus::OK on success, error code otherwise
   */
  GpsStatus getPvt(GpsBasicFixData *pvt_data, uint32_t timeout_ms);

  /**
   * @brief Send a Stop command to the GPS
   * @return GpsStatus::OK on success, error code otherwise
   */
  GpsStatus stop();

private:
  // Constants
  static constexpr uint32_t GPS_TX_TIMEOUT = 100;
  static constexpr uint32_t GPS_BOOT_DELAY_MS = 200;
  static constexpr uint32_t GPS_RX_POLL_TIMEOUT = 10;

  static constexpr uint8_t UBX_CLASS_CFG = 0x06;
  static constexpr uint8_t UBX_ID_CFG_VALSET = 0x8A;
  static constexpr uint8_t UBX_ID_CFG_RST = 0x04;
  static constexpr uint8_t UBX_CFG_LAYER_RAM = 0x01;
  static constexpr uint8_t UBX_RST_MODE_GNSS_STOP = 0x08;

  static constexpr uint8_t UBX_PAYLOAD_LEN_CFG_VALSET8 = 9;
  static constexpr uint8_t UBX_PAYLOAD_LEN_CFG_VALSET16 = 10;
  static constexpr uint8_t UBX_NAV_PVT_PAYLOAD_LEN = 92;
  static constexpr uint8_t UBX_CFG_RST_PAYLOAD_LEN = 4;

  static constexpr uint8_t PKT_OFF_CLASS = 2;

  // Parser states
  enum ParserState {
    STATE_SYNC_1 = 0,
    STATE_SYNC_2,
    STATE_CLASS,
    STATE_ID,
    STATE_LEN_LSB,
    STATE_LEN_MSB,
    STATE_PAYLOAD,
    STATE_CK_A,
    STATE_CK_B
  };

  // Member variables
  UART_HandleTypeDef *uart_handle_;
  uint16_t rate_ms_;

  // Helper methods
  void calcChecksum(const uint8_t *buffer, uint16_t size, uint8_t *ck_a,
                    uint8_t *ck_b);
  GpsStatus writeCfgVal8(uint32_t key_id, uint8_t value);
  GpsStatus writeCfgVal16(uint32_t key_id, uint16_t value);
  GpsStatus sendCommand(uint8_t msg_class, uint8_t msg_id,
                        const uint8_t *payload, uint16_t payload_len);
  GpsStatus resetReceiver(uint8_t reset_mode, uint16_t nav_bbr_mask);
  void parsePvt(const uint8_t *payload, GpsPvtData *data);
  void parseBasicFix(const uint8_t *payload, GpsBasicFixData *data);

  // Endian-safe extraction helpers
  static uint32_t getU4(const uint8_t *b) {
    return (uint32_t)(b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24));
  }
  static int32_t getI4(const uint8_t *b) {
    return (int32_t)(b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24));
  }
  static uint16_t getU2(const uint8_t *b) {
    return (uint16_t)(b[0] | (b[1] << 8));
  }
};

#endif /* INC_UBX_GPS_INTERFACE_HPP_ */
