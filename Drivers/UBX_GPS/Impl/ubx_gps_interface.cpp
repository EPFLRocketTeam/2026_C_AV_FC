#include "../ubx_gps_interface.h"
#include "Application/app_timebase.h"
#include <cstring>
#include <cstdlib>
#include <stdio.h>


// ============================================================================
// CONSTRUCTOR
// ============================================================================

UbxGpsInterface::UbxGpsInterface(UART_HandleTypeDef *huart,
                                 uint16_t rate_ms)
    : uart_handle_(huart), rate_ms_(rate_ms) {
  resetParserState();
}

// ============================================================================
// PUBLIC METHODS
// ============================================================================

GpsStatus UbxGpsInterface::init() {
  resetParserState();
  GpsStatus status;

  // 1. Disable NMEA on UART1
  status = writeCfgVal8(CFG_KEY_UART1OUTPROT_NMEA, 0);
  if (status != GpsStatus::OK)
    return status;

  // 2. Configure power mode (ktp-soft parity: high performance by default)
  status = configurePowerMode();
  if (status != GpsStatus::OK)
    return status;

  // 3. Configure constellation set
  status = configureConstellations();
  if (status != GpsStatus::OK)
    return status;

  // 4. Configure dynamic model
  status = configureDynamics();
  if (status != GpsStatus::OK)
    return status;

  // 5. Configure auto-output messages
  status = configureMessages();
  if (status != GpsStatus::OK)
    return status;

  // 6. Set configured rate
  status = setRate(rate_ms_);

  // Allow GPS to process configs
  HAL_Delay(GPS_BOOT_DELAY_MS);

  return status;
}

GpsStatus UbxGpsInterface::setRate(uint16_t period_ms) {
  if (period_ms < 25)
    return GpsStatus::ERROR_CONFIG;
  rate_ms_ = period_ms;
  return writeCfgVal16(CFG_KEY_RATE_MEAS, period_ms);
}

void printPayload(const uint8_t *payload, size_t payloadLen) {
  printf("Payload (%zu bytes):\n\r", payloadLen);

  for (size_t i = 0; i < payloadLen; ++i) {
    printf("%02X ", payload[i]);
    //printf("%c ", payload[i]);
    if ((i + 1) % 16 == 0) {
      printf("\r\n");
    }
  }

  if (payloadLen % 16 != 0) {
    printf("\n\r");
  }
}

GpsStatus UbxGpsInterface::getPvt(GpsBasicFixData *pvt_data,
                                  uint32_t timeout_ms) {
  if (pvt_data == nullptr) {
    return GpsStatus::ERROR_CONFIG;
  }

  uint8_t rx_byte = 0;
  const uint32_t start_tick = HAL_GetTick();

  auto processByte = [&](uint8_t byte) -> bool {
    switch (parser_state_) {
    case STATE_SYNC_1:
      if (byte == UBX_SYNC_CHAR_1) {
        parser_state_ = STATE_SYNC_2;
      }
      break;

    case STATE_SYNC_2:
      if (byte == UBX_SYNC_CHAR_2) {
        parser_state_ = STATE_CLASS;
      } else {
        resetParserState();
      }
      break;

    case STATE_CLASS:
      if (byte == UBX_CLASS_NAV) {
        parser_state_ = STATE_ID;
        parser_ck_a_calc_ = byte;
        parser_ck_b_calc_ = byte;
      } else {
        resetParserState();
      }
      break;

    case STATE_ID:
      if (byte == UBX_ID_NAV_PVT) {
        parser_state_ = STATE_LEN_LSB;
        parser_ck_a_calc_ += byte;
        parser_ck_b_calc_ += parser_ck_a_calc_;
      } else {
        resetParserState();
      }
      break;

    case STATE_LEN_LSB:
      if (byte == (uint8_t)(UBX_NAV_PVT_PAYLOAD_LEN & 0xFFU)) {
        parser_state_ = STATE_LEN_MSB;
        parser_ck_a_calc_ += byte;
        parser_ck_b_calc_ += parser_ck_a_calc_;
      } else {
        resetParserState();
      }
      break;

    case STATE_LEN_MSB:
      if (byte == (uint8_t)(UBX_NAV_PVT_PAYLOAD_LEN >> 8)) {
        parser_state_ = STATE_PAYLOAD;
        parser_ck_a_calc_ += byte;
        parser_ck_b_calc_ += parser_ck_a_calc_;
        parser_payload_idx_ = 0;
      } else {
        resetParserState();
      }
      break;

    case STATE_PAYLOAD:
      parser_payload_buf_[parser_payload_idx_++] = byte;
      parser_ck_a_calc_ += byte;
      parser_ck_b_calc_ += parser_ck_a_calc_;
      if (parser_payload_idx_ == UBX_NAV_PVT_PAYLOAD_LEN) {
        parser_state_ = STATE_CK_A;
      }
      break;

    case STATE_CK_A:
      if (byte == parser_ck_a_calc_) {
        parser_state_ = STATE_CK_B;
      } else {
        resetParserState();
      }
      break;

    case STATE_CK_B:
      if (byte == parser_ck_b_calc_) {
        parseBasicFix(parser_payload_buf_, pvt_data);
        const uint64_t now_us = app_timebase_now_us();
        pvt_data->timestamp_us = now_us;
        pvt_data->pps_timestamp_us = 0ULL;
        resetParserState();
        return true;
      }
      resetParserState();
      break;

    default:
      resetParserState();
      break;
    }

    return false;
  };

  if (timeout_ms == 0u) {
    uint32_t bytes_polled = 0;
    while (bytes_polled < GPS_RX_NONBLOCKING_MAX_BYTES &&
           HAL_UART_Receive(uart_handle_, &rx_byte, 1, 0u) == HAL_OK) {
      ++bytes_polled;
      if (processByte(rx_byte)) {
        return GpsStatus::OK;
      }
    }
    return GpsStatus::ERROR_TIMEOUT;
  }

  while (true) {
    const uint32_t elapsed_ms = HAL_GetTick() - start_tick;
    if (elapsed_ms >= timeout_ms) {
      break;
    }

    const uint32_t remaining_ms = timeout_ms - elapsed_ms;
    const uint32_t poll_timeout_ms =
        (remaining_ms < GPS_RX_POLL_TIMEOUT) ? remaining_ms : GPS_RX_POLL_TIMEOUT;

    if (HAL_UART_Receive(uart_handle_, &rx_byte, 1, poll_timeout_ms) == HAL_OK) {
      if (processByte(rx_byte)) {
        return GpsStatus::OK;
      }
    }
  }

  return GpsStatus::ERROR_TIMEOUT;
}

GpsStatus UbxGpsInterface::stop() {
  resetParserState();
  GpsStatus status;

  // 1. Disable UBX-NAV-PVT message output
  status = writeCfgVal8(CFG_KEY_MSGOUT_NAV_PVT_UART1, 0);
  if (status != GpsStatus::OK)
    return status;

  // 2. Send Controlled GNSS stop command
  status = resetReceiver(UBX_RST_MODE_GNSS_STOP, 0x0000);
  if (status != GpsStatus::OK)
    return status;

  HAL_Delay(50);

  return GpsStatus::OK;
}

void UbxGpsInterface::resetParserState() {
  parser_state_ = STATE_SYNC_1;
  parser_payload_idx_ = 0;
  parser_ck_a_calc_ = 0;
  parser_ck_b_calc_ = 0;
}

// ============================================================================
// PRIVATE HELPER METHODS
// ============================================================================

GpsStatus UbxGpsInterface::configurePowerMode() {
#if APP_GPS_HIGH_PERFORMANCE != 0u
  // 0 = full power / high performance mode.
  return writeCfgVal8(CFG_KEY_PM_OPERATEMODE, 0u);
#else
  return GpsStatus::OK;
#endif
}

GpsStatus UbxGpsInterface::configureConstellations() {
  const uint8_t constellations = static_cast<uint8_t>(APP_GPS_CONSTELLATIONS);

  GpsStatus status =
      writeCfgVal8(CFG_KEY_SIGNAL_GPS_ENA,
                   (constellations & GPS_CONSTELLATION_GPS) ? 1u : 0u);
  if (status != GpsStatus::OK)
    return status;

  status = writeCfgVal8(CFG_KEY_SIGNAL_GAL_ENA,
                        (constellations & GPS_CONSTELLATION_GALILEO) ? 1u
                                                                    : 0u);
  if (status != GpsStatus::OK)
    return status;

  status = writeCfgVal8(CFG_KEY_SIGNAL_GLO_ENA,
                        (constellations & GPS_CONSTELLATION_GLONASS) ? 1u
                                                                    : 0u);
  if (status != GpsStatus::OK)
    return status;

  return writeCfgVal8(CFG_KEY_SIGNAL_BDS_ENA,
                      (constellations & GPS_CONSTELLATION_BEIDOU) ? 1u : 0u);
}

GpsStatus UbxGpsInterface::configureDynamics() {
  return writeCfgVal8(CFG_KEY_NAVSPG_DYNMODEL,
                      static_cast<uint8_t>(APP_GPS_DYNAMIC_MODEL));
}

GpsStatus UbxGpsInterface::configureMessages() {
  const uint16_t messages = static_cast<uint16_t>(APP_GPS_MESSAGES);

  GpsStatus status =
      writeCfgVal8(CFG_KEY_MSGOUT_NAV_PVT_UART1,
                   (messages & GPS_MESSAGE_PVT) ? 1u : 0u);
  if (status != GpsStatus::OK)
    return status;

  status = writeCfgVal8(CFG_KEY_MSGOUT_NAV_DOP_UART1,
                        (messages & GPS_MESSAGE_DOP) ? 1u : 0u);
  if (status != GpsStatus::OK)
    return status;

  status = writeCfgVal8(CFG_KEY_MSGOUT_NAV_STATUS_UART1,
                        (messages & GPS_MESSAGE_STATUS) ? 1u : 0u);
  if (status != GpsStatus::OK)
    return status;

  return writeCfgVal8(CFG_KEY_MSGOUT_NAV_TIMEUTC_UART1,
                      (messages & GPS_MESSAGE_TIMEUTC) ? 1u : 0u);
}

GpsStatus UbxGpsInterface::sendCommand(uint8_t msg_class, uint8_t msg_id,
                                       const uint8_t *payload,
                                       uint16_t payload_len) {
  // Total: Sync(2) + Class(1) + ID(1) + Length(2) + Payload(N) + Checksum(2)
  uint16_t total_len = 2 + 1 + 1 + 2 + payload_len + 2;
  uint8_t *packet = (uint8_t *)malloc(total_len);
  if (packet == nullptr) {
    return GpsStatus::ERROR_CONFIG;
  }

  uint8_t *p = packet;

  // Header
  *p++ = UBX_SYNC_CHAR_1;
  *p++ = UBX_SYNC_CHAR_2;
  *p++ = msg_class;
  *p++ = msg_id;
  *p++ = (uint8_t)(payload_len & 0xFF);
  *p++ = (uint8_t)(payload_len >> 8);

  // Payload
  if (payload_len > 0 && payload != nullptr) {
    memcpy(p, payload, payload_len);
    p += payload_len;
  }

  // Checksum
  uint8_t ck_a, ck_b;
  calcChecksum(&packet[PKT_OFF_CLASS], 4 + payload_len, &ck_a, &ck_b);
  *p++ = ck_a;
  *p++ = ck_b;

  HAL_StatusTypeDef hal_status =
      HAL_UART_Transmit(uart_handle_, packet, total_len, GPS_TX_TIMEOUT);
  free(packet);

  if (hal_status != HAL_OK) {
    return GpsStatus::ERROR_UART;
  }
  return GpsStatus::OK;
}

GpsStatus UbxGpsInterface::resetReceiver(uint8_t reset_mode,
                                         uint16_t nav_bbr_mask) {
  uint8_t payload[UBX_CFG_RST_PAYLOAD_LEN];

  // navBbrMask (2 bytes, Little Endian)
  payload[0] = (uint8_t)(nav_bbr_mask & 0xFF);
  payload[1] = (uint8_t)((nav_bbr_mask >> 8) & 0xFF);

  // resetMode (1 byte)
  payload[2] = reset_mode;

  // reserved0 (1 byte)
  payload[3] = 0x00;

  return sendCommand(UBX_CLASS_CFG, UBX_ID_CFG_RST, payload,
                     UBX_CFG_RST_PAYLOAD_LEN);
}

GpsStatus UbxGpsInterface::writeCfgVal8(uint32_t key_id, uint8_t value) {
  uint8_t payload[UBX_PAYLOAD_LEN_CFG_VALSET8];
  uint8_t *p = payload;

  // Version(1) + Layer(1) + Reserved(2)
  *p++ = 0x00;
  *p++ = UBX_CFG_LAYER_RAM;
  *p++ = 0x00;
  *p++ = 0x00;

  // Key ID (Little Endian)
  *p++ = (uint8_t)(key_id & 0xFF);
  *p++ = (uint8_t)((key_id >> 8) & 0xFF);
  *p++ = (uint8_t)((key_id >> 16) & 0xFF);
  *p++ = (uint8_t)((key_id >> 24) & 0xFF);

  // Value (1 byte)
  *p++ = value;

  return sendCommand(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload,
                     sizeof(payload));
}

GpsStatus UbxGpsInterface::writeCfgVal16(uint32_t key_id, uint16_t value) {
  uint8_t payload[UBX_PAYLOAD_LEN_CFG_VALSET16];
  uint8_t *p = payload;

  // Version(1) + Layer(1) + Reserved(2)
  *p++ = 0x00;
  *p++ = UBX_CFG_LAYER_RAM;
  *p++ = 0x00;
  *p++ = 0x00;

  // Key ID (Little Endian)
  *p++ = (uint8_t)(key_id & 0xFF);
  *p++ = (uint8_t)((key_id >> 8) & 0xFF);
  *p++ = (uint8_t)((key_id >> 16) & 0xFF);
  *p++ = (uint8_t)((key_id >> 24) & 0xFF);

  // Value (2 bytes Little Endian)
  *p++ = (uint8_t)(value & 0xFF);
  *p++ = (uint8_t)((value >> 8) & 0xFF);

  return sendCommand(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload,
                     sizeof(payload));
}

void UbxGpsInterface::calcChecksum(const uint8_t *buffer, uint16_t size,
                                   uint8_t *ck_a, uint8_t *ck_b) {
  *ck_a = 0;
  *ck_b = 0;
  for (uint16_t i = 0; i < size; i++) {
    *ck_a = *ck_a + buffer[i];
    *ck_b = *ck_b + *ck_a;
  }
}

void UbxGpsInterface::parsePvt(const uint8_t *payload, GpsPvtData *data) {
  const uint8_t *p = payload;

  // --- Time ---
  data->iTOW = getU4(p);
  p += 4;
  data->year = getU2(p);
  p += 2;
  data->month = *p++;
  data->day = *p++;
  data->hour = *p++;
  data->min = *p++;
  data->sec = *p++;

  // --- Validity ---
  uint8_t validByte = *p++;
  data->valid.validDate = (validByte & 0x01);
  data->valid.validTime = (validByte & 0x02);
  data->valid.fullyResolved = (validByte & 0x04);
  data->valid.validMag = (validByte & 0x08);

  data->tAcc = getU4(p);
  p += 4;
  data->nano = getI4(p);
  p += 4;

  data->fixType = static_cast<GpsFixType>(*p++);

  // --- Flags ---
  uint8_t flags = *p++;
  data->flags.gnssFixOK = (flags & 0x01);
  data->flags.diffSoln = (flags & 0x02);
  data->flags.psmState = static_cast<GpsPowerSaveMode>((flags >> 2) & 0x07);
  data->flags.headVehValid = (flags & 0x20);
  data->flags.carrSoln =
      static_cast<GpsCarrierPhaseStatus>((flags >> 6) & 0x03);

  p++; // Skip flags2

  data->numSV = *p++;

  // --- Position ---
  data->lon = getI4(p);
  p += 4;
  data->lat = getI4(p);
  p += 4;
  data->height = getI4(p);
  p += 4;
  data->hMSL = getI4(p);
  p += 4;
  data->hAcc = getU4(p);
  p += 4;
  data->vAcc = getU4(p);
  p += 4;

  // --- Velocity ---
  data->velN = getI4(p);
  p += 4;
  data->velE = getI4(p);
  p += 4;
  data->velD = getI4(p);
  p += 4;
  data->gSpeed = getI4(p);
  p += 4;
  data->headMot = getI4(p);
  p += 4;
  data->sAcc = getU4(p);
  p += 4;
  data->headAcc = getU4(p);
  p += 4;

  // --- DOP ---
  data->pDOP = getU2(p);
  p += 2;
}


void UbxGpsInterface::parseBasicFix(const uint8_t *payload,
                                    GpsBasicFixData *data) {
  GpsPvtData pvt{};
  parsePvt(payload, &pvt);

#ifdef DEBUG_MODE
  printPayload(payload, 92);
#endif
  data->iTOW = pvt.iTOW;
  data->year = pvt.year;
  data->month = pvt.month;
  data->day = pvt.day;
  data->hour = pvt.hour;
  data->min = pvt.min;
  data->sec = pvt.sec;
  data->nano = pvt.nano;

  data->valid = pvt.valid;
  data->fixType = pvt.fixType;
  data->flags = pvt.flags;
  data->numSV = pvt.numSV;

  data->lon = pvt.lon;
  data->lat = pvt.lat;
  data->height = pvt.height;
  data->hMSL = pvt.hMSL;
  data->hAcc = pvt.hAcc;
  data->vAcc = pvt.vAcc;

  data->velN = pvt.velN;
  data->velE = pvt.velE;
  data->velD = pvt.velD;
  data->gSpeed = pvt.gSpeed;
  data->headMot = pvt.headMot;
  data->sAcc = pvt.sAcc;
  data->headAcc = pvt.headAcc;

  data->pDOP = pvt.pDOP;
}
