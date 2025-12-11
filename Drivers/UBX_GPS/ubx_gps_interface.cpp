#include "ubx_gps_interface.h"
#include <cstring>
#include <cstdlib>


// ============================================================================
// CONSTRUCTOR
// ============================================================================

UbxGpsInterface::UbxGpsInterface(UART_HandleTypeDef *huart,
                                 uint16_t rate_ms)
    : uart_handle_(huart), rate_ms_(rate_ms) {}

// ============================================================================
// PUBLIC METHODS
// ============================================================================

GpsStatus UbxGpsInterface::init() {
  GpsStatus status;

  // 1. Disable NMEA on UART1
  status = writeCfgVal8(CFG_KEY_UART1OUTPROT_NMEA, 0);
  if (status != GpsStatus::OK)
    return status;

  // 2. Enable UBX-NAV-PVT on UART1
  status = writeCfgVal8(CFG_KEY_MSGOUT_NAV_PVT_UART1, 1);
  if (status != GpsStatus::OK)
    return status;

  // 3. Set configured rate
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

GpsStatus UbxGpsInterface::getPvt(GpsBasicFixData *pvt_data,
                                  uint32_t timeout_ms) {
  uint32_t start_tick = HAL_GetTick();
  uint8_t rx_byte;
  ParserState step = STATE_SYNC_1;

  // Buffers
  uint8_t payload_buf[UBX_NAV_PVT_PAYLOAD_LEN];
  uint16_t payload_idx = 0;
  uint8_t ck_a_calc = 0, ck_b_calc = 0;

  while ((HAL_GetTick() - start_tick) < timeout_ms) {

    // Read 1 byte at a time
    if (HAL_UART_Receive(uart_handle_, &rx_byte, 1, GPS_RX_POLL_TIMEOUT) ==
        HAL_OK) {

      switch (step) {
      case STATE_SYNC_1:
        if (rx_byte == UBX_SYNC_CHAR_1)
          step = STATE_SYNC_2;
        break;

      case STATE_SYNC_2:
        if (rx_byte == UBX_SYNC_CHAR_2)
          step = STATE_CLASS;
        else
          step = STATE_SYNC_1;
        break;

      case STATE_CLASS:
        if (rx_byte == UBX_CLASS_NAV) {
          step = STATE_ID;
          ck_a_calc = rx_byte;
          ck_b_calc = rx_byte;
        } else {
          step = STATE_SYNC_1;
        }
        break;

      case STATE_ID:
        if (rx_byte == UBX_ID_NAV_PVT) {
          step = STATE_LEN_LSB;
          ck_a_calc += rx_byte;
          ck_b_calc += ck_a_calc;
        } else {
          step = STATE_SYNC_1;
        }
        break;

      case STATE_LEN_LSB:
        if (rx_byte == (uint8_t)(UBX_NAV_PVT_PAYLOAD_LEN & 0xFF)) {
          step = STATE_LEN_MSB;
          ck_a_calc += rx_byte;
          ck_b_calc += ck_a_calc;
        } else {
          step = STATE_SYNC_1;
        }
        break;

      case STATE_LEN_MSB:
        if (rx_byte == (uint8_t)(UBX_NAV_PVT_PAYLOAD_LEN >> 8)) {
          step = STATE_PAYLOAD;
          ck_a_calc += rx_byte;
          ck_b_calc += ck_a_calc;
          payload_idx = 0;
        } else {
          step = STATE_SYNC_1;
        }
        break;

      case STATE_PAYLOAD:
        payload_buf[payload_idx++] = rx_byte;
        ck_a_calc += rx_byte;
        ck_b_calc += ck_a_calc;
        if (payload_idx == UBX_NAV_PVT_PAYLOAD_LEN) {
          step = STATE_CK_A;
        }
        break;

      case STATE_CK_A:
        if (rx_byte == ck_a_calc) {
          step = STATE_CK_B;
        } else {
          step = STATE_SYNC_1;
        }
        break;

      case STATE_CK_B:
        if (rx_byte == ck_b_calc) {
          // --- SUCCESS ---
          parseBasicFix(payload_buf, pvt_data);
          return GpsStatus::OK;
        }
        step = STATE_SYNC_1;
        break;

      default:
        step = STATE_SYNC_1;
        break;
      }
    }
  }

  return GpsStatus::ERROR_TIMEOUT;
}

GpsStatus UbxGpsInterface::stop() {
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

// ============================================================================
// PRIVATE HELPER METHODS
// ============================================================================

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
  const uint8_t *p = payload;

  // Skip to validity flags (offset 11)
  p += 11;

  // --- Validity ---
  uint8_t validByte = *p++;
  data->valid.validDate = (validByte & 0x01);
  data->valid.validTime = (validByte & 0x02);
  data->valid.fullyResolved = (validByte & 0x04);
  data->valid.validMag = (validByte & 0x08);

  // Skip tAcc (4 bytes) and nano (4 bytes)
  p += 8;

  // --- Fix Info ---
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

  // --- numSV ---
  data->numSV = *p++;

  // --- Position ---
  data->lon = getI4(p);
  p += 4;
  data->lat = getI4(p);
  p += 4;

  // Skip height and hMSL
  p += 8;

  // hAcc
  data->hAcc = getU4(p);
  p += 4;

}
