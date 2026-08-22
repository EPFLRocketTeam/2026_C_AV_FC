#include "Core/Inc/main.h"
#include "Application/app_timebase.h"
#include "Modules/rx_radio_module.hpp"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Common.h"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"
#include "Application/FlightControl/fc_commands.hpp"
#include "Application/Data/data.hpp"

extern "C" {
#include "Application/main.h"
}

// Ball-valve commands carry only ACTIVE/INACTIVE over the air, so the
// uplink can only ever fully open or fully close them -- no intermediate
// angle. Use the shell's own units (percent open).
static constexpr float kBallValveOpen  = 100.0f;
static constexpr float kBallValveClose = 0.0f;

void handleRxCommand(void* data) noexcept {
	av_uplink_t* packet = (av_uplink_t*)data;
	if (packet == nullptr) {
		return;
	}
	flight_computer::GOATStore& g = flight_computer::GOATStore::get_instance();
	g.uplinkCmdStore.set_id(packet->order_id);
	g.uplinkCmdStore.set_value(packet->order_value);

	// order_value is specified as strictly ACTIVE or INACTIVE. Anything else
	// is a corrupt byte that survived CRC, so drop the command rather than
	// letting it fall through to the "close" case and move a valve.
	if (packet->order_value != 1 && packet->order_value != 0) {
		printf("[RADIO] bad order_value 0x%02X for id %u, ignored\r\n",
				packet->order_value, packet->order_id);
		return;
	}
	const bool active = (packet->order_value == 1);

	switch (packet->order_id) {
	case AV_CMD_CALIBRATE:
		fc_commands::OnAvCalibrate(nullptr);
		break;
	case AV_CMD_RECOVER:
		fc_commands::OnAvRecover(nullptr);
		break;
	case AV_CMD_ARM:
		fc_commands::OnAvArm(nullptr);
		break;
	case AV_CMD_PRESSURIZE:
		fc_commands::OnPressurize(nullptr, true);
		break;
	case AV_CMD_LAUNCH:
		fc_commands::OnAvLaunch(nullptr);
		break;
	case AV_CMD_ABORT:
		fc_commands::OnAvAbort(nullptr);
		break;



	// Tank pressurization (PO/PE).
	case AV_CMD_DPR_LOX:
		fc_commands::OnDprLoxPressurize(nullptr, active);
		break;
	case AV_CMD_DPR_FUEL:
		fc_commands::OnDprEthPressurize(nullptr, active);
		break;

	// SPO/SPE ball valves. The protocol dropped DPR_CONFIG and is now fixed
	// to ball valves, so these are always present alongside AV_CMD_DPR_*.
	case AV_CMD_SDPR_LOX:
		fc_commands::OnBallLox(nullptr, active ? kBallValveOpen : kBallValveClose);
		break;
	case AV_CMD_SDPR_FUEL:
		fc_commands::OnBallFuel(nullptr, active ? kBallValveOpen : kBallValveClose);
		break;

	// MO/ME on the Engine board, bypassing the FSM.
	case AV_CMD_MAIN_LOX:
		fc_commands::OnMainLox(nullptr, active);
		break;
	case AV_CMD_MAIN_FUEL:
		fc_commands::OnMainFuel(nullptr, active);
		break;

	// VO/VE. ACTIVE means "open the vent" here, matching the shell's
	// "vent lox open" -- not the valves_state bit, which is 0 for open.
	case AV_CMD_VENT_LOX:
		fc_commands::OnVentLox(nullptr, active);
		break;
	case AV_CMD_VENT_FUEL:
		fc_commands::OnVentFuel(nullptr, active);
		break;

	default:
		printf("[RADIO] unhandled order_id %u\r\n", packet->order_id);
		break;
	}
}

static void onPacketReceived(uint8_t packetId, uint8_t *payload, uint32_t length) {
	printf("Decoded packet: id=0x%02X len=%lu payload=[%s]\r\n", packetId,
			(unsigned long) length, (char*) payload);


	if (packetId == GSC_CMD && length == av_uplink_size) {
		handleRxCommand((av_uplink_t*)payload);
	} else {
		printf("Decode Error: Wrong packet id");
	}
}


extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi1;

// The module stores raw pointers to its driver and buffer and is neither
// copyable nor movable, so everything it points at must outlive
// simple_radio_init() -- hence file scope, not locals.
static SX127X_hw_t SX127X_TX_hw;
static SX127X_hw_t SX127X_RX_hw;

static SX127XCapsule tx(&SX127X_TX_hw, nullptr);
static SX127XCapsule rx(&SX127X_RX_hw, onPacketReceived);

static RingBuffer<av_uplink_t, 10> rx_buffer;
static SX127XCapsule *rxArr[1] = {&rx};
static RingBuffer<av_uplink_t, 10> *rxRing[1] = {&rx_buffer};

RxRadioModule rx_module(rxArr, rxRing);

void simple_radio_init(void) {
	SX127X_RX_hw.dio0.port = GPIO_RFM_RX_INT0_GPIO_Port;
	SX127X_RX_hw.dio0.pin = GPIO_RFM_RX_INT0_Pin;
	SX127X_RX_hw.nss.port = SPI_RFM_RX_CS_GPIO_Port;
	SX127X_RX_hw.nss.pin = SPI_RFM_RX_CS_Pin;
	SX127X_RX_hw.reset.port = GPIO_RFM_RX_RST_GPIO_Port;
	SX127X_RX_hw.reset.pin = GPIO_RFM_RX_RST_Pin;
	SX127X_RX_hw.spi = &hspi2;

	SX127X_TX_hw.dio0.port = GPIO_RFM_TX_INT0_GPIO_Port;
	SX127X_TX_hw.dio0.pin = GPIO_RFM_TX_INT0_Pin;
	SX127X_TX_hw.nss.port = SPI_RFM_TX_CS_GPIO_Port;
	SX127X_TX_hw.nss.pin = SPI_RFM_TX_CS_Pin;
	SX127X_TX_hw.reset.port = GPIO_RFM_TX_RST_GPIO_Port;
	SX127X_TX_hw.reset.pin = GPIO_RFM_TX_RST_Pin;
	SX127X_TX_hw.spi = &hspi1;

	SX127X_hw_init(&SX127X_TX_hw);
	SX127X_hw_Reset(&SX127X_TX_hw);

	SX127X_hw_init(&SX127X_RX_hw);
	SX127X_hw_Reset(&SX127X_RX_hw);

	rx_module.init();
}

void simple_radio_tick(void) {
	rx_module.update(0);
}
