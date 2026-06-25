#include "SX1262.h"

#include <cmath>
#include <cstring>
#include <ranges>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_timer.h>
#include <esp_log.h>

namespace YOBA {
	SX1262Error SX1262::setup(
		const gpio_num_t SSPin,
		const gpio_num_t busyPin,
		const gpio_num_t DIO1Pin,
		const gpio_num_t RSTPin,

		const spi_host_device_t SPIHostDevice,
		const uint32_t SPIFrequencyHz,

		const uint32_t frequencyHz,
		const SX1262LoRaBandwidth bandwidth,
		const uint8_t spreadingFactor,
		const SX1262LoRaCodingRate codingRate,
		const uint8_t syncWord,
		const uint16_t preambleLength,

		const uint8_t currentLimitMA,
		const int8_t powerDBm,
		const bool useLDORegulator
	) {
		_SSPin = SSPin;
		_busyPin = busyPin;
		_DIO1Pin = DIO1Pin;
		_RSTPin = RSTPin;

		// -------------------------------- GPIO output --------------------------------

		gpio_config_t g {};

		// SS
		g.pin_bit_mask = 1ULL << _SSPin;

		// RST
		if (_RSTPin != GPIO_NUM_NC)
			g.pin_bit_mask |= 1ULL << _RSTPin;

		g.mode = GPIO_MODE_OUTPUT;
		g.pull_up_en = GPIO_PULLUP_DISABLE;
		g.pull_down_en = GPIO_PULLDOWN_DISABLE;
		g.intr_type = GPIO_INTR_DISABLE;
		gpio_config(&g);

		// Setting SS to high just in case
		setSSPinLevel(true);

		// -------------------------------- GPIO input --------------------------------

		// Busy
		g = {};
		g.pin_bit_mask = 1ULL << static_cast<uint16_t>(_busyPin);
		g.mode = GPIO_MODE_INPUT;
		g.pull_up_en = GPIO_PULLUP_ENABLE;
		g.pull_down_en = GPIO_PULLDOWN_DISABLE;
		g.intr_type = GPIO_INTR_NEGEDGE;
		gpio_config(&g);

		// DIO1
		g = {};
		g.pin_bit_mask = 1ULL << static_cast<uint16_t>(_DIO1Pin);
		g.mode = GPIO_MODE_INPUT;
		g.pull_up_en = GPIO_PULLUP_ENABLE;
		g.pull_down_en = GPIO_PULLDOWN_DISABLE;
		g.intr_type = GPIO_INTR_POSEDGE;
		gpio_config(&g);

		// -------------------------------- Interrupts --------------------------------

		gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

		// Busy
		_busyPinSemaphore = xSemaphoreCreateBinary();
		gpio_isr_handler_add(_busyPin, onBusyPinInterrupt,this);

		// DIO1
		_DIO1PinSemaphore = xSemaphoreCreateBinary();
		gpio_isr_handler_add(_DIO1Pin, onDIO1PinInterrupt, this);

		// -------------------------------- SPI --------------------------------

		spi_device_interface_config_t SPIInterfaceConfig {};
		// CPOL = 0, CPHA = 0
		SPIInterfaceConfig.mode = 0;
		SPIInterfaceConfig.clock_speed_hz = SPIFrequencyHz;
		SPIInterfaceConfig.spics_io_num = _SSPin;
		SPIInterfaceConfig.queue_size = 1;

		const auto ESPError = spi_bus_add_device(SPIHostDevice, &SPIInterfaceConfig, &_SPIDevice);

		if (!checkESPError(ESPError))
			return SX1262Error::SPITransaction;

		// -------------------------------- Initialization sequence --------------------------------

		auto error = reset();
		if (error != SX1262Error::none)
			return error;

		error = validateChip();
		if (error != SX1262Error::none)
			return error;

		// TCXO configuration should be here
		//				error = XTAL && tcxoVoltage > 0.0f) {
		//					setTCXO(tcxoVoltage);
		//				}

		error = setBufferBaseAddress(0x00, 0x00);
		if (error != SX1262Error::none)
			return error;

		error = setPacketType(PACKET_TYPE_LORA);
		if (error != SX1262Error::none)
			return error;

		error = setRXTXFallbackMode(RX_TX_FALLBACK_MODE_FS);
		if (error != SX1262Error::none)
			return error;

		error = clearIRQStatus();
		if (error != SX1262Error::none)
			return error;

		error = setDIOIRQParams();
		if (error != SX1262Error::none)
			return error;

		error = calibrate(CALIBRATE_ALL);
		if (error != SX1262Error::none)
			return error;

		// Wait for calibration completion end. Normally this should take 3.5 ms
		error = waitForBusyPin(1'000);
		if (error != SX1262Error::none)
			return error;

		error = setRegulatorMode(useLDORegulator ? REGULATOR_LDO : REGULATOR_DC_DC);
		if (error != SX1262Error::none)
			return error;

		error = setCurrentLimit(currentLimitMA);
		if (error != SX1262Error::none)
			return error;

		error = setDio2AsRfSwitch(true);
		if (error != SX1262Error::none)
			return error;

		error = setLoRaSyncWord(syncWord);
		if (error != SX1262Error::none)
			return error;

		error = setLoRaPreambleLength(preambleLength);
		if (error != SX1262Error::none)
			return error;

		error = setLoRaCRC(true);
		if (error != SX1262Error::none)
			return error;

		error = invertLoRaIQ(false);
		if (error != SX1262Error::none)
			return error;

		error = setLoRaModulationParams(
			spreadingFactor,
			bandwidth,
			codingRate,
			false
		);

		if (error != SX1262Error::none)
			return error;

		error = setLoRaCADParams();
		if (error != SX1262Error::none)
			return error;

		error = setRFFrequency(frequencyHz);
		if (error != SX1262Error::none)
			return error;

		error = setTXClampConfig(true);
		if (error != SX1262Error::none)
			return error;

		error = setOutputPower(powerDBm);
		if (error != SX1262Error::none)
			return error;

		return SX1262Error::none;
	}

	SX1262Error SX1262::reset() {
		// Toggling RST GPIO
		if (_RSTPin != GPIO_NUM_NC) {
			setRSTPinLevel(false);
			delayMs(10);
			setRSTPinLevel(true);
			delayMs(10);
		}

		// Trying to set mode to standby - SX126x often refuses first few commands after reset
		const auto start = esp_timer_get_time();

		while (true) {
			if (setStandby() == SX1262Error::none)
				return SX1262Error::none;

			// standby command failed, check timeout and try again
			if (esp_timer_get_time() - start >= 1'000'000) {
				// timed out, possibly incorrect wiring
				break;
			}

			// wait a bit to not spam the module
			delayMs(10);
		}

		return SX1262Error::timeout;
	}

	void SX1262::setSPIMutex(const SemaphoreHandle_t mutex) {
		_SPIMutex = mutex;
	}

	SX1262Error SX1262::validateChip() {
		for (uint8_t i = 0; i < 10; ++i) {
			uint8_t buffer[16] {};
			SPIReadRegister(REG_VERSION_STRING, { buffer, 16 });

			if (strncmp(VERSION_STRING, reinterpret_cast<char*>(buffer), 6) == 0)
				return SX1262Error::none;

			ESP_LOGE(_logTag, "failed to validate chip: version mismatch, attempt is %d, value is %s", i, buffer);

			delayMs(10);
		}

		ESP_LOGE(_logTag, "failed to validate chip: maximum attempts exceeded");

		return SX1262Error::invalidChip;
	}

	SX1262Error SX1262::calibrateImage(const uint32_t frequencyHz) {
		uint8_t data[3] {
			CMD_CALIBRATE_IMAGE,
			0,
			0
		};

		// try to match the frequency ranges
		if (frequencyHz >= 902'000'000) {
			data[1] = CAL_IMG_902_MHZ_1;
			data[2] = CAL_IMG_902_MHZ_2;
		}
		else if (frequencyHz >= 863'000'000 && frequencyHz <= 870'000'000) {
			data[1] = CAL_IMG_863_MHZ_1;
			data[2] = CAL_IMG_863_MHZ_2;
		}
		else if (frequencyHz >= 779'000'000 && frequencyHz <= 787'000'000) {
			data[1] = CAL_IMG_779_MHZ_1;
			data[2] = CAL_IMG_779_MHZ_2;
		}
		else if (frequencyHz >= 470'000'000 && frequencyHz <= 510'000'000) {
			data[1] = CAL_IMG_470_MHZ_1;
			data[2] = CAL_IMG_470_MHZ_2;
		}
		else {
			data[1] = CAL_IMG_430_MHZ_1;
			data[2] = CAL_IMG_430_MHZ_2;
		}

		return SPIWrite(data, 3);
	}

	SX1262Error SX1262::setRFFrequency(const uint32_t frequencyHz) {
		if (frequencyHz < 120'000'000 || frequencyHz > 960'000'000) {
			ESP_LOGE(_logTag, "failed to set frequency: value %d is out of range [120; 960]");

			return SX1262Error::invalidArgument;
		}

		// Check if we need to recalibrate image
		if (std::abs(static_cast<int64_t>(frequencyHz) - static_cast<int64_t>(_frequencyHz)) >= static_cast<int64_t>(CAL_IMG_FREQ_TRIG_HZ)) {
			const auto error = calibrateImage(frequencyHz);

			if (error != SX1262Error::none)
				return error;
		}

		_frequencyHz = frequencyHz;

		// From SX1262 datasheet:
		// frequencyHz = regValue * crystalFreqHz / divider
		//
		// Explanations:
		// frequencyHz = desired radio frequency, 868'000'000 Hz for example
		// regValue = 31-bit register value
		// crystalFreqHz = 32'000'000 Hz
		// divider = 2 ^ 25
		//
		// So:
		// regValue = frequencyHz / (crystalFreqHz / divider)
		// regValue = frequencyHz * divider / crystalFreqHz
		// regValue = frequencyHz * 2^25 / 32'000'000 Hz

		const auto regValue = static_cast<uint32_t>(static_cast<uint64_t>(frequencyHz) * static_cast<uint64_t>(RF_DIVIDER) / static_cast<uint64_t>(RF_XTAL_FREQUENCY_HZ));

		const uint8_t data[] {
			CMD_SET_RF_FREQUENCY,
			static_cast<uint8_t>((regValue >> 24) & 0xFF),
			static_cast<uint8_t>((regValue >> 16) & 0xFF),
			static_cast<uint8_t>((regValue >> 8) & 0xFF),
			static_cast<uint8_t>(regValue & 0xFF)
		};

		return SPIWrite(data, 5);
	}

	SX1262Error SX1262::setStandby(const uint8_t value) {
		return SPIWriteCommandAndUint8(CMD_SET_STANDBY, value);
	}

	SX1262Error SX1262::setSymbNumTimeout(const uint8_t value) {
		return SPIWriteCommandAndUint8(CMD_SET_LORA_SYMB_NUM_TIMEOUT, value);
	}

	SX1262Error SX1262::setRX(const uint32_t timeoutUs) {
		return setRXOrTX(CMD_SET_RX, timeoutUs);
	}

	SX1262Error SX1262::setTX(const uint32_t timeoutUs) {
		return setRXOrTX(CMD_SET_TX, timeoutUs);
	}

	SX1262Error SX1262::setRXTXFallbackMode(const uint8_t value) {
		return SPIWriteCommandAndUint8(CMD_SET_RX_TX_FALLBACK_MODE, value);
	}

	SX1262Error SX1262::setLoRaCADParams() {
		const uint8_t data[] {
			CMD_SET_CAD_PARAMS,
			CAD_ON_8_SYMB,
			static_cast<uint8_t>(_LoRaSpreadingFactor + 13),
			CAD_PARAM_DET_MIN,
			CAD_GOTO_STDBY,
			0x00,
			0x00,
			0x00
		};

		return SPIWrite(data, 8);
	}

	SX1262Error SX1262::setBufferBaseAddress(const uint8_t rxAddress, const uint8_t txAddress) {
		const uint8_t data[] {
			CMD_SET_BUFFER_BASE_ADDRESS,
			rxAddress,
			txAddress
		};

		return SPIWrite(data, 3);
	}

	SX1262Error SX1262::getStatus(uint8_t& status) {
		return SPIReadCommand(CMD_GET_STATUS, { &status, 1 });
	}

	SX1262Error SX1262::getPacketType(uint8_t& packetType) {
		return SPIReadCommand(CMD_GET_PACKET_TYPE, { &packetType, 1 });
	}

	SX1262Error SX1262::setPacketType(const uint8_t packetType) {
		return SPIWriteCommandAndUint8(CMD_SET_PACKET_TYPE, packetType);
	}

	SX1262Error SX1262::setRegulatorMode(const uint8_t mode) {
		return SPIWriteCommandAndUint8(CMD_SET_REGULATOR_MODE, mode);
	}

	SX1262Error SX1262::getIRQStatus(uint16_t& status) {
		status = 0;

		const auto error = SPIReadCommand(CMD_GET_IRQ_STATUS, { reinterpret_cast<uint8_t*>(&status), 2 });

		if (error != SX1262Error::none) {
			ESP_LOGE(_logTag, "failed to get IRQ status");

			return error;
		}

		status = ((status & 0xFF) << 8) | ((status >> 8) & 0xFF);

		return SX1262Error::none;
	}

	SX1262Error SX1262::clearIRQStatus(const uint16_t status) {
		const uint8_t data[] {
			CMD_CLEAR_IRQ_STATUS,
			static_cast<uint8_t>((status >> 8) & 0xFF),
			static_cast<uint8_t>(status & 0xFF)
		};

		return SPIWrite(data, 3);
	}

	SX1262Error SX1262::setDIOIRQParams(const uint16_t irqMask, const uint16_t dio1Mask, const uint16_t dio2Mask, const uint16_t dio3Mask) {
		const uint8_t data[] {
			CMD_SET_DIO_IRQ_PARAMS,

			static_cast<uint8_t>((irqMask >> 8) & 0xFF),
			static_cast<uint8_t>(irqMask & 0xFF),

			static_cast<uint8_t>((dio1Mask >> 8) & 0xFF),
			static_cast<uint8_t>(dio1Mask & 0xFF),

			static_cast<uint8_t>((dio2Mask >> 8) & 0xFF),
			static_cast<uint8_t>(dio2Mask & 0xFF),

			static_cast<uint8_t>((dio3Mask >> 8) & 0xFF),
			static_cast<uint8_t>(dio3Mask & 0xFF)
		};

		return SPIWrite(data, 9);
	}

	SX1262Error SX1262::calibrate(const uint8_t value) {
		return SPIWriteCommandAndUint8(CMD_CALIBRATE, value);
	}

	SX1262Error SX1262::setLoRaSyncWord(const uint8_t syncWord, const uint8_t controlBits) {
		const uint8_t data[] {
			static_cast<uint8_t>((syncWord & 0xF0) | ((controlBits & 0xF0) >> 4)),
			static_cast<uint8_t>(((syncWord & 0x0F) << 4) | (controlBits & 0x0F))
		};

		return SPIWriteRegister(REG_LORA_SYNC_WORD_MSB, { data, 2 });
	}

	SX1262Error SX1262::setCurrentLimit(const float currentLimit) {
		// check allowed range
		if (currentLimit < 0 || currentLimit > 140) {
			ESP_LOGE(_logTag, "failed to set current limit: value %f is out of range [0; 140]", currentLimit);
			return SX1262Error::invalidArgument;
		}

		// calculate raw value
		const auto rawLimit = static_cast<uint8_t>(currentLimit / 2.5f);

		return SPIWriteRegister(REG_OCP_CONFIGURATION, { &rawLimit, 1 });
	}

	SX1262Error SX1262::setDio2AsRfSwitch(const bool enable) {
		return SPIWriteCommandAndUint8(CMD_SET_DIO2_AS_RF_SWITCH_CTRL, enable ? DIO2_AS_RF_SWITCH : DIO2_AS_IRQ);
	}

	SX1262Error SX1262::setLoRaModulationParams(const uint8_t spreadingFactor, const SX1262LoRaBandwidth bandwidth, const SX1262LoRaCodingRate codingRate, const uint8_t LDROptimize) {
		if (spreadingFactor < 5 || spreadingFactor > 12) {
			ESP_LOGE(_logTag, "failed to set modulation params: spreading factor %d is out of range [5; 12]", spreadingFactor);
			return SX1262Error::invalidArgument;
		}

		_LoRaSpreadingFactor = spreadingFactor;

		// Bandwidth
		_LoRaBandwidth = bandwidth;
		uint8_t bwRegVal = 0;

		switch (_LoRaBandwidth) {
			case SX1262LoRaBandwidth::bw7_81: bwRegVal = 0x00; break;
			case SX1262LoRaBandwidth::bw10_42: bwRegVal = 0x08; break;
			case SX1262LoRaBandwidth::bw15_63: bwRegVal = 0x01; break;
			case SX1262LoRaBandwidth::bw20_83: bwRegVal = 0x09; break;
			case SX1262LoRaBandwidth::bw31_25: bwRegVal = 0x02; break;
			case SX1262LoRaBandwidth::bw41_67: bwRegVal = 0x0A; break;
			case SX1262LoRaBandwidth::bw62_5: bwRegVal = 0x03; break;
			case SX1262LoRaBandwidth::bw125_0: bwRegVal = 0x04; break;
			case SX1262LoRaBandwidth::bw250_0: bwRegVal = 0x05; break;
			case SX1262LoRaBandwidth::bw500_0: bwRegVal = 0x06; break;
		}

		// Coding rate
		_LoRaCodingRate = codingRate;
		uint8_t crRegVal = 0;

		switch (_LoRaCodingRate) {
			case SX1262LoRaCodingRate::cr4_5: crRegVal = 0x01; break;
			case SX1262LoRaCodingRate::cr4_6: crRegVal = 0x02; break;
			case SX1262LoRaCodingRate::cr4_7: crRegVal = 0x03; break;
			case SX1262LoRaCodingRate::cr4_8: crRegVal = 0x04; break;
			case SX1262LoRaCodingRate::cr4_5LongInterleaver: crRegVal = 0x05; break;
			case SX1262LoRaCodingRate::cr4_6LongInterleaver: crRegVal = 0x06; break;
			case SX1262LoRaCodingRate::cr4_8LongInterleaver: crRegVal = 0x07; break;
		}

		const uint8_t data[5] {
			CMD_SET_MODULATION_PARAMS,
			_LoRaSpreadingFactor,
			bwRegVal,
			crRegVal,
			LDROptimize
		};

		return SPIWrite(data, 5);
	}

	SX1262Error SX1262::setTXClampConfig(const bool enable) {
		// fixes overly eager PA clamping
		// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.2 for details

		uint8_t clampConfig = 0;

		const auto error = SPIReadRegister(REG_TX_CLAMP_CONFIG, { &clampConfig, 1 });

		if (error != SX1262Error::none)
			return error;

		// apply or undo workaround
		if (enable) {
			clampConfig |= 0x1E;
		}
		else {
			clampConfig = (clampConfig & ~0x1E) | 0x08;
		}

		return SPIWriteRegister(REG_TX_CLAMP_CONFIG, { &clampConfig, 1 });
	}

	SX1262Error SX1262::setOutputPower(int8_t powerDBm) {
		// get current OCP configuration
		uint8_t ocp = 0;

		auto error = SPIReadRegister(REG_OCP_CONFIGURATION, { &ocp, 1 });

		if (error != SX1262Error::none) {
			ESP_LOGE(_logTag, "set output power failed: unable to receive OCP configuration");
			return error;
		}

		// set PA config
		error = setPAConfig();

		if (error != SX1262Error::none) {
			ESP_LOGE(_logTag, "set output power failed: unable to set PA config");
			return error;
		}

		// set output power with default 200us ramp
		error = setTXParams(powerDBm, PA_RAMP_200U);

		if (error != SX1262Error::none) {
			ESP_LOGE(_logTag, "set output power failed: unable to set TX params");
			return error;
		}

		// restore OCP configuration
		return SPIWriteRegister(REG_OCP_CONFIGURATION, { &ocp, 1 });
	}

	SX1262Error SX1262::setPacketParams(const uint16_t preambleLength, const uint8_t headerType, const uint8_t length, const uint8_t crcType, const uint8_t invertIQ) {
		const auto error = fixInvertedIQ(invertIQ);

		if (error != SX1262Error::none)
			return error;

		const uint8_t data[7] {
			CMD_SET_PACKET_PARAMS,
			static_cast<uint8_t>((preambleLength >> 8) & 0xFF),
			static_cast<uint8_t>(preambleLength & 0xFF),
			headerType,
			length,
			crcType,
			invertIQ
		};

		return SPIWrite(data, 7);
	}

	SX1262Error SX1262::writeBuffer(const std::span<const uint8_t> data, const uint8_t offset) {
		_SPIBuffer[0] = CMD_WRITE_BUFFER;
		_SPIBuffer[1] = offset;

		std::memcpy(_SPIBuffer + 2, data.data(), data.size());

		return SPIWriteBuffer(2 + data.size());
	}

	SX1262Error SX1262::readBuffer(const std::span<uint8_t> data, const uint8_t offset) {
		if (waitForBusyPin() == SX1262Error::timeout)
			return SX1262Error::timeout;

		_SPIBuffer[0] = CMD_READ_BUFFER; // W: command | R: status
		_SPIBuffer[1] = offset;          // W: offset  | R: status
		_SPIBuffer[2] = 0x00;            // W: -       | R: status
		//        [3]                    // W: -       | R: result

		spi_transaction_t t {};
		t.tx_buffer = _SPIBuffer;
		t.rx_buffer = _SPIBuffer;
		t.length = 8 * (3 + data.size());

		const auto state = SPITransmit(&t);

		if (state) {
			std::memcpy(data.data(), _SPIBuffer + 3, data.size());

			//
			//				for (int i = 0; i < 3 + length; ++i) {
			//					ESP_LOGI(_logTag, "readBuffer buffer[%d]: %d", i, _SPIBuffer[i]);
			//				}
			return SX1262Error::none;
		}

		return SX1262Error::SPITransaction;
	}

	SX1262Error SX1262::getPacketStatus(uint32_t& status) {
		uint8_t data[3] {0, 0, 0};

		const auto error = SPIReadCommand(CMD_GET_PACKET_STATUS, { data, 3 });

		if (error != SX1262Error::none)
			return error;

		status = ((static_cast<uint32_t>(data[0]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[2]));

		return SX1262Error::none;
	}

	SX1262Error SX1262::getRSSI(float& rssi) {
		uint32_t packetStatus = 0;

		const auto error = getPacketStatus(packetStatus);

		if (error != SX1262Error::none)
			return error;

		rssi = getRSSIFromPacketStatus(packetStatus);

		return SX1262Error::none;
	}

	SX1262Error SX1262::getRSSIInst(float& rssi) {
		uint8_t rssiRaw = 0;

		const auto error = SPIReadCommand(CMD_GET_RSSI_INST, { &rssiRaw, 1 });

		if (error != SX1262Error::none)
			return error;

		rssi = static_cast<float>(rssiRaw) / -2.0f;

		return SX1262Error::none;
	}

	SX1262Error SX1262::getSNR(float& snr) {
		uint32_t packetStatus = 0;

		const auto error = getPacketStatus(packetStatus);

		if (error != SX1262Error::none)
			return error;

		snr = getSNRFromPacketStatus(packetStatus);

		return SX1262Error::none;
	}

	SX1262Error SX1262::setLoRaPreambleLength(const uint16_t preambleLength) {
		_preambleLength = preambleLength;

		return updatePacketParams();
	}

	SX1262Error SX1262::setLoRaCRC(const bool enabled) {
		_crcType = enabled ? LORA_CRC_ON : LORA_CRC_OFF;

		return updatePacketParams();
	}

	SX1262Error SX1262::invertLoRaIQ(const bool enable) {
		if (enable) {
			_invertIQ = LORA_IQ_INVERTED;
		}
		else {
			_invertIQ = LORA_IQ_STANDARD;
		}

		return updatePacketParams();
	}

	SX1262Error SX1262::waitForDIO1Semaphore(const uint32_t timeoutUs) const {
		return
			// Already in high
			getDIO1PinLevel()
			? SX1262Error::none
			// Wait for high
			: (
				xSemaphoreTake(
					_DIO1PinSemaphore,
					timeoutUs == 0
					// Zero timeout means infinite waiting
					? portMAX_DELAY
					// FreeRTOS tasks can handle at least portTICK_PERIOD_MS, also adding 100 ms for пропёрживание
					: pdMS_TO_TICKS(std::max(timeoutUs / 1000 + 100, portTICK_PERIOD_MS))
				) == pdTRUE
				? SX1262Error::none
				: SX1262Error::timeout
			);
	}

	SX1262Error SX1262::transmit(const std::span<const uint8_t> data, uint32_t timeoutUs) {
		// Check packet length
		if (_LoRaCodingRate > SX1262LoRaCodingRate::cr4_8) {
			// Long interleaver needs at least 8 bytes
			if (data.size() < 8) {
				ESP_LOGE(_logTag, "failed to transmit: packet is too short for long interleaver coding rate");
				return SX1262Error::invalidArgument;
			}

			// Long interleaver supports up to 253 bytes if CRC is enabled
			if (_crcType == LORA_CRC_ON && data.size() > MAX_PACKET_LENGTH - 2) {
				ESP_LOGE(_logTag, "failed to transmit: packet is too long for long interleaver coding rate");
				return SX1262Error::invalidArgument;
			}
		}

		auto error = setPacketLength(data.size());
		if (error != SX1262Error::none)
			return error;

		uint16_t IRQMask = IRQ_TX_DONE;

		if (timeoutUs > 0)
			IRQMask |= IRQ_TIMEOUT;

		error = setDIOIRQParams(IRQMask, IRQMask);
		if (error != SX1262Error::none)
			return error;

		error = setBufferBaseAddress();
		if (error != SX1262Error::none)
			return error;

		error = writeBuffer(data);
		if (error != SX1262Error::none)
			return error;

		error = clearIRQStatus();
		if (error != SX1262Error::none)
			return error;

		// Important shit
		error = fixLoRaTXModulationBeforeTransmission();
		if (error != SX1262Error::none)
			return error;

		// LET'S FUCKING MOOOOVE
		error = setTX(timeoutUs);
		if (error != SX1262Error::none)
			return error;

		error = waitForDIO1Semaphore(timeoutUs);

		if (error != SX1262Error::none) {
			ESP_LOGE(_logTag, "failed to transmit: dio1 timeout reached");

			return SX1262Error::timeout;
		}

		uint16_t IRQStatus = 0;

		error = getIRQStatus(IRQStatus);
		if (error != SX1262Error::none)
			return error;

		if (IRQStatus & IRQ_TIMEOUT) {
			//					ESP_LOGE(_logTag, "failed to transmit: IRQ timeout reached");

			return SX1262Error::timeout;
		}

		return SX1262Error::none;
	}

	SX1262Error SX1262::fixImplicitTimeout() {
		// fixes timeout in implicit header mode
		// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.3 for details

		// check if we're in implicit LoRa mode
		if (_headerType != LORA_HEADER_IMPLICIT || _packetType != PACKET_TYPE_LORA) {
			// not in the correct mode, nothing to do here
			return SX1262Error::none;
		}

		// stop RTC counter
		constexpr uint8_t rtcStop = 0x00;

		auto error = SPIWriteRegister(REG_RTC_CTRL, { &rtcStop, 1 });
		if (error != SX1262Error::none)
			return error;

		// read currently active event
		uint8_t rtcEvent = 0;

		error = SPIWriteRegister(REG_EVENT_MASK, { &rtcEvent, 1 });
		if (error != SX1262Error::none)
			return error;

		// clear events
		rtcEvent |= 0x02;

		error = SPIWriteRegister(REG_EVENT_MASK, { &rtcEvent, 1 });
		if (error != SX1262Error::none)
			return error;

		return SX1262Error::none;
	}

	SX1262Error SX1262::getReceivedPacketLength(uint8_t& length, uint8_t& offset) {
		// in implicit mode, return the cached value if the offset was not requested
		if ((_packetType == PACKET_TYPE_LORA) && (_headerType == LORA_HEADER_IMPLICIT) && (!offset)) {
			length = _packetLength;
			offset = 0;

			return SX1262Error::none;
		}

		// if offset was requested, or in explicit mode, we always have to perform the SPI transaction
		uint8_t data[] {
			0,
			0
		};

		const auto error = SPIReadCommand(CMD_GET_RX_BUFFER_STATUS, { data, 2 });
		if (error != SX1262Error::none)
			return error;

		length = data[0];
		offset = data[1];

		return SX1262Error::none;
	}

	SX1262Error SX1262::finishReceive() {
		// try to fix timeout error in implicit header mode
		// check for modem type and header mode is done in fixImplicitTimeout()
		const auto error = fixImplicitTimeout();
		if (error != SX1262Error::none)
			return error;

		return error;
	}

	SX1262Error SX1262::receive(uint8_t* buffer, uint8_t& receivedLength, uint32_t timeoutUs) {
		//				if (!setStandby())
		//					return false;

		uint16_t IRQMask = IRQ_RX_DONE;

		if (timeoutUs > 0)
			IRQMask |= IRQ_TIMEOUT;

		auto error = setDIOIRQParams(IRQMask, IRQMask);
		if (error != SX1262Error::none)
			return error;

		error = clearIRQStatus();
		if (error != SX1262Error::none)
			return error;

		error = setBufferBaseAddress();
		if (error != SX1262Error::none)
			return error;

		error = setPacketLength(255);
		if (error != SX1262Error::none)
			return error;

		// LET'S FUCKING MOOOOVE
		error = setRX(timeoutUs);
		if (error != SX1262Error::none)
			return error;

		error = waitForDIO1Semaphore(timeoutUs);

		if (error != SX1262Error::none) {
			ESP_LOGE(_logTag, "failed to receive: dio1 timeout reached");

			finishReceive();

			return SX1262Error::timeout;
		}

		uint16_t IRQStatus = 0;

		error = getIRQStatus(IRQStatus);
		if (error != SX1262Error::none)
			return error;

		error = finishReceive();
		if (error != SX1262Error::none)
			return error;

		if (IRQStatus & IRQ_TIMEOUT) {
			//					ESP_LOGE(_logTag, "failed to receive: IRQ timeout reached");

			return SX1262Error::timeout;
		}

		// check integrity CRC
		// Report CRC mismatch when there's a payload CRC error, or a header error and no valid header (to avoid false alarm from previous packet)
		if ((IRQStatus & IRQ_CRC_ERR) || ((IRQStatus & IRQ_HEADER_ERR) && !(IRQStatus & IRQ_HEADER_VALID))) {
			ESP_LOGE(_logTag, "failed to receive: CRC mismatch");
			return SX1262Error::invalidChecksum;
		}

		// get packet length and Rx buffer offset
		uint8_t offset = 0;

		error = getReceivedPacketLength(receivedLength, offset);
		if (error != SX1262Error::none)
			return error;

		//				ESP_LOGI(_logTag, "receive() length: %d, offset: %d", length, offset);

		// read packet data starting at offset
		error = readBuffer({ buffer, receivedLength }, offset);
		if (error != SX1262Error::none)
			return error;

		return SX1262Error::none;
	}

	SX1262Error SX1262::fixLoRaTXModulationBeforeTransmission() {
		// fix tx modulation for 500 kHz LoRa
		// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.1 for details

		uint8_t txModulation = 0;

		const auto error = SPIReadRegister(REG_TX_MODULATION, { &txModulation, 1 });
		if (error != SX1262Error::none)
			return error;

		// fix the value for LoRa with 500 kHz bandwidth
		if (_packetType == PACKET_TYPE_LORA && _LoRaBandwidth == SX1262LoRaBandwidth::bw500_0) {
			txModulation &= 0xFB;
		}
		else {
			txModulation |= 0x04;
		}

		return SPIWriteRegister(REG_TX_MODULATION, { &txModulation, 1 });
	}

	SX1262Error SX1262::setPacketLength(const uint8_t packetLength) {
		_packetLength = packetLength;

		return updatePacketParams();
	}

	SX1262Error SX1262::setPAConfig(const uint8_t paDutyCycle, const uint8_t deviceSel, const uint8_t hpMax, const uint8_t paLut) {
		const uint8_t data[5] {
			CMD_SET_PA_CONFIG,
			paDutyCycle,
			hpMax,
			deviceSel,
			paLut
		};

		return SPIWrite(data, 5);
	}

	SX1262Error SX1262::setTXParams(const int8_t power, const uint8_t rampTime) {
		if (power < -9 || power > 22) {
			ESP_LOGE(_logTag, "set output power failed: value %d is out of range [-9; 22]", power);
			return SX1262Error::invalidArgument;
		}

		const uint8_t data[] {
			CMD_SET_TX_PARAMS,
			static_cast<uint8_t>(power),
			rampTime
		};

		return SPIWrite(data, 3);
	}

	// const char* SX1262::errorToString(const SX1262Error error) {
	// 	switch (error) {
	// 		case SX1262Error::none: return "none";
	// 		case SX1262Error::invalidChip: return "invalid chip";
	// 		case SX1262Error::SPITransaction: return "SPI transaction";
	// 		case SX1262Error::timeout: return "timeout";
	// 		case SX1262Error::invalidArgument: return "invalid argument";
	// 		case SX1262Error::invalidPacketType: return "invalid packet type";
	// 		default: return "invalid checksum";
	// 	}
	// }

	void SX1262::errorToString(const SX1262Error error, const std::span<char> str) {
		switch (error) {
			case SX1262Error::none:
				std::strncpy(str.data(), "none", str.size());
				break;
			case SX1262Error::invalidChip:
				std::strncpy(str.data(), "invalid chip", str.size());
				break;
			case SX1262Error::SPITransaction:
				std::strncpy(str.data(), "SPI transaction", str.size());
				break;
			case SX1262Error::timeout:
				std::strncpy(str.data(), "timeout", str.size());
				break;
			case SX1262Error::invalidArgument:
				std::strncpy(str.data(), "invalid argument", str.size());
				break;
			case SX1262Error::invalidPacketType:
				std::strncpy(str.data(), "invalid packet type", str.size());
				break;
			case SX1262Error::invalidChecksum:
				std::strncpy(str.data(), "invalid checksum", str.size());
				break;
		}
	}

	void SX1262::setSSPinLevel(const bool value) const {
		gpio_set_level(_SSPin, value);
	}

	void SX1262::setRSTPinLevel(const bool value) const {
		gpio_set_level(_RSTPin, value);
	}

	bool SX1262::getBusyPinLevel() const {
		return gpio_get_level(_busyPin);
	}

	bool SX1262::getDIO1PinLevel() const {
		return gpio_get_level(_DIO1Pin);
	}

	void SX1262::onBusyPinInterrupt(void* arg) {
		BaseType_t higherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(static_cast<SX1262*>(arg)->_busyPinSemaphore, &higherPriorityTaskWoken);

		if (higherPriorityTaskWoken)
			portYIELD_FROM_ISR();
	}

	void SX1262::onDIO1PinInterrupt(void* arg) {
		BaseType_t higherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(static_cast<SX1262*>(arg)->_DIO1PinSemaphore, &higherPriorityTaskWoken);

		if (higherPriorityTaskWoken)
			portYIELD_FROM_ISR();
	}

	SX1262Error SX1262::waitForBusyPin(const uint32_t timeoutMs) const {
		if (!getBusyPinLevel() || xSemaphoreTake(_busyPinSemaphore, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)
			return SX1262Error::none;

		ESP_LOGE(_logTag, "failed to wait for busy pin: timeout reached");

		return SX1262Error::timeout;
	}

	bool SX1262::SPITransmit(spi_transaction_t* t) const {
		if (_SPIMutex)
			xSemaphoreTake(_SPIMutex, portMAX_DELAY);

		const auto state = checkESPError(spi_device_transmit(_SPIDevice, t));

		if (_SPIMutex)
			xSemaphoreGive(_SPIMutex);

		return state;
	}

	SX1262Error SX1262::SPIReadCommand(const uint8_t command, std::span<uint8_t> data) {
		if (waitForBusyPin() == SX1262Error::timeout)
			return SX1262Error::timeout;

		_SPIBuffer[0] = command; // W: command | R: status
		_SPIBuffer[1] = 0x00;    // W: -       | R: status
		_SPIBuffer[2] = 0x00;    // W: -       | R: result

		spi_transaction_t t {};
		t.tx_buffer = _SPIBuffer;
		t.rx_buffer = _SPIBuffer;
		t.length = 8 * (2 + data.size());

		const auto state = SPITransmit(&t);

		if (state) {
			//				for (int i = 0; i < 3; ++i) {
			//					ESP_LOGI(_logTag, "SPIReadCommand buffer[%d]: %d", i, _SPIBuffer[i]);
			//				}

			std::memcpy(data.data(), _SPIBuffer + 2, data.size());

			return SX1262Error::none;
		}

		return SX1262Error::SPITransaction;
	}

	SX1262Error SX1262::SPIReadRegister(const uint16_t reg, const std::span<uint8_t> data) {
		if (waitForBusyPin() == SX1262Error::timeout)
			return SX1262Error::timeout;

		_SPIBuffer[0] = CMD_READ_REGISTER; // W: command  | R: status
		_SPIBuffer[1] = (reg >> 8) & 0xFF; // W: Reg MSB  | R: status
		_SPIBuffer[2] = reg & 0xFF;        // W: Reg LSB  | R: status
		_SPIBuffer[3] = 0x00;              //             | R: status
		//         4                                      | R: data start...

		spi_transaction_t t {};
		t.tx_buffer = _SPIBuffer;
		t.rx_buffer = _SPIBuffer;
		t.length = 8 * (4 + data.size());

		const auto state = SPITransmit(&t);

		if (state) {
			std::memcpy(data.data(), _SPIBuffer + 4, data.size());

			//				for (int i = 0; i < 4 + length; ++i) {
			//					ESP_LOGI(_logTag, "SPIReadRegister buffer[%d]: %d", i, _SPIBuffer[i]);
			//				}
			return SX1262Error::none;
		}

		return SX1262Error::SPITransaction;
	}

	SX1262Error SX1262::SPIWrite(const uint8_t* data, const uint16_t length) {
		std::memcpy(_SPIBuffer, data, length);

		return SPIWriteBuffer(length);
	}

	SX1262Error SX1262::SPIWriteCommandAndUint8(const uint8_t command, const uint8_t data) {
		_SPIBuffer[0] = command;
		_SPIBuffer[1] = data;

		return SPIWriteBuffer(2);
	}

	SX1262Error SX1262::SPIWriteRegister(const uint16_t reg, const std::span<const uint8_t> data) {
		_SPIBuffer[0] = CMD_WRITE_REGISTER;
		_SPIBuffer[1] = (reg >> 8) & 0xFF;  // Reg MSB
		_SPIBuffer[2] = reg & 0xFF;         // Reg LSB

		std::memcpy(_SPIBuffer + 3, data.data(), data.size());

		return SPIWriteBuffer(3 + data.size());
	}

	SX1262Error SX1262::SPIWriteBuffer(const uint16_t totalLength) const {
		if (waitForBusyPin() == SX1262Error::timeout)
			return SX1262Error::timeout;

		spi_transaction_t t {};
		t.tx_buffer = _SPIBuffer;
		t.length = 8 * totalLength;

		return SPITransmit(&t) ? SX1262Error::none : SX1262Error::SPITransaction;
	}

	bool SX1262::checkESPError(const esp_err_t error) {
		if (error != ESP_OK) {
			ESP_ERROR_CHECK_WITHOUT_ABORT(error);
			return false;
		}

		return true;
	}

	SX1262Error SX1262::checkForLoRaPacketType() {
		uint8_t packetType = 0;

		const auto error = getPacketType(packetType);

		if (error != SX1262Error::none)
			return error;

		if (packetType != PACKET_TYPE_LORA) {
			ESP_LOGE(_logTag, "failed to set coding rate: packet type %d is not LoRa", packetType);
			return SX1262Error::invalidPacketType;
		}

		return SX1262Error::none;
	}

	void SX1262::delayMs(const uint32_t ms) {
		vTaskDelay(pdMS_TO_TICKS(std::max<uint32_t>(ms, portTICK_PERIOD_MS)));
	}

	SX1262Error SX1262::setRXOrTX(const uint8_t command, const uint32_t timeoutUs) {
		// From datasheet: timeoutUs = timeoutValue * 15.625 µs
		const uint32_t timeout = timeoutUs / 15.625f;

		const uint8_t data[] {
			command,
			static_cast<uint8_t>((timeout >> 16) & 0xFF),
			static_cast<uint8_t>((timeout >> 8) & 0xFF),
			static_cast<uint8_t>(timeout & 0xFF)
		};

		return SPIWrite(data, 4);
	}

	SX1262Error SX1262::fixInvertedIQ(const uint8_t iqConfig) {
		// fixes IQ configuration for inverted IQ
		// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details

		// read current IQ configuration
		uint8_t iqConfigCurrent = 0;

		const auto error = SPIReadRegister(REG_IQ_CONFIG, { &iqConfigCurrent, 1 });

		if (error != SX1262Error::none)
			return error;

		// set correct IQ configuration
		if (iqConfig == LORA_IQ_INVERTED) {
			iqConfigCurrent &= 0xFB;
		}
		else {
			iqConfigCurrent |= 0x04;
		}

		// update with the new value
		return SPIWriteRegister(REG_IQ_CONFIG, { &iqConfigCurrent, 1 });
	}

	float SX1262::getRSSIFromPacketStatus(const uint32_t packetStatus) {
		const uint8_t RSSIValue = packetStatus & 0xFF;

		return RSSIValue / -2.f;
	}

	float SX1262::getSNRFromPacketStatus(const uint32_t packetStatus) {
		const uint8_t snrPkt = (packetStatus >> 8) & 0xFF;

		if (snrPkt < 128)
			return snrPkt / 4.f;

		return (snrPkt - 256) / 4.f;
	}

	SX1262Error SX1262::updatePacketParams() {
		return setPacketParams(
			_preambleLength,
			_headerType,
			_packetLength,
			_crcType,
			_invertIQ
		);
	}
}
