
// Based on excellent work of https://github.com/jgromes/RadioLib/
// and adapted for ESP-IDF realities without Arduino shitware
//
// - Replaced GPIO polling loops with interrupt-based FreeRTOS semaphores
// - Replaced HALs with native SPI and DMA transactions
// - Separated module interaction into 2 classes:
//     SX1262 is used for direct access - it contains register map and provides only basic methods for reading and
//       writing data without any abstractions, caching and overhead
//     SX1262Ex is based on the first one and provides high-level abstractions with all the necessary checks and
//       convenient methods

#pragma once

#include <cmath>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_timer.h>
#include <esp_log.h>

namespace YOBA {
	enum class SX1262Error {
		none,
		invalidChip,
		SPI,
		timeout,
		invalidArgument,
		invalidPacketType,
		invalidChecksum
	};
	
	class SX1262 {
		public:
			virtual SX1262Error setup(
				spi_host_device_t SPIHostDevice,
				uint32_t SPIFrequencyHz,
				gpio_num_t SSPin,
				gpio_num_t RSTPin,
				gpio_num_t busyPin
			) {
				_SSPin = SSPin;
				_RSTPin = RSTPin;
				_busyPin = busyPin;
				
				// -------------------------------- GPIO output --------------------------------
				
				// Output
				gpio_config_t g {};
				
				// SS
				g.pin_bit_mask = (1ULL << _SSPin);
				
				// RST
				if (_RSTPin != GPIO_NUM_NC) {
					g.pin_bit_mask |= (1ULL << _RSTPin);
				}
				
				g.mode = GPIO_MODE_OUTPUT;
				g.pull_up_en = GPIO_PULLUP_DISABLE;
				g.pull_down_en = GPIO_PULLDOWN_DISABLE;
				g.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&g);
				
				// Setting SS to high just in case
				setSSPinLevel(true);
				
				// -------------------------------- GPIO input --------------------------------
				
				// Busy
				if (_busyPin != GPIO_NUM_NC) {
					g = {};
					g.pin_bit_mask = 1ULL << _busyPin;
					g.mode = GPIO_MODE_INPUT;
					g.pull_up_en = GPIO_PULLUP_ENABLE;
					g.pull_down_en = GPIO_PULLDOWN_DISABLE;
					g.intr_type = GPIO_INTR_NEGEDGE;
					gpio_config(&g);
				}
				
				// -------------------------------- Interrupts --------------------------------
				
				_busyPinSemaphore = xSemaphoreCreateBinary();
				
				gpio_install_isr_service(0);
				gpio_isr_handler_add(_busyPin, onBusyPinInterrupt, this);
				
				// -------------------------------- SPI --------------------------------
				
				spi_device_interface_config_t SPIInterfaceConfig {};
				// CPOL = 0, CPHA = 0
				SPIInterfaceConfig.mode = 0;
				SPIInterfaceConfig.clock_speed_hz = SPIFrequencyHz;
				SPIInterfaceConfig.spics_io_num = _SSPin;
				SPIInterfaceConfig.queue_size = 1;
				
				auto error = spi_bus_add_device(SPIHostDevice, &SPIInterfaceConfig, &_SPIDevice);
				
				if (!checkESPError(error))
					return SX1262Error::SPI;
				
				return SX1262Error::none;
			}
			
			virtual SX1262Error reset() {
				// Toggling RST GPIO
				if (_RSTPin != GPIO_NUM_NC) {
					setRSTPinLevel(false);
					delayMs(10);
					setRSTPinLevel(true);
					delayMs(10);
				}
				
				// Trying to set mode to standby - SX126x often refuses first few commands after reset
				auto start = esp_timer_get_time();
				
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
				
				return SX1262Error::none;
			}
			
			virtual SX1262Error validateChip() {
				for (uint8_t i = 0; i < 10; ++i) {
					uint8_t buffer[16] = {0};
					SPIReadRegister(REG_VERSION_STRING, buffer, 16);
					
					if (strncmp(VERSION_STRING, reinterpret_cast<char*>(buffer), 6) == 0) {
						ESP_LOGI(_logTag, "chip version: %s", buffer);
						
						return SX1262Error::none;
					} else {
						ESP_LOGE(_logTag, "failed to validate chip: version mismatch, attempt is %d, value is %s", i, buffer);
						
						delayMs(10);
					}
				}
				
				ESP_LOGE(_logTag, "failed to validate chip: maximum attempts exceeded");
				
				return SX1262Error::invalidChip;
			}
			
			/*!
			  \brief Perform image rejection calibration for the specified frequency band.
			  WARNING: Use at your own risk! Setting incorrect values may lead to decreased performance
			  \param freqMin Frequency band lower bound.
			  \param freqMax Frequency band upper bound.
			  \returns \ref status_codes
			*/
			virtual SX1262Error calibrateImageRejection(uint16_t freqMin, uint16_t freqMax) {
				// calculate the calibration coefficients and calibrate image
				uint8_t data[3] = {
					CMD_CALIBRATE_IMAGE,
					(uint8_t) std::floor(((float) freqMin - 1.0f) / 4.0f),
					(uint8_t) std::ceil(((float) freqMax + 1.0f) / 4.0f)
				};
				
				data[0] = (data[0] % 2) ? data[0] : data[0] - 1;
				data[1] = (data[1] % 2) ? data[1] : data[1] + 1;
				
				return SPIWrite(data, 3);
			}
			
			/*!
			  \brief Perform image rejection calibration for the specified frequency.
			  Will try to use Semtech-defined presets first, and if none of them matches,
			  custom iamge calibration will be attempted using calibrateImageRejection.
			  \param frequencyMHz Frequency to perform the calibration for.
			  \returns \ref status_codes
			*/
			virtual SX1262Error calibrateImage(uint16_t frequencyMHz) {
				uint8_t data[3] = {
					CMD_CALIBRATE_IMAGE,
					0,
					0
				};
				
				// try to match the frequency ranges
				if ((frequencyMHz >= 902) && (frequencyMHz <= 928)) {
					data[1] = CAL_IMG_902_MHZ_1;
					data[2] = CAL_IMG_902_MHZ_2;
				} else if ((frequencyMHz >= 863) && (frequencyMHz <= 870)) {
					data[1] = CAL_IMG_863_MHZ_1;
					data[2] = CAL_IMG_863_MHZ_2;
				} else if ((frequencyMHz >= 779) && (frequencyMHz <= 787)) {
					data[1] = CAL_IMG_779_MHZ_1;
					data[2] = CAL_IMG_779_MHZ_2;
				} else if ((frequencyMHz >= 470) && (frequencyMHz <= 510)) {
					data[1] = CAL_IMG_470_MHZ_1;
					data[2] = CAL_IMG_470_MHZ_2;
				} else if ((frequencyMHz >= 430) && (frequencyMHz <= 440)) {
					data[1] = CAL_IMG_430_MHZ_1;
					data[2] = CAL_IMG_430_MHZ_2;
				}
				
				// matched with predefined ranges, do the calibration
				if (data[1]) {
					return SPIWrite(data, 3);
				}
				
				// if nothing matched, try custom calibration - they may or may not work
				ESP_LOGW(_logTag, "failed to match predefined frequency range, trying custom");
				return calibrateImageRejection(frequencyMHz - 4, frequencyMHz + 4);
			}
			
			virtual SX1262Error setRFFrequency(uint16_t frequencyMHz) {
				if (frequencyMHz < 120 || frequencyMHz > 960) {
					ESP_LOGE(_logTag, "failed to set frequency: value %d is out of range [120; 960]");
					
					return SX1262Error::invalidArgument;
				}
				
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
				
				const auto regValue = static_cast<uint32_t>(static_cast<uint64_t>(frequencyMHz) * static_cast<uint64_t>(RF_DIVIDER) / static_cast<uint64_t>(RF_CRYSTAL_FREQUENCY_MHZ));
				
				const uint8_t data[] = {
					CMD_SET_RF_FREQUENCY,
					static_cast<uint8_t>((regValue >> 24) & 0xFF),
					static_cast<uint8_t>((regValue >> 16) & 0xFF),
					static_cast<uint8_t>((regValue >> 8) & 0xFF),
					static_cast<uint8_t>(regValue & 0xFF)
				};
				
				return SPIWrite(data, 5);
			}
			
			virtual SX1262Error setStandby(uint8_t value = STANDBY_RC) {
				return SPIWriteCommandAndUint8(CMD_SET_STANDBY, value);
			}
			
			virtual SX1262Error setSymbNumTimeout(uint8_t value) {
				return SPIWriteCommandAndUint8(CMD_SET_LORA_SYMB_NUM_TIMEOUT, value);
			}
			
			virtual SX1262Error setRX(uint32_t timeoutUs = 0) {
				return setRXOrTX(CMD_SET_RX, timeoutUs);
			}
			
			virtual SX1262Error setTX(uint32_t timeoutUs = 0) {
				return setRXOrTX(CMD_SET_TX, timeoutUs);
			}
			
			virtual SX1262Error setRXTXFallbackMode(uint8_t value = RX_TX_FALLBACK_MODE_STDBY_RC) {
				return SPIWriteCommandAndUint8(CMD_SET_RX_TX_FALLBACK_MODE, value);
			}
			
			virtual SX1262Error setCADParams(uint8_t spreadingFactor) {
				const uint8_t data[] = {
					CMD_SET_CAD_PARAMS,
					CAD_ON_8_SYMB,
					static_cast<uint8_t>(spreadingFactor + 13),
					CAD_PARAM_DET_MIN,
					CAD_GOTO_STDBY,
					0x00,
					0x00,
					0x00
				};
				
				return SPIWrite(data, 8);
			}
			
			virtual SX1262Error setBufferBaseAddress(uint8_t rxAddress = 0x00, uint8_t txAddress = 0x00) {
				const uint8_t data[] = {
					CMD_SET_BUFFER_BASE_ADDRESS,
					rxAddress,
					txAddress
				};
				
				return SPIWrite(data, 3);
			}
			
			virtual SX1262Error getStatus(uint8_t& status) {
				return SPIReadCommand(CMD_GET_STATUS, &status, 1);
			}
			
			virtual SX1262Error getPacketType(uint8_t& packetType) {
				return SPIReadCommand(CMD_GET_PACKET_TYPE, &packetType, 1);
			}
			
			virtual SX1262Error setPacketType(uint8_t packetType) {
				return SPIWriteCommandAndUint8(CMD_SET_PACKET_TYPE, packetType);
			}
			
			virtual SX1262Error setRegulatorMode(uint8_t mode) {
				return SPIWriteCommandAndUint8(CMD_SET_REGULATOR_MODE, mode);
			}
			
			virtual SX1262Error getIRQStatus(uint16_t& status) {
				status = 0;
				
				const auto error = SPIReadCommand(CMD_GET_IRQ_STATUS, reinterpret_cast<uint8_t*>(&status), 2);
				
				if (error != SX1262Error::none) {
					ESP_LOGE(_logTag, "failed to get IRQ status");
					
					return error;
				}
				
				status = ((status & 0xFF) << 8) | ((status >> 8) & 0xFF);
				
				return SX1262Error::none;
			}
			
			virtual SX1262Error clearIRQStatus(uint16_t status = IRQ_ALL) {
				const uint8_t data[] = {
					CMD_CLEAR_IRQ_STATUS,
					(uint8_t) ((status >> 8) & 0xFF),
					(uint8_t) (status & 0xFF)
				};
				
				return SPIWrite(data, 3);
			}
			
			virtual SX1262Error setDIOIRQParams(uint16_t irqMask = IRQ_NONE, uint16_t dio1Mask = IRQ_NONE, uint16_t dio2Mask = IRQ_NONE, uint16_t dio3Mask = IRQ_NONE) {
				const uint8_t data[] = {
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
			
			virtual SX1262Error calibrate(uint8_t value) {
				return SPIWriteCommandAndUint8(CMD_CALIBRATE, value);
			}
			
			/*!
			 \brief Sets LoRa sync word.
			 
			 \param syncWord LoRa sync word to be set.
			 \param controlBits Undocumented control bits, required for compatibility purposes.
			 
			 \returns \ref status_codes
		   */
			virtual SX1262Error setSyncWord(uint8_t syncWord, uint8_t controlBits = 0x44) {
				const auto error = checkForLoRaPacketType();
				
				if (error != SX1262Error::none)
					return error;
				
				uint8_t data[] {
					static_cast<uint8_t>((syncWord & 0xF0) | ((controlBits & 0xF0) >> 4)),
					static_cast<uint8_t>(((syncWord & 0x0F) << 4) | (controlBits & 0x0F))
				};
			
				return SPIWriteRegister(REG_LORA_SYNC_WORD_MSB, data, 2);
			}
			
			/*!
			 \brief Sets current protection limit. Can be set in 2.5 mA steps.
			 \param currentLimit current protection limit to be set in mA. Allowed values range from 0 to 140.
			 \returns \ref status_codes
		   */
			virtual SX1262Error setCurrentLimit(float currentLimit) {
				// check allowed range
				if (currentLimit < 0 || currentLimit > 140) {
					ESP_LOGE(_logTag, "failed to set current limit: value %f is out of range [0; 140]", currentLimit);
					return SX1262Error::invalidArgument;
				}
				
				// calculate raw value
				const auto rawLimit = static_cast<uint8_t>(currentLimit / 2.5f);
				
				return SPIWriteRegister(REG_OCP_CONFIGURATION, &rawLimit, 1);
			}
			
			/*!
			  \brief Set DIO2 to function as RF switch (default in Semtech example designs).
			  \returns \ref status_codes
			*/
			virtual SX1262Error setDio2AsRfSwitch(bool enable) {
				return SPIWriteCommandAndUint8(CMD_SET_DIO2_AS_RF_SWITCH_CTRL, enable ? DIO2_AS_RF_SWITCH : DIO2_AS_IRQ);
			}
			
			virtual SX1262Error setModulationParams(
				uint8_t spreadingFactor,
				uint8_t bandwidth,
				uint8_t codingRate,
				uint8_t ldrOptimize
			) {
				// 500/9/8  - 0x09 0x04 0x03 0x00 - SF9, BW125, 4/8
				// 500/11/8 - 0x0B 0x04 0x03 0x00 - SF11 BW125, 4/7
				const uint8_t data[5] = {
					CMD_SET_MODULATION_PARAMS,
					spreadingFactor,
					bandwidth,
					codingRate,
					ldrOptimize
				};
				
				return SPIWrite(data, 5);
			}
			
			virtual SX1262Error setTXClampConfig(bool enable) {
				// fixes overly eager PA clamping
				// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.2 for details
				
				uint8_t clampConfig = 0;
				
				const auto error = SPIReadRegister(REG_TX_CLAMP_CONFIG, &clampConfig, 1);
				
				if (error != SX1262Error::none)
					return error;
				
				// apply or undo workaround
				if (enable) {
					clampConfig |= 0x1E;
				}
				else {
					clampConfig = (clampConfig & ~0x1E) | 0x08;
				}
				
				return SPIWriteRegister(REG_TX_CLAMP_CONFIG, &clampConfig, 1);
			}
			
			virtual SX1262Error setOutputPower(int8_t powerDBm) {
				// get current OCP configuration
				uint8_t ocp = 0;
				
				auto error = SPIReadRegister(REG_OCP_CONFIGURATION, &ocp, 1);
				
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
				return SPIWriteRegister(REG_OCP_CONFIGURATION, &ocp, 1);
			}
			
			virtual SX1262Error setPacketParams(
				uint16_t preambleLength,
				uint8_t headerType,
				uint8_t length,
				uint8_t crcType,
				uint8_t invertIQ
			) {
				const auto error = fixInvertedIQ(invertIQ);
				
				if (error != SX1262Error::none)
					return error;
				
				const uint8_t data[7] = {
					CMD_SET_PACKET_PARAMS,
					(uint8_t) ((preambleLength >> 8) & 0xFF),
					(uint8_t) (preambleLength & 0xFF),
					headerType,
					length,
					crcType,
					invertIQ
				};
				
				return SPIWrite(data, 7);
			}
			
			SX1262Error writeBuffer(const uint8_t* data, uint8_t length, uint8_t offset = 0x00) {
				_SPIBuffer[0] = CMD_WRITE_BUFFER;
				_SPIBuffer[1] = offset;
				
				std::memcpy(_SPIBuffer + 2, data, length);
				
				return writeSPIBuffer(2 + length);
			}
			
			SX1262Error readBuffer(uint8_t* data, uint8_t length, uint8_t offset) {
				if (waitForBusyPin() == SX1262Error::timeout)
					return SX1262Error::timeout;
					
				_SPIBuffer[0] = CMD_READ_BUFFER; // W: command | R: status
				_SPIBuffer[1] = offset;          // W: offset  | R: status
				_SPIBuffer[2] = 0x00;            // W: -       | R: status
				//        [3]                    // W: -       | R: result
				
				spi_transaction_t t {};
				t.tx_buffer = _SPIBuffer;
				t.rx_buffer = _SPIBuffer;
				t.length = 8 * (3 + length);
				
				const auto state = checkESPError(spi_device_transmit(_SPIDevice, &t));
				
				if (state) {
					std::memcpy(data, _SPIBuffer + 3, length);
					
					//
//				for (int i = 0; i < 3 + length; ++i) {
//					ESP_LOGI(_logTag, "readBuffer buffer[%d]: %d", i, _SPIBuffer[i]);
//				}
					return SX1262Error::none;
				}
				
				return SX1262Error::SPI;
			}
			
			SX1262Error getPacketStatus(uint32_t& status) {
				uint8_t data[3] = {0, 0, 0};
				
				const auto error = SPIReadCommand(CMD_GET_PACKET_STATUS, data, 3);
				
				if (error != SX1262Error::none)
					return error;
				
				status = ((((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | (uint32_t)data[2]);
				
				return SX1262Error::none;
			}
			
			SX1262Error getRSSI(float& rssi) {
				uint32_t packetStatus = 0;
				
				const auto error = getPacketStatus(packetStatus);
				
				if (error != SX1262Error::none)
					return error;
				
				rssi = getRSSIFromPacketStatus(packetStatus);
				
				return SX1262Error::none;
			}

			// get instantaneous RSSI value
			SX1262Error getRSSIInst(float& rssi) {
				uint8_t rssiRaw = 0;
				
				const auto error = SPIReadCommand(CMD_GET_RSSI_INST, &rssiRaw, 1);
				
				if (error != SX1262Error::none)
					return error;
				
				rssi = ((float) rssiRaw / (-2.0f));
				
				return SX1262Error::none;
			}
			
			SX1262Error getSNR(float& snr) {
				uint32_t packetStatus = 0;
				
				const auto error = getPacketStatus(packetStatus);
				
				if (error != SX1262Error::none)
					return error;
				
				snr = getSNRFromPacketStatus(packetStatus);
				
				return SX1262Error::none;
			}
			
		protected:
			bool checkESPError(esp_err_t error) {
				if (error != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(error);
					return false;
				}
				
				return true;
			}
			
			SX1262Error checkForLoRaPacketType() {
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
			
			void delayMs(uint32_t ms) {
				vTaskDelay(pdMS_TO_TICKS(std::max<uint32_t>(ms, portTICK_PERIOD_MS)));
			}
			
			SX1262Error waitForBusyPin(uint32_t timeoutMs = 1'000) {
				if (!getBusyPinLevel() || xSemaphoreTake(_busyPinSemaphore, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)
					return SX1262Error::none;

				ESP_LOGE(_logTag, "failed to wait for busy pin: timeout reached");

				return SX1262Error::timeout;
			}
			
			// -------------------------------- Reading --------------------------------
			
			SX1262Error SPIReadCommand(uint8_t command, uint8_t* data, uint8_t length) {
				if (waitForBusyPin() == SX1262Error::timeout)
					return SX1262Error::timeout;
				
				_SPIBuffer[0] = command; // W: command | R: status
				_SPIBuffer[1] = 0x00;    // W: -       | R: status
				_SPIBuffer[2] = 0x00;    // W: -       | R: result
				
				spi_transaction_t t {};
				t.tx_buffer = _SPIBuffer;
				t.rx_buffer = _SPIBuffer;
				t.length = 8 * (2 + length);
				
				const auto state = checkESPError(spi_device_transmit(_SPIDevice, &t));
				
				if (state) {
					//				for (int i = 0; i < 3; ++i) {
//					ESP_LOGI(_logTag, "SPIReadCommand buffer[%d]: %d", i, _SPIBuffer[i]);
//				}
					
					std::memcpy(data, _SPIBuffer + 2, length);
					
					return SX1262Error::none;
				}
				
				return SX1262Error::SPI;
			}
			
			SX1262Error SPIReadRegister(const uint16_t reg, uint8_t* data, const size_t length) {
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
				t.length = 8 * (4 + length);
				
				const auto state = checkESPError(spi_device_transmit(_SPIDevice, &t));
				
				if (state) {
					std::memcpy(data, _SPIBuffer + 4, length);
					
					//				for (int i = 0; i < 4 + length; ++i) {
//					ESP_LOGI(_logTag, "SPIReadRegister buffer[%d]: %d", i, _SPIBuffer[i]);
//				}
					return SX1262Error::none;
				}
				
				return SX1262Error::SPI;
			}
			
			// -------------------------------- Writing --------------------------------
			
			SX1262Error SPIWrite(const uint8_t* data, uint16_t length) {
				std::memcpy(_SPIBuffer, data, length);
				
				return writeSPIBuffer(length);
			}
			
			SX1262Error SPIWriteCommandAndUint8(uint8_t command, uint8_t data) {
				_SPIBuffer[0] = command;
				_SPIBuffer[1] = data;
				
				return writeSPIBuffer(2);
			}
			
			SX1262Error SPIWriteRegister(const uint16_t reg, const uint8_t* data, size_t length) {
				_SPIBuffer[0] = CMD_WRITE_REGISTER;
				_SPIBuffer[1] = (reg >> 8) & 0xFF; // Reg MSB
				_SPIBuffer[2] = reg & 0xFF;        // Reg LSB
				
				std::memcpy(_SPIBuffer + 3, data, length);
				
				return writeSPIBuffer(3 + length);
			}
		
		private:
			constexpr static const char* _logTag = "SX1262";
			
			gpio_num_t _SSPin = GPIO_NUM_NC;
			gpio_num_t _busyPin = GPIO_NUM_NC;
			gpio_num_t _RSTPin = GPIO_NUM_NC;
			
			SemaphoreHandle_t _busyPinSemaphore;
			spi_device_handle_t _SPIDevice {};
			
			constexpr static uint16_t _SPIBufferLength = 4 + 256;
			uint8_t _SPIBuffer[_SPIBufferLength] {};
			
			void setSSPinLevel(bool value) {
				gpio_set_level(_SSPin, value);
			}
			
			void setRSTPinLevel(bool value) {
				gpio_set_level(_RSTPin, value);
			}
			
			bool getBusyPinLevel() {
				return gpio_get_level(_busyPin);
			}
			
			IRAM_ATTR static void onBusyPinInterrupt(void* arg) {
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				
				xSemaphoreGiveFromISR(reinterpret_cast<SX1262*>(arg)->_busyPinSemaphore, &xHigherPriorityTaskWoken);
				
				if (xHigherPriorityTaskWoken) {
					portYIELD_FROM_ISR();
				}
			}
			
			// -------------------------------- Auxiliary --------------------------------
			
			SX1262Error writeSPIBuffer(uint16_t totalLength) {
				if (waitForBusyPin() == SX1262Error::timeout)
					return SX1262Error::timeout;
				
				spi_transaction_t t {};
				t.tx_buffer = _SPIBuffer;
				t.length = 8 * totalLength;
				
				return checkESPError(spi_device_transmit(_SPIDevice, &t)) ? SX1262Error::none : SX1262Error::SPI;
			}
			
			SX1262Error setRXOrTX(uint8_t command, uint32_t timeoutUs) {
				// From datasheet: timeoutUs = timeoutValue * 15.625 µs
				const uint32_t timeout = timeoutUs / 15.625f;
				
				const uint8_t data[] = {
					command,
					static_cast<uint8_t>((timeout >> 16) & 0xFF),
					static_cast<uint8_t>((timeout >> 8) & 0xFF),
					static_cast<uint8_t>(timeout & 0xFF)
				};
				
				return SPIWrite(data, 4);
			}
			
			SX1262Error fixInvertedIQ(uint8_t iqConfig) {
				// fixes IQ configuration for inverted IQ
				// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details
				
				// read current IQ configuration
				uint8_t iqConfigCurrent = 0;
				
				const auto error = SPIReadRegister(REG_IQ_CONFIG, &iqConfigCurrent, 1);
				
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
				return SPIWriteRegister(REG_IQ_CONFIG, &iqConfigCurrent, 1);
			}
			
			/*!
			 \brief Set the PA (power amplifier) configuration. Allows user to optimize PA for a specific output power
			 and matching network. Any calls to this method must be done after calling begin/beginFSK and/or setOutputPower.
			 WARNING: Use at your own risk! Setting invalid values can and will lead to permanent damage!
			 \param paDutyCycle PA duty cycle raw value.
			 \param deviceSel Device select, usually PA_CONFIG_SX1261,
			 PA_CONFIG_SX1262 or PA_CONFIG_SX1268.
			 \param hpMax hpMax raw value.
			 \param paLut paLut PA lookup table raw value.
			 \returns \ref status_codes
		   */
			SX1262Error setPAConfig(uint8_t paDutyCycle = 0x04, uint8_t deviceSel = PA_CONFIG_SX1262, uint8_t hpMax = PA_CONFIG_HP_MAX, uint8_t paLut = PA_CONFIG_PA_LUT) {
				const uint8_t data[5] = {
					CMD_SET_PA_CONFIG,
					paDutyCycle,
					hpMax,
					deviceSel,
					paLut
				};
				
				return SPIWrite(data, 5);
			}
			
			SX1262Error setTXParams(int8_t power, uint8_t rampTime) {
				if (power < -9 || power > 22) {
					ESP_LOGE(_logTag, "set output power failed: value %d is out of range [-9; 22]", power);
					return SX1262Error::invalidArgument;
				}
				
				const uint8_t data[] = {
					CMD_SET_TX_PARAMS,
					static_cast<uint8_t>(power),
					rampTime
				};
				
				return SPIWrite(data, 3);
			}
			
			float getRSSIFromPacketStatus(uint32_t packetStatus) {
				uint8_t rssiPkt = packetStatus & 0xFF;
				
				return -1.f * rssiPkt / 2.f;
			}
			
			float getSNRFromPacketStatus(uint32_t packetStatus) {
				uint8_t snrPkt = (packetStatus >> 8) & 0xFF;
				
				if (snrPkt < 128) {
					return snrPkt / 4.f;
				}
				else {
					return (snrPkt - 256) / 4.f;
				}
			}
			
		public:
			
			// -------------------------------- Module properties --------------------------------
			
			constexpr static uint32_t RF_CRYSTAL_FREQUENCY_MHZ = 32;
			constexpr static uint32_t RF_DIVIDER = 1ULL << 25;
			constexpr static uint8_t IMPLICIT_PACKET_LENGTH = 255;
			
			// -------------------------------- Commands --------------------------------
			
			// Operational modes commands
			constexpr static uint8_t CMD_NOP = 0x00;
			constexpr static uint8_t CMD_SET_SLEEP = 0x84;
			constexpr static uint8_t CMD_SET_STANDBY = 0x80;
			constexpr static uint8_t CMD_SET_FS = 0xC1;
			constexpr static uint8_t CMD_SET_TX = 0x83;
			constexpr static uint8_t CMD_SET_RX = 0x82;
			constexpr static uint8_t CMD_STOP_TIMER_ON_PREAMBLE = 0x9F;
			constexpr static uint8_t CMD_SET_RX_DUTY_CYCLE = 0x94;
			constexpr static uint8_t CMD_SET_CAD = 0xC5;
			constexpr static uint8_t CMD_SET_TX_CONTINUOUS_WAVE = 0xD1;
			constexpr static uint8_t CMD_SET_TX_INFINITE_PREAMBLE = 0xD2;
			constexpr static uint8_t CMD_SET_REGULATOR_MODE = 0x96;
			constexpr static uint8_t CMD_CALIBRATE = 0x89;
			constexpr static uint8_t CMD_CALIBRATE_IMAGE = 0x98;
			constexpr static uint8_t CMD_SET_PA_CONFIG = 0x95;
			constexpr static uint8_t CMD_SET_RX_TX_FALLBACK_MODE = 0x93;
			
			// register and buffer access commands
			constexpr static uint8_t CMD_WRITE_REGISTER = 0x0D;
			constexpr static uint8_t CMD_READ_REGISTER = 0x1D;
			constexpr static uint8_t CMD_WRITE_BUFFER = 0x0E;
			constexpr static uint8_t CMD_READ_BUFFER = 0x1E;
			
			// DIO and IRQ control
			constexpr static uint8_t CMD_SET_DIO_IRQ_PARAMS = 0x08;
			constexpr static uint8_t CMD_GET_IRQ_STATUS = 0x12;
			constexpr static uint8_t CMD_CLEAR_IRQ_STATUS = 0x02;
			constexpr static uint8_t CMD_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D;
			constexpr static uint8_t CMD_SET_DIO3_AS_TCXO_CTRL = 0x97;
			
			// RF, modulation and packet commands
			constexpr static uint8_t CMD_SET_RF_FREQUENCY = 0x86;
			constexpr static uint8_t CMD_SET_PACKET_TYPE = 0x8A;
			constexpr static uint8_t CMD_GET_PACKET_TYPE = 0x11;
			constexpr static uint8_t CMD_SET_TX_PARAMS = 0x8E;
			constexpr static uint8_t CMD_SET_MODULATION_PARAMS = 0x8B;
			constexpr static uint8_t CMD_SET_PACKET_PARAMS = 0x8C;
			constexpr static uint8_t CMD_SET_CAD_PARAMS = 0x88;
			constexpr static uint8_t CMD_SET_BUFFER_BASE_ADDRESS = 0x8F;
			constexpr static uint8_t CMD_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0;
			
			// status commands
			constexpr static uint8_t CMD_GET_STATUS = 0xC0;
			constexpr static uint8_t CMD_GET_RSSI_INST = 0x15;
			constexpr static uint8_t CMD_GET_RX_BUFFER_STATUS = 0x13;
			constexpr static uint8_t CMD_GET_PACKET_STATUS = 0x14;
			constexpr static uint8_t CMD_GET_DEVICE_ERRORS = 0x17;
			constexpr static uint8_t CMD_CLEAR_DEVICE_ERRORS = 0x07;
			constexpr static uint8_t CMD_GET_STATS = 0x10;
			constexpr static uint8_t CMD_RESET_STATS = 0x00;
			
			constexpr static uint8_t CMD_PRAM_UPDATE = 0xD9;
			constexpr static uint8_t CMD_SET_LBT_SCAN_PARAMS = 0x9A;
			constexpr static uint8_t CMD_SET_SPECTR_SCAN_PARAMS = 0x9B;
			
			// SX126X SPI command variables
			
			// CMD_SET_SLEEP                                                                    MSB   LSB   DESCRIPTION
			constexpr static uint8_t SLEEP_START_COLD = 0b00000000;  //  2     2   sleep mode: cold start, configuration is lost (default)
			constexpr static uint8_t SLEEP_START_WARM = 0b00000100;  //  2     2               warm start, configuration is retained
			constexpr static uint8_t SLEEP_RTC_OFF = 0b00000000;  //  0     0   wake on RTC timeout: disabled
			constexpr static uint8_t SLEEP_RTC_ON = 0b00000001;  //  0     0                        enabled
			
			// CMD_SET_STANDBY
			constexpr static uint8_t STANDBY_RC = 0x00;        //  7     0   standby mode: 13 MHz RC oscillator
			constexpr static uint8_t STANDBY_XOSC = 0x01;        //  7     0                 32 MHz crystal oscillator
			
			// CMD_SET_RX
			constexpr static uint32_t RX_TIMEOUT_NONE = 0x000000;    //  23    0   Rx timeout duration: no timeout (Rx single mode)
			constexpr static uint32_t RX_TIMEOUT_INF = 0xFFFFFF;    //  23    0                        infinite (Rx continuous mode)
			
			// CMD_SET_TX
			constexpr static uint8_t TX_TIMEOUT_NONE = 0x000000;    //  23    0   Tx timeout duration: no timeout (Tx single mode)
			
			// CMD_STOP_TIMER_ON_PREAMBLE
			constexpr static uint8_t STOP_ON_PREAMBLE_OFF = 0x00;        //  7     0   stop timer on: sync word or header (default)
			constexpr static uint8_t STOP_ON_PREAMBLE_ON = 0x01;        //  7     0                  preamble detection
			
			// CMD_SET_REGULATOR_MODE
			constexpr static uint8_t REGULATOR_LDO = 0x00;        //  7     0   set regulator mode: LDO (default)
			constexpr static uint8_t REGULATOR_DC_DC = 0x01;        //  7     0                       DC-DC
			
			// CMD_CALIBRATE
			constexpr static uint8_t CALIBRATE_IMAGE_OFF = 0b00000000;  //  6     6   image calibration: disabled
			constexpr static uint8_t CALIBRATE_IMAGE_ON = 0b01000000;  //  6     6                      enabled
			constexpr static uint8_t CALIBRATE_ADC_BULK_P_OFF = 0b00000000;  //  5     5   ADC bulk P calibration: disabled
			constexpr static uint8_t CALIBRATE_ADC_BULK_P_ON = 0b00100000;  //  5     5                           enabled
			constexpr static uint8_t CALIBRATE_ADC_BULK_N_OFF = 0b00000000;  //  4     4   ADC bulk N calibration: disabled
			constexpr static uint8_t CALIBRATE_ADC_BULK_N_ON = 0b00010000;  //  4     4                           enabled
			constexpr static uint8_t CALIBRATE_ADC_PULSE_OFF = 0b00000000;  //  3     3   ADC pulse calibration: disabled
			constexpr static uint8_t CALIBRATE_ADC_PULSE_ON = 0b00001000;  //  3     3                          enabled
			constexpr static uint8_t CALIBRATE_PLL_OFF = 0b00000000;  //  2     2   PLL calibration: disabled
			constexpr static uint8_t CALIBRATE_PLL_ON = 0b00000100;  //  2     2                    enabled
			constexpr static uint8_t CALIBRATE_RC13M_OFF = 0b00000000;  //  1     1   13 MHz RC osc. calibration: disabled
			constexpr static uint8_t CALIBRATE_RC13M_ON = 0b00000010;  //  1     1                               enabled
			constexpr static uint8_t CALIBRATE_RC64K_OFF = 0b00000000;  //  0     0   64 kHz RC osc. calibration: disabled
			constexpr static uint8_t CALIBRATE_RC64K_ON = 0b00000001;  //  0     0                               enabled
			constexpr static uint8_t CALIBRATE_ALL = 0b01111111;  //  6     0   calibrate all blocks
			
			// CMD_CALIBRATE_IMAGE
			constexpr static uint8_t CAL_IMG_430_MHZ_1 = 0x6B;
			constexpr static uint8_t CAL_IMG_430_MHZ_2 = 0x6F;
			constexpr static uint8_t CAL_IMG_470_MHZ_1 = 0x75;
			constexpr static uint8_t CAL_IMG_470_MHZ_2 = 0x81;
			constexpr static uint8_t CAL_IMG_779_MHZ_1 = 0xC1;
			constexpr static uint8_t CAL_IMG_779_MHZ_2 = 0xC5;
			constexpr static uint8_t CAL_IMG_863_MHZ_1 = 0xD7;
			constexpr static uint8_t CAL_IMG_863_MHZ_2 = 0xDB;
			constexpr static uint8_t CAL_IMG_902_MHZ_1 = 0xE1;
			constexpr static uint8_t CAL_IMG_902_MHZ_2 = 0xE9;
			constexpr static uint8_t CAL_IMG_FREQ_TRIG_MHZ = 20;
			
			// CMD_SET_PA_CONFIG
			constexpr static uint8_t PA_CONFIG_HP_MAX = 0x07;
			constexpr static uint8_t PA_CONFIG_PA_LUT = 0x01;
			constexpr static uint8_t PA_CONFIG_SX1262 = 0x00;
			
			// CMD_SET_RX_TX_FALLBACK_MODE
			constexpr static uint8_t RX_TX_FALLBACK_MODE_FS = 0x40;        //  7     0   after Rx/Tx go to: FS mode
			constexpr static uint8_t RX_TX_FALLBACK_MODE_STDBY_XOSC = 0x30;        //  7     0                      standby with crystal oscillator
			constexpr static uint8_t RX_TX_FALLBACK_MODE_STDBY_RC = 0x20;        //  7     0                      standby with RC oscillator (default)
			
			// CMD_SET_DIO_IRQ_PARAMS
			constexpr static uint16_t IRQ_LR_FHSS_HOP = 0b0100000000000000;  //  14    14  PA ramped up during LR-FHSS hop
			constexpr static uint16_t IRQ_TIMEOUT = 0b0000001000000000;  //  9     9   Rx or Tx timeout
			constexpr static uint16_t IRQ_CAD_DETECTED = 0b0000000100000000;  //  8     8   channel activity detected
			constexpr static uint16_t IRQ_CAD_DONE = 0b0000000010000000;  //  7     7   channel activity detection finished
			constexpr static uint16_t IRQ_CRC_ERR = 0b0000000001000000;  //  6     6   wrong CRC received
			constexpr static uint16_t IRQ_HEADER_ERR = 0b0000000000100000;  //  5     5   LoRa header CRC error
			constexpr static uint16_t IRQ_HEADER_VALID = 0b0000000000010000;  //  4     4   valid LoRa header received
			constexpr static uint16_t IRQ_SYNC_WORD_VALID = 0b0000000000001000;  //  3     3   valid sync word detected
			constexpr static uint16_t IRQ_PREAMBLE_DETECTED = 0b0000000000000100;  //  2     2   preamble detected
			constexpr static uint16_t IRQ_RX_DONE = 0b0000000000000010;  //  1     1   packet received
			constexpr static uint16_t IRQ_TX_DONE = 0b0000000000000001;  //  0     0   packet transmission completed
			constexpr static uint16_t IRQ_ALL = 0b0100001111111111;  //  14    0   all interrupts
			constexpr static uint16_t IRQ_NONE = 0b0000000000000000;  //  14    0   no interrupts
			
			// CMD_SET_DIO2_AS_RF_SWITCH_CTRL
			constexpr static uint8_t DIO2_AS_IRQ = 0x00;        //  7     0   DIO2 configuration: IRQ
			constexpr static uint8_t DIO2_AS_RF_SWITCH = 0x01;        //  7     0                       RF switch control
			
			// CMD_SET_DIO3_AS_TCXO_CTRL
			constexpr static uint8_t DIO3_OUTPUT_1_6 = 0x00;        //  7     0   DIO3 voltage output for TCXO: 1.6 V
			constexpr static uint8_t DIO3_OUTPUT_1_7 = 0x01;        //  7     0                                 1.7 V
			constexpr static uint8_t DIO3_OUTPUT_1_8 = 0x02;        //  7     0                                 1.8 V
			constexpr static uint8_t DIO3_OUTPUT_2_2 = 0x03;        //  7     0                                 2.2 V
			constexpr static uint8_t DIO3_OUTPUT_2_4 = 0x04;        //  7     0                                 2.4 V
			constexpr static uint8_t DIO3_OUTPUT_2_7 = 0x05;        //  7     0                                 2.7 V
			constexpr static uint8_t DIO3_OUTPUT_3_0 = 0x06;        //  7     0                                 3.0 V
			constexpr static uint8_t DIO3_OUTPUT_3_3 = 0x07;        //  7     0                                 3.3 V
			
			// CMD_SET_PACKET_TYPE
			constexpr static uint8_t PACKET_TYPE_GFSK = 0x00;        //  7     0   packet type: GFSK
			constexpr static uint8_t PACKET_TYPE_LORA = 0x01;        //  7     0                LoRa
			constexpr static uint8_t PACKET_TYPE_BPSK = 0x02;        //  7     0                BPSK
			constexpr static uint8_t PACKET_TYPE_LR_FHSS = 0x03;        //  7     0                LR-FHSS
			
			// CMD_SET_TX_PARAMS
			constexpr static uint8_t PA_RAMP_10U = 0x00;        //  7     0   ramp time: 10 us
			constexpr static uint8_t PA_RAMP_20U = 0x01;        //  7     0              20 us
			constexpr static uint8_t PA_RAMP_40U = 0x02;        //  7     0              40 us
			constexpr static uint8_t PA_RAMP_80U = 0x03;        //  7     0              80 us
			constexpr static uint8_t PA_RAMP_200U = 0x04;        //  7     0              200 us
			constexpr static uint8_t PA_RAMP_800U = 0x05;        //  7     0              800 us
			constexpr static uint8_t PA_RAMP_1700U = 0x06;        //  7     0              1700 us
			constexpr static uint8_t PA_RAMP_3400U = 0x07;        //  7     0              3400 us
			
			// CMD_SET_MODULATION_PARAMS
			constexpr static uint8_t GFSK_FILTER_NONE = 0x00;        //  7     0   GFSK filter: none
			constexpr static uint8_t GFSK_FILTER_GAUSS_0_3 = 0x08;        //  7     0                Gaussian, BT = 0.3
			constexpr static uint8_t GFSK_FILTER_GAUSS_0_5 = 0x09;        //  7     0                Gaussian, BT = 0.5
			constexpr static uint8_t GFSK_FILTER_GAUSS_0_7 = 0x0A;        //  7     0                Gaussian, BT = 0.7
			constexpr static uint8_t GFSK_FILTER_GAUSS_1 = 0x0B;        //  7     0                Gaussian, BT = 1
			constexpr static uint8_t GFSK_RX_BW_4_8 = 0x1F;        //  7     0   GFSK Rx bandwidth: 4.8 kHz
			constexpr static uint8_t GFSK_RX_BW_5_8 = 0x17;        //  7     0                      5.8 kHz
			constexpr static uint8_t GFSK_RX_BW_7_3 = 0x0F;        //  7     0                      7.3 kHz
			constexpr static uint8_t GFSK_RX_BW_9_7 = 0x1E;        //  7     0                      9.7 kHz
			constexpr static uint8_t GFSK_RX_BW_11_7 = 0x16;        //  7     0                      11.7 kHz
			constexpr static uint8_t GFSK_RX_BW_14_6 = 0x0E;        //  7     0                      14.6 kHz
			constexpr static uint8_t GFSK_RX_BW_19_5 = 0x1D;        //  7     0                      19.5 kHz
			constexpr static uint8_t GFSK_RX_BW_23_4 = 0x15;        //  7     0                      23.4 kHz
			constexpr static uint8_t GFSK_RX_BW_29_3 = 0x0D;        //  7     0                      29.3 kHz
			constexpr static uint8_t GFSK_RX_BW_39_0 = 0x1C;        //  7     0                      39.0 kHz
			constexpr static uint8_t GFSK_RX_BW_46_9 = 0x14;        //  7     0                      46.9 kHz
			constexpr static uint8_t GFSK_RX_BW_58_6 = 0x0C;        //  7     0                      58.6 kHz
			constexpr static uint8_t GFSK_RX_BW_78_2 = 0x1B;        //  7     0                      78.2 kHz
			constexpr static uint8_t GFSK_RX_BW_93_8 = 0x13;        //  7     0                      93.8 kHz
			constexpr static uint8_t GFSK_RX_BW_117_3 = 0x0B;        //  7     0                      117.3 kHz
			constexpr static uint8_t GFSK_RX_BW_156_2 = 0x1A;        //  7     0                      156.2 kHz
			constexpr static uint8_t GFSK_RX_BW_187_2 = 0x12;        //  7     0                      187.2 kHz
			constexpr static uint8_t GFSK_RX_BW_234_3 = 0x0A;        //  7     0                      234.3 kHz
			constexpr static uint8_t GFSK_RX_BW_312_0 = 0x19;        //  7     0                      312.0 kHz
			constexpr static uint8_t GFSK_RX_BW_373_6 = 0x11;        //  7     0                      373.6 kHz
			constexpr static uint8_t GFSK_RX_BW_467_0 = 0x09;        //  7     0                      467.0 kHz
			constexpr static uint8_t LORA_BW_7_8 = 0x00;        //  7     0   LoRa bandwidth: 7.8 kHz
			constexpr static uint8_t LORA_BW_10_4 = 0x08;        //  7     0                   10.4 kHz
			constexpr static uint8_t LORA_BW_15_6 = 0x01;        //  7     0                   15.6 kHz
			constexpr static uint8_t LORA_BW_20_8 = 0x09;        //  7     0                   20.8 kHz
			constexpr static uint8_t LORA_BW_31_25 = 0x02;        //  7     0                   31.25 kHz
			constexpr static uint8_t LORA_BW_41_7 = 0x0A;        //  7     0                   41.7 kHz
			constexpr static uint8_t LORA_BW_62_5 = 0x03;        //  7     0                   62.5 kHz
			constexpr static uint8_t LORA_BW_125_0 = 0x04;        //  7     0                   125.0 kHz
			constexpr static uint8_t LORA_BW_250_0 = 0x05;        //  7     0                   250.0 kHz
			constexpr static uint8_t LORA_BW_500_0 = 0x06;        //  7     0                   500.0 kHz
			constexpr static uint8_t LORA_CR_4_5 = 0x01;        //  7     0   LoRa coding rate: 4/5
			constexpr static uint8_t LORA_CR_4_6 = 0x02;        //  7     0                     4/6
			constexpr static uint8_t LORA_CR_4_7 = 0x03;        //  7     0                     4/7
			constexpr static uint8_t LORA_CR_4_8 = 0x04;        //  7     0                     4/8
			constexpr static uint8_t LORA_CR_4_5_LI = 0x05;        //  7     0                     4/5, long interleaver
			constexpr static uint8_t LORA_CR_4_6_LI = 0x06;        //  7     0                     4/6, long interleaver
			constexpr static uint8_t LORA_CR_4_8_LI = 0x07;        //  7     0                     4/8, long interleaver
			constexpr static uint8_t LORA_LOW_DATA_RATE_OPTIMIZE_OFF = 0x00;        //  7     0   LoRa low data rate optimization: disabled
			constexpr static uint8_t LORA_LOW_DATA_RATE_OPTIMIZE_ON = 0x01;        //  7     0                                    enabled
			constexpr static uint8_t BPSK_PULSE_SHAPE = 0x16;        //  7     0   BSPK pulse shape double OSR, RRC, BT=0.7
			
			// CMD_SET_PACKET_PARAMS
			constexpr static uint8_t GFSK_PREAMBLE_DETECT_OFF = 0x00;        //  7     0   GFSK minimum preamble length before reception starts: detector disabled
			constexpr static uint8_t GFSK_PREAMBLE_DETECT_8 = 0x04;        //  7     0                                                         8 bits
			constexpr static uint8_t GFSK_PREAMBLE_DETECT_16 = 0x05;        //  7     0                                                         16 bits
			constexpr static uint8_t GFSK_PREAMBLE_DETECT_24 = 0x06;        //  7     0                                                         24 bits
			constexpr static uint8_t GFSK_PREAMBLE_DETECT_32 = 0x07;        //  7     0                                                         32 bits
			constexpr static uint8_t GFSK_ADDRESS_FILT_OFF = 0x00;        //  7     0   GFSK address filtering: disabled
			constexpr static uint8_t GFSK_ADDRESS_FILT_NODE = 0x01;        //  7     0                           node only
			constexpr static uint8_t GFSK_ADDRESS_FILT_NODE_BROADCAST = 0x02;        //  7     0                           node and broadcast
			constexpr static uint8_t GFSK_PACKET_FIXED = 0x00;        //  7     0   GFSK packet type: fixed (payload length known in advance to both sides)
			constexpr static uint8_t GFSK_PACKET_VARIABLE = 0x01;        //  7     0                     variable (payload length added to packet)
			constexpr static uint8_t GFSK_CRC_OFF = 0x01;        //  7     0   GFSK packet CRC: disabled
			constexpr static uint8_t GFSK_CRC_1_BYTE = 0x00;        //  7     0                    1 byte
			constexpr static uint8_t GFSK_CRC_2_BYTE = 0x02;        //  7     0                    2 byte
			constexpr static uint8_t GFSK_CRC_1_BYTE_INV = 0x04;        //  7     0                    1 byte, inverted
			constexpr static uint8_t GFSK_CRC_2_BYTE_INV = 0x06;        //  7     0                    2 byte, inverted
			constexpr static uint8_t GFSK_WHITENING_OFF = 0x00;        //  7     0   GFSK data whitening: disabled
			constexpr static uint8_t GFSK_WHITENING_ON = 0x01;        //  7     0                        enabled
			constexpr static uint8_t LORA_HEADER_EXPLICIT = 0x00;        //  7     0   LoRa header mode: explicit
			constexpr static uint8_t LORA_HEADER_IMPLICIT = 0x01;        //  7     0                     implicit
			constexpr static uint8_t LORA_CRC_OFF = 0x00;        //  7     0   LoRa CRC mode: disabled
			constexpr static uint8_t LORA_CRC_ON = 0x01;        //  7     0                  enabled
			constexpr static uint8_t LORA_IQ_STANDARD = 0x00;        //  7     0   LoRa IQ setup: standard
			constexpr static uint8_t LORA_IQ_INVERTED = 0x01;        //  7     0                  inverted
			constexpr static uint8_t BPSK_RAMP_UP_TIME_NONE = 0x0000;      // 15     0   BPSK ramp-up time optimization: none
			constexpr static uint16_t BPSK_RAMP_UP_TIME_100_BPS = 0x370F;      // 15     0                                   for 100 bps
			constexpr static uint16_t BPSK_RAMP_UP_TIME_600_BPS = 0x092F;      // 15     0                                   for 600 bps
			constexpr static uint16_t BPSK_RAMP_DOWN_TIME_NONE = 0x0000;      // 15     0   BPSK ramp-down time optimization: none
			constexpr static uint16_t BPSK_RAMP_DOWN_TIME_100_BPS = 0x1D70;      // 15     0                                     for 100 bps
			constexpr static uint16_t BPSK_RAMP_DOWN_TIME_600_BPS = 0x04E1;      // 15     0                                     for 600 bps
			
			// CMD_SET_CAD_PARAMS
			constexpr static uint8_t CAD_ON_1_SYMB = 0x00;        //  7     0   number of symbols used for CAD: 1
			constexpr static uint8_t CAD_ON_2_SYMB = 0x01;        //  7     0                                   2
			constexpr static uint8_t CAD_ON_4_SYMB = 0x02;        //  7     0                                   4
			constexpr static uint8_t CAD_ON_8_SYMB = 0x03;        //  7     0                                   8
			constexpr static uint8_t CAD_ON_16_SYMB = 0x04;        //  7     0                                   16
			constexpr static uint8_t CAD_GOTO_STDBY = 0x00;        //  7     0   after CAD is done, always go to STDBY_RC mode
			constexpr static uint8_t CAD_GOTO_RX = 0x01;        //  7     0   after CAD is done, go to Rx mode if activity is detected
			constexpr static uint8_t CAD_PARAM_DEFAULT = 0xFF;        //  7     0   used by the CAD methods to specify default parameter value
			constexpr static uint8_t CAD_PARAM_DET_MIN = 10;          //  7     0   default detMin CAD parameter
			
			// CMD_GET_STATUS
			constexpr static uint8_t STATUS_MODE_STDBY_RC = 0b00100000;  //  6     4   current chip mode: STDBY_RC
			constexpr static uint8_t STATUS_MODE_STDBY_XOSC = 0b00110000;  //  6     4                      STDBY_XOSC
			constexpr static uint8_t STATUS_MODE_FS = 0b01000000;  //  6     4                      FS
			constexpr static uint8_t STATUS_MODE_RX = 0b01010000;  //  6     4                      RX
			constexpr static uint8_t STATUS_MODE_TX = 0b01100000;  //  6     4                      TX
			constexpr static uint8_t STATUS_DATA_AVAILABLE = 0b00000100;  //  3     1   command status: packet received and data can be retrieved
			constexpr static uint8_t STATUS_CMD_TIMEOUT = 0b00000110;  //  3     1                   SPI command timed out
			constexpr static uint8_t STATUS_CMD_INVALID = 0b00001000;  //  3     1                   invalid SPI command
			constexpr static uint8_t STATUS_CMD_FAILED = 0b00001010;  //  3     1                   SPI command failed to execute
			constexpr static uint8_t STATUS_TX_DONE = 0b00001100;  //  3     1                   packet transmission done
			constexpr static uint8_t STATUS_SPI_FAILED = 0b11111111;  //  7     0   SPI transaction failed
			
			// CMD_GET_PACKET_STATUS
			constexpr static uint8_t GFSK_RX_STATUS_PREAMBLE_ERR = 0b10000000;  //  7     7   GFSK Rx status: preamble error
			constexpr static uint8_t GFSK_RX_STATUS_SYNC_ERR = 0b01000000;  //  6     6                   sync word error
			constexpr static uint8_t GFSK_RX_STATUS_ADRS_ERR = 0b00100000;  //  5     5                   address error
			constexpr static uint8_t GFSK_RX_STATUS_CRC_ERR = 0b00010000;  //  4     4                   CRC error
			constexpr static uint8_t GFSK_RX_STATUS_LENGTH_ERR = 0b00001000;  //  3     3                   length error
			constexpr static uint8_t GFSK_RX_STATUS_ABORT_ERR = 0b00000100;  //  2     2                   abort error
			constexpr static uint8_t GFSK_RX_STATUS_PACKET_RECEIVED = 0b00000010;  //  2     2                   packet received
			constexpr static uint8_t GFSK_RX_STATUS_PACKET_SENT = 0b00000001;  //  2     2                   packet sent
			
			// CMD_GET_DEVICE_ERRORS
			constexpr static uint16_t PA_RAMP_ERR = 0b100000000; //  8     8   device errors: PA ramping failed
			constexpr static uint16_t PLL_LOCK_ERR = 0b001000000; //  6     6                  PLL failed to lock
			constexpr static uint16_t XOSC_START_ERR = 0b000100000; //  5     5                  crystal oscillator failed to start
			constexpr static uint16_t IMG_CALIB_ERR = 0b000010000; //  4     4                  image calibration failed
			constexpr static uint16_t ADC_CALIB_ERR = 0b000001000; //  3     3                  ADC calibration failed
			constexpr static uint16_t PLL_CALIB_ERR = 0b000000100; //  2     2                  PLL calibration failed
			constexpr static uint16_t RC13M_CALIB_ERR = 0b000000010; //  1     1                  RC13M calibration failed
			constexpr static uint16_t RC64K_CALIB_ERR = 0b000000001; //  0     0                  RC64K calibration failed
			
			// CMD_SET_LBT_SCAN_PARAMS + CMD_SET_SPECTR_SCAN_PARAMS
			constexpr static uint8_t SCAN_INTERVAL_7_68_US = 10;          //  7     0   RSSI reading interval: 7.68 us
			constexpr static uint8_t SCAN_INTERVAL_8_20_US = 11;          //  7     0                          8.20 us
			constexpr static uint8_t SCAN_INTERVAL_8_68_US = 12;          //  7     0                          8.68 us
			
			// -------------------------------- Registers --------------------------------
			
			// SX126X register map
			constexpr static uint16_t REG_BPSK_PACKET_PARAMS = 0x00F0;
			constexpr static uint16_t REG_RX_GAIN_RETENTION_0 = 0x029F; // SX1268 datasheet v1.1, section 9.6
			constexpr static uint16_t REG_RX_GAIN_RETENTION_1 = 0x02A0; // SX1268 datasheet v1.1, section 9.6
			constexpr static uint16_t REG_RX_GAIN_RETENTION_2 = 0x02A1; // SX1268 datasheet v1.1, section 9.6
			constexpr static uint16_t REG_VERSION_STRING = 0x0320;
			constexpr static uint16_t REG_HOPPING_ENABLE = 0x0385;
			constexpr static uint16_t REG_LR_FHSS_PACKET_LENGTH = 0x0386;
			constexpr static uint16_t REG_LR_FHSS_NUM_HOPPING_BLOCKS = 0x0387;
			constexpr static uint16_t REG_SPECTRAL_SCAN_RESULT = 0x0401;
			constexpr static uint16_t REG_DIOX_OUT_ENABLE = 0x0580;
			constexpr static uint16_t REG_DIOX_DRIVE_STRENGTH = 0x0582;
			constexpr static uint16_t REG_DIOX_IN_ENABLE = 0x0583;
			constexpr static uint16_t REG_DIOX_PULL_UP_CTRL = 0x0584;
			constexpr static uint16_t REG_DIOX_PULL_DOWN_CTRL = 0x0585;
			constexpr static uint16_t REG_TX_BITBANG_ENABLE_0 = 0x0587;
			constexpr static uint16_t REG_PATCH_UPDATE_ENABLE = 0x0610;
			constexpr static uint16_t REG_TX_BITBANG_ENABLE_1 = 0x0680;
			constexpr static uint16_t REG_GFSK_FIX_4 = 0x06AC;
			constexpr static uint16_t REG_WHITENING_INITIAL_MSB = 0x06B8;
			constexpr static uint16_t REG_WHITENING_INITIAL_LSB = 0x06B9;
			constexpr static uint16_t REG_RX_TX_PLD_LEN = 0x06BB;
			constexpr static uint16_t REG_CRC_INITIAL_MSB = 0x06BC;
			constexpr static uint16_t REG_CRC_INITIAL_LSB = 0x06BD;
			constexpr static uint16_t REG_CRC_POLYNOMIAL_MSB = 0x06BE;
			constexpr static uint16_t REG_CRC_POLYNOMIAL_LSB = 0x06BF;
			constexpr static uint16_t REG_SYNC_WORD_0 = 0x06C0;
			constexpr static uint16_t REG_SYNC_WORD_1 = 0x06C1;
			constexpr static uint16_t REG_SYNC_WORD_2 = 0x06C2;
			constexpr static uint16_t REG_SYNC_WORD_3 = 0x06C3;
			constexpr static uint16_t REG_SYNC_WORD_4 = 0x06C4;
			constexpr static uint16_t REG_SYNC_WORD_5 = 0x06C5;
			constexpr static uint16_t REG_SYNC_WORD_6 = 0x06C6;
			constexpr static uint16_t REG_SYNC_WORD_7 = 0x06C7;
			constexpr static uint16_t REG_NODE_ADDRESS = 0x06CD;
			constexpr static uint16_t REG_BROADCAST_ADDRESS = 0x06CE;
			constexpr static uint16_t REG_GFSK_FIX_1 = 0x06D1;
			constexpr static uint16_t REG_PAYLOAD_LENGTH = 0x0702;
			constexpr static uint16_t REG_PACKET_PARAMS = 0x0704;
			constexpr static uint16_t REG_LORA_SYNC_TIMEOUT = 0x0706;
			constexpr static uint16_t REG_IQ_CONFIG = 0x0736;
			constexpr static uint16_t REG_LORA_SYNC_WORD_MSB = 0x0740;
			constexpr static uint16_t REG_LORA_SYNC_WORD_LSB = 0x0741;
			constexpr static uint16_t REG_LORA_RX_CODING_RATE = 0x0749;
			constexpr static uint16_t REG_FREQ_ERROR_RX_CRC = 0x076B;
			constexpr static uint16_t REG_SPECTRAL_SCAN_STATUS = 0x07CD;
			constexpr static uint16_t REG_RX_ADDR_PTR = 0x0803;
			constexpr static uint16_t REG_RANDOM_NUMBER_0 = 0x0819;
			constexpr static uint16_t REG_RANDOM_NUMBER_1 = 0x081A;
			constexpr static uint16_t REG_RANDOM_NUMBER_2 = 0x081B;
			constexpr static uint16_t REG_RANDOM_NUMBER_3 = 0x081C;
			constexpr static uint16_t REG_TX_MODULATION = 0x0889; // SX1268 datasheet v1.1, section 15.1
			constexpr static uint16_t REG_RF_FREQUENCY_0 = 0x088B;
			constexpr static uint16_t REG_RF_FREQUENCY_1 = 0x088C;
			constexpr static uint16_t REG_RF_FREQUENCY_2 = 0x088D;
			constexpr static uint16_t REG_RF_FREQUENCY_3 = 0x088E;
			constexpr static uint16_t REG_RSSI_AVG_WINDOW = 0x089B;
			constexpr static uint16_t REG_RX_GAIN = 0x08AC;
			constexpr static uint16_t REG_GFSK_FIX_3 = 0x08B8;
			constexpr static uint16_t REG_TX_CLAMP_CONFIG = 0x08D8;
			constexpr static uint16_t REG_ANA_LNA = 0x08E2;
			constexpr static uint16_t REG_LNA_CAP_TUNE_N = 0x08E3;
			constexpr static uint16_t REG_LNA_CAP_TUNE_P = 0x08E4;
			constexpr static uint16_t REG_ANA_MIXER = 0x08E5;
			constexpr static uint16_t REG_OCP_CONFIGURATION = 0x08E7;
			constexpr static uint16_t REG_RTC_CTRL = 0x0902;
			constexpr static uint16_t REG_XTA_TRIM = 0x0911;
			constexpr static uint16_t REG_XTB_TRIM = 0x0912;
			constexpr static uint16_t REG_DIO3_OUT_VOLTAGE_CTRL = 0x0920;
			constexpr static uint16_t REG_EVENT_MASK = 0x0944;
			constexpr static uint16_t REG_PATCH_MEMORY_BASE = 0x8000;
			
			// SX126X register variables
			
			// REG_VERSION_STRING
			// Note: this should really be "2", however, it seems that all SX1262 devices report as SX1261
			constexpr static const char* VERSION_STRING = "SX1261";
			
			// REG_HOPPING_ENABLE                                          MSB   LSB   DESCRIPTION
			constexpr static uint8_t HOPPING_ENABLED = 0b00000001;  //  0     0   intra-packet hopping for LR-FHSS: enabled
			constexpr static uint8_t HOPPING_DISABLED = 0b00000000;  //  0     0                                     (disabled)
			
			// REG_LORA_SYNC_WORD_MSB + LSB
			constexpr static uint8_t SYNC_WORD_PUBLIC = 0x34;        // actually 0x3444  NOTE: The low nibbles in each byte (0x_4_4) are masked out since apparently, they're reserved.
			constexpr static uint8_t SYNC_WORD_PRIVATE = 0x12;        // actually 0x1424        You couldn't make this up if you tried.
			
			// REG_TX_BITBANG_ENABLE_1
			constexpr static uint8_t TX_BITBANG_1_DISABLED = 0b00000000;  //  6     4   Tx bitbang: disabled (default)
			constexpr static uint8_t TX_BITBANG_1_ENABLED = 0b00010000;  //  6     4               enabled
			
			// REG_TX_BITBANG_ENABLE_0
			constexpr static uint8_t TX_BITBANG_0_DISABLED = 0b00000000;  //  3     0   Tx bitbang: disabled (default)
			constexpr static uint8_t TX_BITBANG_0_ENABLED = 0b00001100;  //  3     0               enabled
			
			// REG_DIOX_OUT_ENABLE
			constexpr static uint8_t DIO1_OUT_DISABLED = 0b00000010;  //  1     1   DIO1 output: disabled
			constexpr static uint8_t DIO1_OUT_ENABLED = 0b00000000;  //  1     1                enabled
			constexpr static uint8_t DIO2_OUT_DISABLED = 0b00000100;  //  2     2   DIO2 output: disabled
			constexpr static uint8_t DIO2_OUT_ENABLED = 0b00000000;  //  2     2                enabled
			constexpr static uint8_t DIO3_OUT_DISABLED = 0b00001000;  //  3     3   DIO3 output: disabled
			constexpr static uint8_t DIO3_OUT_ENABLED = 0b00000000;  //  3     3                enabled
			
			// REG_DIOX_IN_ENABLE
			constexpr static uint8_t DIO1_IN_DISABLED = 0b00000000;  //  1     1   DIO1 input: disabled
			constexpr static uint8_t DIO1_IN_ENABLED = 0b00000010;  //  1     1               enabled
			constexpr static uint8_t DIO2_IN_DISABLED = 0b00000000;  //  2     2   DIO2 input: disabled
			constexpr static uint8_t DIO2_IN_ENABLED = 0b00000100;  //  2     2               enabled
			constexpr static uint8_t DIO3_IN_DISABLED = 0b00000000;  //  3     3   DIO3 input: disabled
			constexpr static uint8_t DIO3_IN_ENABLED = 0b00001000;  //  3     3               enabled
			
			// REG_RX_GAIN
			constexpr static uint8_t RX_GAIN_BOOSTED = 0x96;        //  7     0   Rx gain: boosted
			constexpr static uint8_t RX_GAIN_POWER_SAVING = 0x94;        //  7     0            power saving
			constexpr static uint8_t RX_GAIN_SPECTRAL_SCAN = 0xCB;        //  7     0            spectral scan
			
			// REG_PATCH_UPDATE_ENABLE
			constexpr static uint8_t PATCH_UPDATE_DISABLED = 0b00000000;  //  4     4   patch update: disabled
			constexpr static uint8_t PATCH_UPDATE_ENABLED = 0b00010000;  //  4     4                 enabled
			
			// REG_SPECTRAL_SCAN_STATUS
			constexpr static uint8_t SPECTRAL_SCAN_NONE = 0x00;        //  7     0   spectral scan status: none
			constexpr static uint8_t SPECTRAL_SCAN_ONGOING = 0x0F;        //  7     0                         ongoing
			constexpr static uint8_t SPECTRAL_SCAN_ABORTED = 0xF0;        //  7     0                         aborted
			constexpr static uint8_t SPECTRAL_SCAN_COMPLETED = 0xFF;        //  7     0                         completed
			
			// REG_RSSI_AVG_WINDOW
			constexpr static uint8_t SPECTRAL_SCAN_WINDOW_DEFAULT = (0x05
				<< 2); //  7     0   default RSSI average window
			
			// REG_ANA_LNA
			constexpr static uint8_t LNA_RNG_DISABLED = 0b00000001;  //  0     0   random number: disabled
			constexpr static uint8_t LNA_RNG_ENABLED = 0b00000000;  //  0     0                  enabled
			
			// REG_ANA_MIXER
			constexpr static uint8_t MIXER_RNG_DISABLED = 0b00000001;  //  7     7   random number: disabled
			constexpr static uint8_t MIXER_RNG_ENABLED = 0b00000000;  //  7     7                  enabled
			
			// size of the spectral scan result
			constexpr static uint8_t SPECTRAL_SCAN_RES_SIZE = 33;
			
			// LR-FHSS configuration
			constexpr static uint8_t LR_FHSS_CR_5_6 = (0x00UL << 0);   //  7     0     LR FHSS coding rate: 5/6
			constexpr static uint8_t LR_FHSS_CR_2_3 = (0x01UL << 0);   //  7     0                          2/3
			constexpr static uint8_t LR_FHSS_CR_1_2 = (0x02UL << 0);   //  7     0                          1/2
			constexpr static uint8_t LR_FHSS_CR_1_3 = (0x03UL << 0);   //  7     0                          1/3
			constexpr static uint8_t LR_FHSS_MOD_TYPE_GMSK = (0x00UL << 0);   //  7     0     LR FHSS modulation: GMSK
			constexpr static uint8_t LR_FHSS_GRID_STEP_FCC = (0x00UL
				<< 0);   //  7     0     LR FHSS step size: 25.390625 kHz (FCC)
			constexpr static uint8_t LR_FHSS_GRID_STEP_NON_FCC = (0x01UL
				<< 0);   //  7     0                        3.90625 kHz (non-FCC)
			constexpr static uint8_t LR_FHSS_HOPPING_DISABLED = (0x00UL
				<< 0);   //  7     0     LR FHSS hopping: disabled
			constexpr static uint8_t LR_FHSS_HOPPING_ENABLED = (0x01UL << 0);   //  7     0                      enabled
			constexpr static uint8_t LR_FHSS_BW_39_06 = (0x00UL << 0);   //  7     0     LR FHSS bandwidth: 39.06 kHz
			constexpr static uint8_t LR_FHSS_BW_85_94 = (0x01UL << 0);   //  7     0                        85.94 kHz
			constexpr static uint8_t LR_FHSS_BW_136_72 = (0x02UL << 0);   //  7     0                        136.72 kHz
			constexpr static uint8_t LR_FHSS_BW_183_59 = (0x03UL << 0);   //  7     0                        183.59 kHz
			constexpr static uint8_t LR_FHSS_BW_335_94 = (0x04UL << 0);   //  7     0                        335.94 kHz
			constexpr static uint8_t LR_FHSS_BW_386_72 = (0x05UL << 0);   //  7     0                        386.72 kHz
			constexpr static uint8_t LR_FHSS_BW_722_66 = (0x06UL << 0);   //  7     0                        722.66 kHz
			constexpr static uint8_t LR_FHSS_BW_773_44 = (0x07UL << 0);   //  7     0                        773.44 kHz
			constexpr static uint8_t LR_FHSS_BW_1523_4 = (0x08UL << 0);   //  7     0                        1523.4 kHz
			constexpr static uint8_t LR_FHSS_BW_1574_2 = (0x09UL << 0);   //  7     0                        1574.2 kHz
	};
}