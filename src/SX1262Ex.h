#pragma once

#include "SX1262.h"

#include <cmath>

#include <freertos/semphr.h>

namespace YOBA {
	class SX1262Ex : public SX1262 {
		public:
			SX1262Error setup(
				spi_host_device_t SPIHostDevice,
				uint32_t SPIFrequencyHz,
				
				gpio_num_t SSPin,
				gpio_num_t RSTPin,
				gpio_num_t busyPin,
				gpio_num_t DIO1Pin,
				
				uint16_t frequencyMHz,
				float bandwidthKHz,
				uint8_t spreadingFactor,
				uint8_t codingRate,
				uint8_t syncWord,
				int8_t powerDBm,
				uint16_t preambleLength,
				bool useLDORegulator = false
			) {
				auto error = SX1262::setup(
					SPIHostDevice,
					SPIFrequencyHz,
					
					SSPin,
					RSTPin,
					busyPin
				);
				
				if (error != SX1262Error::none)
					return error;
				
				_DIO1Pin = DIO1Pin;
				
				// BW in kHz and SF are required in order to calculate LDRO for setModulationParams
				// set the defaults, this will get overwritten later anyway
				_bandwidthKHz = 500.0;
				_spreadingFactor = 9;
				
				// initialize configuration variables (will be overwritten during public settings configuration)
				_bandwidth = LORA_BW_500_0;  // initialized to 500 kHz, since lower values will interfere with LLCC68
				_codingRate = LORA_CR_4_7;
				_ldrOptimize = false;
				_crcType = LORA_CRC_ON;
				_preambleLength = preambleLength;
				_tcxoDelay = 0;
				_headerType = LORA_HEADER_EXPLICIT;
				
				// -------------------------------- GPIO input --------------------------------
				
				// DIO1
				gpio_config_t g = {};
				g.pin_bit_mask = 1ULL << _DIO1Pin;
				g.mode = GPIO_MODE_INPUT;
				g.pull_up_en = GPIO_PULLUP_ENABLE;
				g.pull_down_en = GPIO_PULLDOWN_DISABLE;
				g.intr_type = GPIO_INTR_POSEDGE;
				gpio_config(&g);
				
				// -------------------------------- Interrupts --------------------------------
				
				_DIO1PinSemaphore = xSemaphoreCreateBinary();
				
				gpio_install_isr_service(0);
				gpio_isr_handler_add(_DIO1Pin, onDIO1PinInterrupt, this);
				
				// -------------------------------- Initialization --------------------------------
				
				error = reset();
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
				
				// Set some CAD parameters - will be overwritten when calling CAD anyway
				error = setCADParams(_spreadingFactor);
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
				waitForBusyPin(1'000);
				
				error = setRegulatorMode(useLDORegulator ? REGULATOR_LDO : REGULATOR_DC_DC);
				if (error != SX1262Error::none)
					return error;
				
				error = setCodingRate(codingRate);
				if (error != SX1262Error::none)
					return error;
				
				error = setSyncWord(syncWord);
				if (error != SX1262Error::none)
					return error;
				
				error = setPreambleLength(preambleLength);
				if (error != SX1262Error::none)
					return error;
				
				error = setCurrentLimit(60.0);
				if (error != SX1262Error::none)
					return error;
				
				error = setDio2AsRfSwitch(true);
				if (error != SX1262Error::none)
					return error;
				
				error = setCRC(2);
				if (error != SX1262Error::none)
					return error;
				
				error = invertIQ(false);
				if (error != SX1262Error::none)
					return error;
				
				error = setSpreadingFactor(spreadingFactor);
				if (error != SX1262Error::none)
					return error;
				
				error = setBandwidth(bandwidthKHz);
				if (error != SX1262Error::none)
					return error;
				
				error = setRFFrequency(frequencyMHz);
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
			
			/*!
			  \brief Sets LoRa coding rate denominator. Allowed values range from 4 to 8. Note that a value of 4 means no coding,
			  is undocumented and not recommended without your own FEC.
			  
			  \param codingRate LoRa coding rate denominator to be set.
			  \param longInterleave Enable long interleaver when set to true.
			  Note that with long interleaver enabled, CR 4/7 is not possible, there are packet length restrictions,
			  and it is not compatible with SX127x radios.
			  
			  \returns \ref status_codes
			*/
			SX1262Error setCodingRate(uint8_t codingRate, bool longInterleave = false) {
				const auto error = checkForLoRaPacketType();
				
				if (error != SX1262Error::none)
					return error;
				
				if (codingRate < 4 || codingRate > 8) {
					ESP_LOGE(_logTag, "failed to set coding rate: value %d is out of range [4; 8]", codingRate);
					return SX1262Error::invalidArgument;
				}
				
				if (longInterleave) {
					switch (codingRate) {
						case 4:
							_codingRate = 0;
							break;
						case 5:
						case 6:
							_codingRate = codingRate;
							break;
						case 8:
							_codingRate = codingRate - 1;
							break;
						default:
							ESP_LOGE(_logTag, "failed to set coding rate: value %d is invalid", codingRate);
							
							return SX1262Error::invalidArgument;
					}
				} else {
					_codingRate = codingRate - 4;
				}
				
				updateLDROptimize(_ldrOptimize);
				
				return updateModulationParams();
			}
			
			SX1262Error setBandwidth(float bandwidth) {
				const auto error = checkForLoRaPacketType();
				
				if (error != SX1262Error::none)
					return error;
				
				// Ensure byte conversion doesn't overflow
				if (bandwidth < 0 || bandwidth > 510) {
					ESP_LOGE(_logTag, "failed to set bandwidth: value %f is out of range [0; 510]", bandwidth);
					return SX1262Error::invalidArgument;
				}
				
				// check allowed bandwidth values
				auto bandWidthDiv2 = static_cast<uint8_t>((float) bandwidth / 2.f + 0.01f);
				
				switch (bandWidthDiv2) {
					case 3: // 7.8:
						_bandwidth = LORA_BW_7_8;
						break;
					case 5: // 10.4:
						_bandwidth = LORA_BW_10_4;
						break;
					case 7: // 15.6:
						_bandwidth = LORA_BW_15_6;
						break;
					case 10: // 20.8:
						_bandwidth = LORA_BW_20_8;
						break;
					case 15: // 31.25:
						_bandwidth = LORA_BW_31_25;
						break;
					case 20: // 41.7:
						_bandwidth = LORA_BW_41_7;
						break;
					case 31: // 62.5:
						_bandwidth = LORA_BW_62_5;
						break;
					case 62: // 125.0:
						_bandwidth = LORA_BW_125_0;
						break;
					case 125: // 250.0
						_bandwidth = LORA_BW_250_0;
						break;
					case 250: // 500.0
						_bandwidth = LORA_BW_500_0;
						break;
					default: {
						ESP_LOGE(_logTag, "failed to set bandwidth: value %f is invalid ", bandwidth);
						return SX1262Error::invalidArgument;
					}
				}
				
				_bandwidthKHz = bandwidth;
				
				updateLDROptimize(_ldrOptimize);
				
				return updateModulationParams();
			}
			
			SX1262Error setRFFrequency(uint16_t frequencyMHz) {
				// check if we need to recalibrate image
				if ((std::abs(static_cast<int32_t>(frequencyMHz) - static_cast<int32_t>(_frequencyMHz)) >= CAL_IMG_FREQ_TRIG_MHZ)) {
					const auto error = calibrateImage(frequencyMHz);
					
					if (error != SX1262Error::none)
						return error;
				}
				
				return SX1262::setRFFrequency(frequencyMHz);
			}
			
			/*!
  \brief Sets preamble length for LoRa or FSK modem. Allowed values range from 1 to 65535.
  \param preambleLength Preamble length to be set in symbols (LoRa) or bits (FSK).
  NOTE: In FSK mode, sync word length limits the preamble detector length
  (the number of preamble bits that must be detected to start receiving packet).
  For details, see the note in SX1261 datasheet, Rev 2.1, section 6.2.2.1, page 45.
  Preamble detector length is adjusted automatically each time this method is called.
  \returns \ref status_codes
*/
			SX1262Error setPreambleLength(uint16_t preambleLength) {
				const auto error = checkForLoRaPacketType();
				
				if (error != SX1262Error::none)
					return error;
				
				_preambleLength = preambleLength;
				
				return updatePacketParams();
			}
			
			/*!
			  \brief Sets CRC configuration.
			  \param len CRC length in bytes, Allowed values are 1 or 2, set to 0 to disable CRC.
			  \param initial Initial CRC value. FSK only. Defaults to 0x1D0F (CCIT CRC).
			  \param polynomial Polynomial for CRC calculation. FSK only. Defaults to 0x1021 (CCIT CRC).
			  \param inverted Invert CRC bytes. FSK only. Defaults to true (CCIT CRC).
			  \returns \ref status_codes
			*/
			SX1262Error setCRC(uint8_t len, uint16_t initial = 0x1D0F, uint16_t polynomial = 0x1021, bool inverted = true) {
				const auto error = checkForLoRaPacketType();
				
				if (error != SX1262Error::none)
					return error;
				
				// LoRa CRC doesn't allow to set CRC polynomial, initial value, or inversion
				
				if (len) {
					_crcType = LORA_CRC_ON;
				} else {
					_crcType = LORA_CRC_OFF;
				}
				
				return updatePacketParams();
			}
			
			/*!
			 \brief Enable/disable inversion of the I and Q signals
			 \param enable IQ inversion enabled (true) or disabled (false);
			 \returns \ref status_codes
		   */
			SX1262Error invertIQ(bool enable) {
				const auto error = checkForLoRaPacketType();
				
				if (error != SX1262Error::none)
					return error;
				
				if (enable) {
					_invertIQ = LORA_IQ_INVERTED;
				} else {
					_invertIQ = LORA_IQ_STANDARD;
				}
				
				return updatePacketParams();
			}
			
			/*!
			 \brief Sets LoRa spreading factor. Allowed values range from 5 to 12.
			 \param sf LoRa spreading factor to be set.
			 \returns \ref status_codes
		   */
			SX1262Error setSpreadingFactor(uint8_t sf) {
				const auto error = checkForLoRaPacketType();
				
				if (error != SX1262Error::none)
					return error;
				
				if (sf < 5 || sf > 12) {
					ESP_LOGE(_logTag, "failed to set spreading factor: value %d is out of range [5; 12]", sf);
					return SX1262Error::invalidArgument;
				}
				
				_spreadingFactor = sf;
				
				return updateModulationParams();
			}
			
			SX1262Error finishTransmit() {
//				if (!setStandby())
//					return false;
				
				auto error = clearIRQStatus();
				if (error != SX1262Error::none)
					return error;
				
				return SX1262Error::none;
			}
			
			SX1262Error waitForDIO1Semaphore(uint32_t timeoutUs) {
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
			
			SX1262Error transmit(const uint8_t* data, uint8_t length, uint32_t timeoutUs = 0) {
//				if (!setStandby())
//					return false;
				
				// check packet length
				if (_codingRate > LORA_CR_4_8) {
					// Long Interleaver needs at least 8 bytes
					if (length < 8) {
						ESP_LOGE(_logTag, "failed to transmit: packet is too short");
						return SX1262Error::invalidArgument;
					}
					
					// Long Interleaver supports up to 253 bytes if CRC is enabled
					if (_crcType == LORA_CRC_ON && (length > IMPLICIT_PACKET_LENGTH - 2)) {
						ESP_LOGE(_logTag, "failed to transmit: packet is too long");
						return SX1262Error::invalidArgument;
					}
				}
				
				auto error = updatePacketParams(length);
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
				
				error = writeBuffer(data, length);
				if (error != SX1262Error::none)
					return error;
				
				error = clearIRQStatus();
				if (error != SX1262Error::none)
					return error;
				
				// Important shit
				error = fixTXModulationBeforeTransmission();
				if (error != SX1262Error::none)
					return error;
				
				// LET'S FUCKING MOOOOVE
				error = setTX(timeoutUs);
				if (error != SX1262Error::none)
					return error;
				
				error = waitForDIO1Semaphore(timeoutUs);
				
				if (error != SX1262Error::none) {
					ESP_LOGE(_logTag, "failed to transmit: dio1 timeout reached");
					
					finishTransmit();
					
					return SX1262Error::timeout;
				}
				
				uint16_t IRQStatus = 0;
				
				error = getIRQStatus(IRQStatus);
				if (error != SX1262Error::none)
					return error;
				
				error = finishTransmit();
				if (error != SX1262Error::none)
					return error;
				
				if (IRQStatus & IRQ_TIMEOUT) {
//					ESP_LOGE(_logTag, "failed to transmit: IRQ timeout reached");
					
					return SX1262Error::timeout;
				}
				
				return SX1262Error::none;
			}
			
			SX1262Error fixImplicitTimeout() {
				// fixes timeout in implicit header mode
				// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.3 for details
				
				//check if we're in implicit LoRa mode
				uint8_t packetType = 0;
				
				auto error = getPacketType(packetType);
				
				if (error != SX1262Error::none)
					return error;
				
				if (_headerType != LORA_HEADER_IMPLICIT || packetType != PACKET_TYPE_LORA) {
					// not in the correct mode, nothing to do here
					return SX1262Error::none;
				}
				
				// stop RTC counter
				uint8_t rtcStop = 0x00;
				
				error = SPIWriteRegister(REG_RTC_CTRL, &rtcStop, 1);
				if (error != SX1262Error::none)
					return error;
				
				// read currently active event
				uint8_t rtcEvent = 0;
				
				error = SPIWriteRegister(REG_EVENT_MASK, &rtcEvent, 1);
				if (error != SX1262Error::none)
					return error;
				
				// clear events
				rtcEvent |= 0x02;
				
				error = SPIWriteRegister(REG_EVENT_MASK, &rtcEvent, 1);
				if (error != SX1262Error::none)
					return error;
				
				return SX1262Error::none;
			}
			
			SX1262Error getPacketLength(uint8_t& length, uint8_t& offset) {
				uint8_t packetType = 0;
				
				auto error = getPacketType(packetType);
				if (error != SX1262Error::none)
					return error;
				
				// in implicit mode, return the cached value if the offset was not requested
				if ((packetType == PACKET_TYPE_LORA) && (_headerType == LORA_HEADER_IMPLICIT) && (!offset)) {
					length = IMPLICIT_PACKET_LENGTH;
					offset = 0;
					
					return SX1262Error::none;
				}
				
				// if offset was requested, or in explicit mode, we always have to perform the SPI transaction
				uint8_t data[] = {
					0,
					0
				};
				
				error = SPIReadCommand(CMD_GET_RX_BUFFER_STATUS, data, 2);
				if (error != SX1262Error::none)
					return error;
				
				offset = data[1];
				length = data[0];
				
				return SX1262Error::none;
			}
			
			SX1262Error finishReceive() {
//				if (!setStandby())
//					return false;
				
				// try to fix timeout error in implicit header mode
				// check for modem type and header mode is done in fixImplicitTimeout()
				const auto error = fixImplicitTimeout();
				
				if (error != SX1262Error::none)
					return error;
				
				// clear interrupt flags
				return clearIRQStatus();
			}
			
			SX1262Error receive(uint8_t* data, uint8_t& length, uint32_t timeoutUs = 0) {
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
				
				error = updatePacketParams();
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
				
				error = getPacketLength(length, offset);
				if (error != SX1262Error::none)
					return error;

//				ESP_LOGI(_logTag, "receive() length: %d, offset: %d", length, offset);
				
				// read packet data starting at offset
				error = readBuffer(data, length, offset);
				if (error != SX1262Error::none)
					return error;
				
				return SX1262Error::none;
			}
		
		private:
			constexpr static const char* _logTag = "SX1262Ex";
			
			using SX1262::setup;
			
			gpio_num_t _DIO1Pin = GPIO_NUM_NC;
			
			uint16_t _frequencyMHz = 0;
			uint8_t _spreadingFactor = 0;
			uint8_t _codingRate = 0;
			uint8_t _bandwidth = 0;
			float _bandwidthKHz = 0;
			uint16_t _preambleLength = 0;
			uint8_t _crcType = 0;
			uint8_t _headerType = 0;
			uint32_t _tcxoDelay = 0;
			uint8_t _invertIQ = LORA_IQ_STANDARD;
			
			// LoRa low data rate optimization
			bool _ldrOptimize = false;
			bool _ldrOptimizeAuto = false;
			
			SemaphoreHandle_t _DIO1PinSemaphore;
			
			SX1262Error updateModulationParams() {
				return setModulationParams(
					_spreadingFactor,
					_bandwidth,
					_codingRate,
					_ldrOptimize
				);
			}
			
			SX1262Error updatePacketParams(uint8_t length = IMPLICIT_PACKET_LENGTH) {
				return setPacketParams(
					_preambleLength,
					_headerType,
					length,
					_crcType,
					_invertIQ
				);
			}
			
			void updateLDROptimize(bool value) {
				// calculate symbol length and enable low data rate optimization, if autoconfiguration is enabled
				if (_ldrOptimizeAuto) {
					float symbolLength = (float) (uint32_t(1) << _spreadingFactor) / (float) _bandwidthKHz;
					
					if (symbolLength >= 16.0f) {
						_ldrOptimize = LORA_LOW_DATA_RATE_OPTIMIZE_ON;
					} else {
						_ldrOptimize = LORA_LOW_DATA_RATE_OPTIMIZE_OFF;
					}
				}
				else {
					_ldrOptimize = value;
				}
			}
			
			SX1262Error fixTXModulationBeforeTransmission() {
				// fix tx modulation for 500 kHz LoRa
				// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.1 for details
				
				uint8_t txModulation = 0;
				
				auto error = SPIReadRegister(REG_TX_MODULATION, &txModulation, 1);
				if (error != SX1262Error::none)
					return error;
				
				uint8_t packetType = 0;
				
				error = getPacketType(packetType);
				if (error != SX1262Error::none)
					return error;
				
				// fix the value for LoRa with 500 kHz bandwidth
				if (packetType == PACKET_TYPE_LORA && std::fabsf(_bandwidthKHz - 500.0f) <= 0.001f) {
					txModulation &= 0xFB;
				}
				else {
					txModulation |= 0x04;
				}
				
				return SPIWriteRegister(REG_TX_MODULATION, &txModulation, 1);
			}
			
			bool getDIO1PinLevel() {
				return gpio_get_level(_DIO1Pin);
			}
			
			IRAM_ATTR void onDIO1PinInterrupt() {
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				
				xSemaphoreGiveFromISR(_DIO1PinSemaphore, &xHigherPriorityTaskWoken);
				
				if (xHigherPriorityTaskWoken) {
					portYIELD_FROM_ISR();
				}
			}
			
			IRAM_ATTR static void onDIO1PinInterrupt(void* arg) {
				reinterpret_cast<SX1262Ex*>(arg)->onDIO1PinInterrupt();
			}
	};
}