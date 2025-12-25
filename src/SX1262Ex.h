#pragma once

#include "SX1262.h"

#include <cmath>

#include <freertos/semphr.h>

namespace YOBA {
	class SX1262Ex : public SX1262 {
		public:
			bool setup(
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
				if (!SX1262::setup(
					SPIHostDevice,
					SPIFrequencyHz,
					
					SSPin,
					RSTPin,
					busyPin
				))
					return false;
				
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
				
				if (!reset())
					return false;
				
				if (!validateChip())
					return false;
				
				// TCXO configuration should be here
				//				if (!XTAL && tcxoVoltage > 0.0f) {
				//					setTCXO(tcxoVoltage);
				//				}
				
				if (!setBufferBaseAddress(0x00, 0x00))
					return false;
				
				if (!setPacketType(PACKET_TYPE_LORA))
					return false;
				
				if (!setRXTXFallbackMode(RX_TX_FALLBACK_MODE_STDBY_RC))
					return false;
				
				// Set some CAD parameters - will be overwritten when calling CAD anyway
				if (!setCADParams(_spreadingFactor))
					return false;
				
				if (!clearIRQStatus())
					return false;
				
				if (!setDIOIRQParams())
					return false;
				
				if (!calibrate(CALIBRATE_ALL))
					return false;
				
				// Wait for calibration completion end. Normally this should take 3.5 ms
				waitForBusyPin(1'000);
				
				if (!setRegulatorMode(useLDORegulator ? REGULATOR_LDO : REGULATOR_DC_DC))
					return false;
				
				if (!setCodingRate(codingRate))
					return false;
				
				if (!setSyncWord(syncWord))
					return false;
				
				if (!setPreambleLength(preambleLength))
					return false;
				
				if (!setCurrentLimit(60.0))
					return false;
				
				if (!setDio2AsRfSwitch(true))
					return false;
				
				if (!setCRC(2))
					return false;
				
				if (!invertIQ(false))
					return false;
				
				if (!setSpreadingFactor(spreadingFactor))
					return false;
				
				if (!setBandwidth(bandwidthKHz))
					return false;
				
				if (!setRFFrequency(frequencyMHz))
					return false;
				
				if (!setTXClampConfig(true))
					return false;
				
				if (!setOutputPower(powerDBm))
					return false;
				
				return true;
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
			bool setCodingRate(uint8_t codingRate, bool longInterleave = false) {
				if (!checkForLoRaPacketType())
					return false;
				
				if (codingRate < 4 || codingRate > 8) {
					ESP_LOGE(_logTag, "failed to set coding rate: value %d is out of range [4; 8]", codingRate);
					return false;
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
							
							return false;
					}
				} else {
					_codingRate = codingRate - 4;
				}
				
				updateLDROptimize(_ldrOptimize);
				
				return updateModulationParams();
			}
			
			bool setBandwidth(float bandwidth) {
				if (!checkForLoRaPacketType())
					return false;
				
				// Ensure byte conversion doesn't overflow
				if (bandwidth < 0 || bandwidth > 510) {
					ESP_LOGE(_logTag, "failed to set bandwidth: value %f is out of range [0; 510]", bandwidth);
					return false;
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
						return false;
					}
				}
				
				_bandwidthKHz = bandwidth;
				
				updateLDROptimize(_ldrOptimize);
				
				return updateModulationParams();
			}
			
			bool setRFFrequency(uint16_t frequencyMHz) {
				// check if we need to recalibrate image
				if ((std::abs(static_cast<int32_t>(frequencyMHz) - static_cast<int32_t>(_frequencyMHz)) >= CAL_IMG_FREQ_TRIG_MHZ)) {
					if (!calibrateImage(frequencyMHz)) {
						return false;
					}
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
			bool setPreambleLength(uint16_t preambleLength) {
				if (!checkForLoRaPacketType())
					return false;
				
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
			bool setCRC(uint8_t len, uint16_t initial = 0x1D0F, uint16_t polynomial = 0x1021, bool inverted = true) {
				if (!checkForLoRaPacketType())
					return false;
				
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
			bool invertIQ(bool enable) {
				if (!checkForLoRaPacketType())
					return false;
				
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
			bool setSpreadingFactor(uint8_t sf) {
				if (!checkForLoRaPacketType())
					return false;
				
				if (sf < 5 || sf > 12) {
					ESP_LOGE(_logTag, "failed to set spreading factor: value %d is out of range [5; 12]", sf);
					return false;
				}
				
				_spreadingFactor = sf;
				
				return updateModulationParams();
			}
			
			bool finishTransmit() {
				if (!setStandby())
					return false;
				
				if (!clearIRQStatus())
					return false;
				
				return true;
			}
			
			bool waitForDIO1Semaphore(uint32_t timeoutUs) {
				return
					// Already in high
					getDIO1PinLevel()
					// Wait for high
					|| xSemaphoreTake(
						   _DIO1PinSemaphore,
						   timeoutUs == 0
						   // Zero timeout means infinite waiting
						   ? portMAX_DELAY
						   // FreeRTOS tasks can handle at least portTICK_PERIOD_MS, also adding 100 ms for пропёрживание
						   : pdMS_TO_TICKS(std::max(timeoutUs / 1000 + 100, portTICK_PERIOD_MS))
					   ) == pdTRUE;
			}
			
			bool transmit(const uint8_t* data, uint8_t length, uint32_t timeoutUs = 0) {
				// set mode to standby
				if (!setStandby()) {
					ESP_LOGE(_logTag, "failed to transmit: unable to enter standby mode");
					return false;
				}
				
				// check packet length
				if (_codingRate > LORA_CR_4_8) {
					// Long Interleaver needs at least 8 bytes
					if (length < 8) {
						ESP_LOGE(_logTag, "failed to transmit: packet is too short");
						return false;
					}
					
					// Long Interleaver supports up to 253 bytes if CRC is enabled
					if (_crcType == LORA_CRC_ON && (length > IMPLICIT_PACKET_LENGTH - 2)) {
						ESP_LOGE(_logTag, "failed to transmit: packet is too long");
						return false;
					}
				}
				
				if (!updatePacketParams(length))
					return false;
				
				uint16_t IRQMask = IRQ_TX_DONE;
				
				if (timeoutUs > 0)
					IRQMask |= IRQ_TIMEOUT;
				
				if (!setDIOIRQParams(IRQMask, IRQMask))
					return false;
				
				if (!setBufferBaseAddress())
					return false;
				
				if (!writeBuffer(data, length))
					return false;
				
				if (!clearIRQStatus())
					return false;
				
				// Important shit
				if (!fixTXModulationBeforeTransmission())
					return false;
				
				// LET'S FUCKING MOOOOVE
				if (!setTX(timeoutUs))
					return false;
				
				if (!waitForDIO1Semaphore(timeoutUs)) {
					ESP_LOGE(_logTag, "failed to transmit: semaphore timeout reached");
					
					return finishTransmit();
				}
				
				uint16_t IRQStatus = 0;
				
				if (!getIRQStatus(IRQStatus))
					return false;
				
				if (!finishTransmit())
					return false;
				
				if (IRQStatus & IRQ_TIMEOUT) {
					ESP_LOGE(_logTag, "failed to transmit: IRQ timeout reached");
					
					return false;
				}
				
				return true;
			}
			
			bool fixImplicitTimeout() {
				// fixes timeout in implicit header mode
				// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.3 for details
				
				//check if we're in implicit LoRa mode
				uint8_t packetType = 0;
				
				if (!getPacketType(packetType))
					return false;
				
				if (_headerType != LORA_HEADER_IMPLICIT || packetType != PACKET_TYPE_LORA) {
					// not in the correct mode, nothing to do here
					return true;
				}
				
				// stop RTC counter
				uint8_t rtcStop = 0x00;
				
				if (!SPIWriteRegister(REG_RTC_CTRL, &rtcStop, 1))
					return false;
				
				// read currently active event
				uint8_t rtcEvent = 0;
				
				if (!SPIWriteRegister(REG_EVENT_MASK, &rtcEvent, 1))
					return false;
				
				// clear events
				rtcEvent |= 0x02;
				if (!SPIWriteRegister(REG_EVENT_MASK, &rtcEvent, 1))
					return false;
				
				return true;
			}
			
			bool getPacketLength(uint8_t& length, uint8_t& offset) {
				uint8_t packetType = 0;
				
				if (!getPacketType(packetType))
					return false;
				
				// in implicit mode, return the cached value if the offset was not requested
				if ((packetType == PACKET_TYPE_LORA) && (_headerType == LORA_HEADER_IMPLICIT) && (!offset)) {
					length = IMPLICIT_PACKET_LENGTH;
					offset = 0;
					
					return true;
				}
				
				// if offset was requested, or in explicit mode, we always have to perform the SPI transaction
				uint8_t data[] = {
					0,
					0
				};
				
				if (!SPIReadCommand(CMD_GET_RX_BUFFER_STATUS, data, 2))
					return false;
				
				offset = data[1];
				length = data[0];
				
				return true;
			}
			
			bool finishReceive() {
				if (!setStandby())
					return false;
				
				// try to fix timeout error in implicit header mode
				// check for modem type and header mode is done in fixImplicitTimeout()
				if (!fixImplicitTimeout())
					return false;
				
				// clear interrupt flags
				return clearIRQStatus();
			}
			
			bool receive(uint8_t* data, uint8_t& length, uint32_t timeoutUs = 0) {
				if (!setStandby())
					return false;
				
				uint16_t IRQMask = IRQ_RX_DONE;
				
				if (timeoutUs > 0)
					IRQMask |= IRQ_TIMEOUT;
				
				if (!setDIOIRQParams(IRQMask, IRQMask))
					return false;
				
				if (!clearIRQStatus())
					return false;
				
				if (!setBufferBaseAddress())
					return false;
				
				if (!updatePacketParams())
					return false;
				
				// LET'S FUCKING MOOOOVE
				if (!setRX(timeoutUs))
					return false;
				
				if (!waitForDIO1Semaphore(timeoutUs)) {
					ESP_LOGE(_logTag, "failed to receive: semaphore timeout reached");
					finishReceive();
					
					return false;
				}
				
				uint16_t IRQStatus = 0;
				
				if (!getIRQStatus(IRQStatus))
					return false;
				
				if (!finishReceive())
					return false;
				
				if (IRQStatus & IRQ_TIMEOUT) {
					ESP_LOGE(_logTag, "failed to receive: IRQ timeout reached");
					
					return false;
				}
				
				// check integrity CRC
				// Report CRC mismatch when there's a payload CRC error, or a header error and no valid header (to avoid false alarm from previous packet)
				if ((IRQStatus & IRQ_CRC_ERR) || ((IRQStatus & IRQ_HEADER_ERR) && !(IRQStatus & IRQ_HEADER_VALID))) {
					ESP_LOGE(_logTag, "failed to receive: CRC mismatch");
					
					return false;
				}
				
				// get packet length and Rx buffer offset
				uint8_t offset = 0;
				
				if (!getPacketLength(length, offset))
					return false;

//				ESP_LOGI(_logTag, "receive() length: %d, offset: %d", length, offset);
				
				// read packet data starting at offset
				if (!readBuffer(data, length, offset))
					return false;
				
				return true;
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
			bool _ldrOptimizeAuto = true;
			
			SemaphoreHandle_t _DIO1PinSemaphore;
			
			bool updateModulationParams() {
				return setModulationParams(
					_spreadingFactor,
					_bandwidth,
					_codingRate,
					_ldrOptimize
				);
			}
			
			bool updatePacketParams(uint8_t length = IMPLICIT_PACKET_LENGTH) {
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
				} else {
					_ldrOptimize = value;
				}
			}
			
			bool fixTXModulationBeforeTransmission() {
				// fix tx modulation for 500 kHz LoRa
				// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.1 for details
				
				uint8_t txModulation = 0;
				
				if (!SPIReadRegister(REG_TX_MODULATION, &txModulation, 1))
					return false;
				
				uint8_t packetType = 0;
				
				if (!getPacketType(packetType))
					return false;
				
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