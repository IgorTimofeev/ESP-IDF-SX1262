
// Based on excellent work of https://github.com/jgromes/RadioLib/
// and adapted for ESP-IDF realities without Arduino shitware
//
// - Replaced GPIO polling loops with interrupt-based FreeRTOS semaphores
// - Replaced HALs with native SPI and DMA transactions

#pragma once

#include <cstdint>
#include <span>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>

namespace YOBA {
	enum class SX1262Error {
		none,
		invalidChip,
		SPITransaction,
		timeout,
		invalidArgument,
		invalidPacketType,
		invalidChecksum
	};

	enum class SX1262LoRaCodingRate : uint8_t {
		cr4_5,                // 4/5
		cr4_6,                // 4/6
		cr4_7,                // 4/7
		cr4_8,                // 4/8

		cr4_5LongInterleaver, // 4/5, long interleaver
		cr4_6LongInterleaver, // 4/6, long interleaver
		cr4_8LongInterleaver, // 4/8, long interleaver
	};

	enum class SX1262LoRaBandwidth : uint8_t {
		bw7_81,  // 7.81 kHz
		bw10_42, // 10.42 kHz
		bw15_63, // 15.63 kHz
		bw20_83, // 20.83 kHz
		bw31_25, // 31.25 kHz
		bw41_67, // 41.67 kHz
		bw62_5,  // 62.5 kHz
		bw125_0, // 125.0 kHz
		bw250_0, // 250.0 kHz
		bw500_0, // 500.0 kHz
	};

	class SX1262 {
		public:
			SX1262Error setup(
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
			);

			SX1262Error reset();
			void setSPIMutex(const SemaphoreHandle_t mutex);
			SX1262Error validateChip();

			/*!
			  \brief Perform image rejection calibration for the specified frequency.
			  Will try to use Semtech-defined presets first, and if none of them matches,
			  custom iamge calibration will be attempted using calibrateImageRejection.
			  \param frequencyHz Frequency to perform the calibration for.
			  \returns \ref status_codes
			*/
			SX1262Error calibrateImage(const uint32_t frequencyHz);
			SX1262Error setRFFrequency(const uint32_t frequencyHz);
			SX1262Error setStandby(const uint8_t value = STANDBY_RC);
			SX1262Error setSymbNumTimeout(const uint8_t value);

			// Allowed timeout values:
			//
			// | Value                       | Description |
			// | RX_TIMEOUT_NONE (0)         | RX single mode |
			// | RX_TIMEOUT_INF (0xFFFFFF)   | RX continuous mode |
			// | others                      | RX with timeout |
			SX1262Error setRX(const uint32_t timeoutUs = RX_TIMEOUT_NONE);
			SX1262Error setTX(const uint32_t timeoutUs = 0);
			SX1262Error setRXTXFallbackMode(const uint8_t value = RX_TX_FALLBACK_MODE_STDBY_RC);
			// Depends on spreading factor
			SX1262Error setLoRaCADParams();
			SX1262Error setBufferBaseAddress(const uint8_t rxAddress = 0x00, const uint8_t txAddress = 0x00);
			SX1262Error getStatus(uint8_t& status);
			SX1262Error getPacketType(uint8_t& packetType);
			SX1262Error setPacketType(const uint8_t packetType);
			SX1262Error setRegulatorMode(const uint8_t mode);
			SX1262Error getIRQStatus(uint16_t& status);
			SX1262Error clearIRQStatus(const uint16_t status = IRQ_ALL);
			SX1262Error setDIOIRQParams(const uint16_t irqMask = IRQ_NONE, const uint16_t dio1Mask = IRQ_NONE, const uint16_t dio2Mask = IRQ_NONE, const uint16_t dio3Mask = IRQ_NONE);
			SX1262Error calibrate(const uint8_t value);

			/*!
			 \brief Sets LoRa sync word.

			 \param syncWord LoRa sync word to be set.
			 \param controlBits Undocumented control bits, required for compatibility purposes.

			 \returns \ref status_codes
		   */
			SX1262Error setLoRaSyncWord(const uint8_t syncWord, const uint8_t controlBits = 0x44);

			/*!
			 \brief Sets current protection limit. Can be set in 2.5 mA steps.
			 \param currentLimit current protection limit to be set in mA. Allowed values range from 0 to 140.
			 \returns \ref status_codes
		   */
			SX1262Error setCurrentLimit(const float currentLimit);

			/*!
			  \brief Set DIO2 to function as RF switch (default in Semtech example designs).
			  \returns \ref status_codes
			*/
			SX1262Error setDio2AsRfSwitch(const bool enable);

			SX1262Error setLoRaModulationParams(
				const uint8_t spreadingFactor,
				const SX1262LoRaBandwidth bandwidth,
				const SX1262LoRaCodingRate codingRate,
				const uint8_t LDROptimize = false
			);

			SX1262Error setTXClampConfig(const bool enable);
			SX1262Error setOutputPower(int8_t powerDBm);

			SX1262Error setPacketParams(
				const uint16_t preambleLength,
				const uint8_t headerType,
				const uint8_t length,
				const uint8_t crcType,
				const uint8_t invertIQ
			);

			SX1262Error writeBuffer(const std::span<const uint8_t> data, const uint8_t offset = 0x00);
			SX1262Error readBuffer(const std::span<uint8_t> data, const uint8_t offset);
			SX1262Error getPacketStatus(uint32_t& status);
			SX1262Error getRSSI(float& rssi);
			SX1262Error getRSSIInst(float& rssi);
			SX1262Error getSNR(float& snr);

			/*!
			  \brief Sets preamble length for LoRa or FSK modem. Allowed values range from 1 to 65535.
			  \param preambleLength Preamble length to be set in symbols (LoRa) or bits (FSK).
			  NOTE: In FSK mode, sync word length limits the preamble detector length
			  (the number of preamble bits that must be detected to start receiving packet).
			  For details, see the note in SX1261 datasheet, Rev 2.1, section 6.2.2.1, page 45.
			  Preamble detector length is adjusted automatically each time this method is called.
			  \returns \ref status_codes
			*/
			SX1262Error setLoRaPreambleLength(const uint16_t preambleLength);
			SX1262Error setLoRaCRC(const bool enabled);

			/*!
			 \brief Enable/disable inversion of the I and Q signals
			 \param enable IQ inversion enabled (true) or disabled (false);
			 \returns \ref status_codes
		   */
			SX1262Error invertLoRaIQ(const bool enable);
			SX1262Error waitForDIO1Semaphore(const uint32_t timeoutUs) const;
			SX1262Error fixImplicitTimeout();
			SX1262Error getReceivedPacketLength(uint8_t& length, uint8_t& offset);
			SX1262Error fixLoRaTXModulationBeforeTransmission();
			SX1262Error setPacketLength(const uint8_t packetLength);

			/*!
			 \brief Set the PA (power amplifier) configuration. Allows user to optimize PA for a specific output power
			 and matching network. Any calls to this method must be done after calling begin/beginFSK and/or setOutputPower.
			 WARNING: Use at your own risk! Setting invalid values can and will lead to permanent damage!
			 \param paDutyCycle PA duty cycle raw value.
			 \param deviceSel Device select, usually PA_CONFIG_SX1262,
			 PA_CONFIG_SX1262 or PA_CONFIG_SX1268.
			 \param hpMax hpMax raw value.
			 \param paLut paLut PA lookup table raw value.
			 \returns \ref status_codes
			*/
			SX1262Error setPAConfig(const uint8_t paDutyCycle = 0x04, const uint8_t deviceSel = PA_CONFIG_SX1262, const uint8_t hpMax = PA_CONFIG_HP_MAX, const uint8_t paLut = PA_CONFIG_PA_LUT);
			SX1262Error setTXParams(const int8_t power, const uint8_t rampTime);

			static void errorToString(const SX1262Error error, const std::span<char> str);

			// -------------------------------- Easy-to-use methods --------------------------------

			SX1262Error receive(uint8_t* buffer, uint8_t& receivedLength, uint32_t timeoutUs = 0);
			SX1262Error transmit(const std::span<const uint8_t> data, uint32_t timeoutUs = 0);

		private:
			constexpr static auto _logTag = "SX1262";

			uint32_t _frequencyHz = 0;
			uint8_t _LoRaSpreadingFactor = 7;
			SX1262LoRaCodingRate _LoRaCodingRate = SX1262LoRaCodingRate::cr4_5;
			SX1262LoRaBandwidth _LoRaBandwidth = SX1262LoRaBandwidth::bw500_0;
			uint16_t _preambleLength = 0;
			uint8_t _crcType = LORA_CRC_ON;
			uint8_t _headerType = LORA_HEADER_EXPLICIT;
			uint8_t _packetType = PACKET_TYPE_LORA;
			uint8_t _invertIQ = LORA_IQ_STANDARD;
			uint8_t _packetLength = 255;

			// -------------------------------- GPIO --------------------------------

			gpio_num_t _SSPin = GPIO_NUM_NC;
			gpio_num_t _busyPin = GPIO_NUM_NC;
			gpio_num_t _DIO1Pin = GPIO_NUM_NC;
			gpio_num_t _RSTPin = GPIO_NUM_NC;

			void setSSPinLevel(const bool value) const;
			void setRSTPinLevel(const bool value) const;
			bool getBusyPinLevel() const;
			bool getDIO1PinLevel() const;

			// -------------------------------- Interrupts --------------------------------

			SemaphoreHandle_t _busyPinSemaphore {};
			SemaphoreHandle_t _DIO1PinSemaphore {};

			IRAM_ATTR void onBusyPinInterrupt() const;
			IRAM_ATTR void onDIO1PinInterrupt() const;

			SX1262Error waitForBusyPin(const uint32_t timeoutMs = 1'000) const;

			// -------------------------------- SPI --------------------------------

			spi_device_handle_t _SPIDevice {};
			SemaphoreHandle_t _SPIMutex = nullptr;

			constexpr static uint16_t _SPIBufferLength = 4 + 256;
			uint8_t _SPIBuffer[_SPIBufferLength] {};

			bool SPITransmit(spi_transaction_t* t) const;
			SX1262Error SPIReadCommand(const uint8_t command, const std::span<uint8_t> data);
			SX1262Error SPIReadRegister(const uint16_t reg, const std::span<uint8_t> data);
			SX1262Error SPIWrite(const uint8_t* data, const uint16_t length);
			SX1262Error SPIWriteCommandAndUint8(const uint8_t command, const uint8_t data);
			SX1262Error SPIWriteRegister(const uint16_t reg, const std::span<const uint8_t> data);
			SX1262Error SPIWriteBuffer(const uint16_t totalLength) const;

			// -------------------------------- Auxiliary --------------------------------

			static bool checkESPError(const esp_err_t error);
			SX1262Error checkForLoRaPacketType();
			static void delayMs(const uint32_t ms);
			SX1262Error setRXOrTX(const uint8_t command, const uint32_t timeoutUs);
			SX1262Error fixInvertedIQ(const uint8_t iqConfig);

			static float getRSSIFromPacketStatus(const uint32_t packetStatus);
			static float getSNRFromPacketStatus(const uint32_t packetStatus);
			SX1262Error updatePacketParams();

			SX1262Error finishReceive();


		public:
			
			// -------------------------------- Module properties --------------------------------
			
			constexpr static uint32_t RF_XTAL_FREQUENCY_HZ = 32'000'000;
			constexpr static uint32_t RF_DIVIDER = 1ULL << 25;
			constexpr static uint8_t MAX_PACKET_LENGTH = 255;

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
			constexpr static uint32_t CAL_IMG_FREQ_TRIG_HZ = 20'000'000;
			
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