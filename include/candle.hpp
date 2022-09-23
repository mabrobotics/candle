#pragma once

#include <semaphore.h>

#include <iostream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "bus.hpp"
#include "mab_types.hpp"
#include "md80.hpp"
#include "spiDevice.hpp"
#include "uartDevice.hpp"
#include "usbDevice.hpp"

/* Turn on benchmarking */
#define BENCHMARKING 0
/* Turn on RX and TX timestamps */
#define BENCHMARKING_VERBOSE 0

namespace mab
{
enum CANdleMode_E
{
	CONFIG,
	UPDATE
};

/**
 * @enum CANdleBaudrate_E
 * @brief Enum with coded FDCAN baudrates
 */
enum CANdleBaudrate_E : uint8_t
{
	CAN_BAUD_1M = 1, /*!< FDCAN Baudrate of 1Mbps (1 000 000 bits per second) */
	CAN_BAUD_2M = 2, /*!< FDCAN Baudrate of 2Mbps (2 000 000 bits per second) */
	CAN_BAUD_5M = 5, /*!< FDCAN Baudrate of 5Mbps (5 000 000 bits per second) */
	CAN_BAUD_8M = 8, /*!< FDCAN Baudrate of 8Mbps (8 000 000 bits per second) */
};

/*! \class Candle
	\brief Class for communicating with CANdle (USB-CAN converter) and Md80 drives.

	This class is an connector between non-realtime user code and real-time CANdle firmware. It can be used to
	configure Md80's via FDCAN, as well as control them.
	It can automatically communicate with CANdle/Md80s to relieve the user from requiring them to manually
	control USB and FDCAN communications.
*/
class Candle
{
   private:
	static std::vector<Candle*> instances;
	const std::string version = "v3.0";
	std::thread receiverThread;
	std::thread transmitterThread;
	sem_t transmitted;
	sem_t received;

	bool printVerbose = true;

	CANdleMode_E mode = CANdleMode_E::CONFIG;

	Bus* bus = nullptr;

	uint32_t candleDeviceVersion = 10;
	const uint32_t candleCompatibleVersion = 14;
	const uint32_t md80CompatibleVersion = 20;

	int maxDevices = 12;
	bool shouldStopReceiver;
	bool shouldStopTransmitter;
	mab::CANdleBaudrate_E canBaudrate;

	int msgsReceived = 0;
	int msgsSent = 0;
	float usbCommsFreq = 0.0f;

	/* controller limits */
	static const uint16_t driverMinBandwidth = 50;
	static const uint16_t driverMaxBandwidth = 2500;
	const float driverMaxCurrent = 40.0f;
	const float driverMinCurrent = 1.0f;

	/* register data */
	static const uint32_t maxCanFramelen = 64;
	char regTxBuffer[maxCanFramelen];
	char regRxBuffer[maxCanFramelen];
	char* regTxPtr = nullptr;
	char* regRxPtr = nullptr;

#ifdef BENCHMARKING
	long long txTimestamp = 0;
	bool flag_glob_tx = false;
	bool flag_glob_rx = false;
	long long time_delta;
#endif

	void transmitNewStdFrame();

	void receive();
	void manageReceivedFrame();
	void transmit();

	bool inUpdateMode();
	bool inConfigMode();

	void sendGetInfoFrame(mab::Md80& drive);
	void sendMotionCommand(mab::Md80& drive, float pos, float vel, float torque);

	/* register actions private functions */
	uint32_t packRegister(uint16_t regId, char* regValue, char* buffer);
	uint32_t unPackRegister(uint16_t regId, char* regValue, char* regDataPos);
	uint32_t copyRegister(char* dest, char* source, uint32_t size, uint32_t freeSpace);
	bool prepareFrameMd80Register(mab::Md80FrameId_E frameId, mab::Md80Reg_E regId, char* regValue);

   public:
	/**
	 * @brief A constructor of Candle class
	 * @param canBaudrate Sets a baudrate that CANdle will use to talk to drives
	 * @param printVerbose if true, additional printing will be enables. Usefull for debugging
	 * @param fastMode setups update rate NORMAL for 100Hz (max 12 drives), FAST1 for 250Hz (max 6 drives), FAST2 for 500Hz (max 3 drives)
	 * @param printFailure if false the constructor will not display terminal messages when something fails
	 * @return A functional CANdle class object if succesfull, a nullptr if critical failure occured.
	 */
	Candle(CANdleBaudrate_E canBaudrate, bool printVerbose = false, bool printFailure = true, mab::BusType_E busType = mab::BusType_E::USB);
	/**
	 * @brief A destructor of Candle class. Takes care of all started threads that need to be stopped before clean exit
	 */
	~Candle();
	/**
	 * @brief Updates the current communication speed mode, based on the number of md80s
	 */
	// void updateModeBasedOnMd80List();
	/**
	 * @brief Getter for version number
	 * @return std::string with version in format "vMAJOR.MINOR"
	 */
	const std::string getVersion();

	/**
	 * @brief Getter for device ID. Can be used to differentiate between multiple CANdle's connected to one computer (USB).
	 * @return unique 64-bit identified
	 */
	unsigned long int getDeviceId();

	/**
	 * @brief A vector holding all md80 instances that were succesfully added via `addMd80` method. This vector
	 * can be used to modify regulator and control parameters of the md80 drives.
	 */
	std::vector<Md80> md80s;

	/**
	@brief Enables/disables extended printing.
	*/
	void setVebose(bool enable);

	/**
	 * @brief Returns actual USB communication rate with CANdle. This is calculated by measuring how much time was needed to send 250 messages.
	 * @return average communication frequency in Hertz
	 */
	int getActualCommunicationFrequency();

	/**
	@brief Sends a FDCAN Frame to IDs in range (10 - 2047), and checks for valid responses from Md80 at 1M baudrate.
	@return the vector FDCAN IDs of drives that were found. If no drives were found, the vector is empty
	*/
	std::vector<uint16_t> ping();
	/**
	@brief Sends a FDCAN Frame to IDs in range (10 - 2047), and checks for valid responses from Md80; Pings at specific abudrate
	@param baudrate a baudrate to be pinged.
	@return the vector FDCAN IDs of drives that were found. If no drives were found, the vector is empty
	*/
	std::vector<uint16_t> ping(mab::CANdleBaudrate_E baudrate);

	/**
	@brief Sends a Generic FDCAN Frame to the IDs in range (10 - 2047), and checks for valid responses from Md80;
	@param canId FDCAN ID of the device
	@param msgLen length of FDCAN message
	@param txBuffer pointer to data buffer to be transmited
	@param rxBuffer pointer to data buffer for storing a response. Buffer should be 64 bytes long.
	@param timeoutMs timeout for receiving in milliseconds
	@return true if received response, false otherwise
	*/
	bool sengGenericFDCanFrame(uint16_t canId, int msgLen, const char* txBuffer, char* rxBuffer, int timeoutMs = 100);

	/**
	@brief Adds Md80 to auto update vector.
	@param canId FDCAN ID of the drive to be added
	@param printFailure when false the function will not display fail messages
	@return true if drive has been found and was added, false otherwise
	*/
	bool addMd80(uint16_t canId, bool printFailure = true);
	/**
	@brief Changes FDCAN baudrate that CANdle uses to talk to Md80s.
	@param canBaudrate enum listing all available baudrates. CAN_BAUD_1M is equal to baudrate of 1 000 000 bits per second.
	@param printVersionInfo(optional) checks CANdle firmware version
	@return true if baudrate was changed, false otherwise
	*/
	bool configCandleBaudrate(CANdleBaudrate_E canBaudrate, bool printVersionInfo = false);

	/**
	@brief Changes FDCAN parameters of the Md80.
	@param canId ID of the drive to be modified
	@param newId ID that the drive shall change to
	@param newBaudrateMbps FDCAN baudrate that the drive shall use
	@param newTimeout FDCAN watchdof timeout in milliseconds. If set to 0 the watchdog is disabled.
	@return true if all parameters were changed succesfully, false otherwise
	*/
	bool configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout);
	/**
	@brief Changes max phase-to-phase motor current.
	@param canId ID of the drive
	@param currentLimit phase-to-phase current limit in Amps
	@return true if setting was succesfull, false otherwise
	*/
	bool configMd80SetCurrentLimit(uint16_t canId, float currentLimit);
	/**
	@brief Saves FDCAN and Current Limiter settings to Md80's non-volatile memory
	@param canId ID of the drive
	@return true if saveing was succesfull, false otherwise
	*/
	bool configMd80Save(uint16_t canId);
	/**
	@brief Blink on board LED
	@param canId ID of the drive
	@return true if blinking, false otherwise
	*/
	bool configMd80Blink(uint16_t canId);
	/**
	@brief set torque bandwidth
	@param canId ID of the drive
	@param torqueBandwidth torquer bandwidth to be set
	@return true if change was succesfull, false otherwise
	*/
	bool configMd80TorqueBandwidth(uint16_t canId, uint16_t torqueBandwidth);

	/**
	@brief Sets current motor position as zero position -> reference for any future movements.
	@param drive reference to a Md80 class (candle.md80s memeber)
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80SetEncoderZero(Md80& drive);
	/**
	@brief Changes max phase-to-phase motor current.
	@param canId ID of the drive
	@param currentLimit phase-to-phase current limit in Amps
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80SetEncoderZero(uint16_t canId);

	/**
	@brief Sets control mode of the Md80
	@param drive reference to a Md80 class (candle.md80s memeber)
	@param mode Control mode to be used on the drive
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80Mode(Md80& drive, Md80Mode_E mode);
	/**
	@brief Sets control mode of the Md80
	@param canId ID of the drive
	@param mode Control mode to be used on the drive
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80Mode(uint16_t canId, Md80Mode_E mode);

	/**
	@brief Enables/disabled actuaction of the Md80
	@param drive reference to a Md80 class (candle.md80s memeber)
	@param enable if true the drive will be enabled, if false the drive will be disabled
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80Enable(Md80& drive, bool enable);
	/**
	@brief Enables/disabled actuaction of the Md80
	@param canId ID of the drive
	@param enable if true the drive will be enabled, if false the drive will be disabled
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80Enable(uint16_t canId, bool enable);

	/**
	@brief Searched if the drive with provided FDCAN canId exists in `Md80s` list (exists only if was previously added
	by `addMd80` method)
	@param canId ID of the drive
	@return a reference to a drive if found, nullptr otherwise
	*/
	Md80& getMd80FromList(uint16_t canId);

	/**
	@brief Begins auto update mode. In this mode, host and CANdle will automatically exchange USB messages with md80 commands
	and states. In this mode CANdle will automatically send commands and gather state from all Md80's added to update
	vector with `::addMd80` method. In this mode no config* or control* methods can be called.
	@return true if mode was set succesfully, false otherwise
	*/
	bool begin();
	/**
	@brief Ends auto update mode. Sets mode back to idle (config) mode. In this mode, control*, config* and other methods
	can be used.
	@return true if mode was set succesfully, false otherwise
	*/
	bool end();
	/**
	@brief Clears all parameters set on the CANdle, returning it to the state same as after a powerup.
	@return true if reset was succesfull, false otherwise
	*/
	bool reset();

	/**
	@brief Triggers a calibration routine of the drive's internal electronics.
	@param canId ID of the drive
	@return true if the calibration started succesfully, false otherwise
	*/
	bool setupMd80Calibration(uint16_t canId);
	/**
	@brief Prints diagnostic message from md80.
	@param canId ID of the drive
	@return true if the succesfull, false otherwise
	*/
	bool setupMd80Diagnostic(uint16_t canId);
	/**
	@brief Retrieves extended diagnostic parameters from md80
	@param canId ID of the drive
	@return true if the succesfull, false otherwise
	*/
	bool setupMd80DiagnosticExtended(uint16_t canId);
	/**
	@brief Returns current CAN baudrate
	@return either mab::CANdleBaudrate_E::1M, mab::CANdleBaudrate_E::2M, mab::CANdleBaudrate_E::5M, or mab::CANdleBaudrate_E::8M
	*/
	mab::CANdleBaudrate_E getCurrentBaudrate();
	/**
	@brief checks if a drive could be reached with current baudrate
	@param canId ID of the drive
	@return true if drive was successfully contacted, false otherwise
	*/
	bool checkMd80ForBaudrate(uint16_t canId);
	/**
	@brief writes a single-field register
	@param canId ID of the drive
	@param regId register's ID
	@param value value to be written
	@return true if register was written
	*/

	bool interpretMd80Register(uint16_t canId)
	{
		(void)canId;
		return true;
	};

	template <typename T2, typename... Ts>
	bool interpretMd80Register(uint16_t canId, mab::Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		/* if new frame */
		if (regRxPtr == nullptr)
			regRxPtr = &regRxBuffer[2];

		uint32_t offset = unPackRegister(regId, (char*)&regValue, regRxPtr);
		if (offset == 0)
			return false;

		regRxPtr += offset;
		return interpretMd80Register(canId, vs...);
	}

	bool prepareMd80Register(uint16_t canId, mab::Md80FrameId_E frameType)
	{
		(void)frameType;
		/* clear the RX buffer and send register request */
		memset(regRxBuffer, 0, sizeof(regRxBuffer));
		regTxPtr = nullptr;
		regRxPtr = nullptr;
		return sengGenericFDCanFrame(canId, sizeof(regTxBuffer), regTxBuffer, regRxBuffer, 100);
	}

	template <typename T2, typename... Ts>
	bool prepareMd80Register(uint16_t canId, mab::Md80FrameId_E frameType, mab::Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		static_assert(!std::is_same<double, T2>::value, "register value should be float not double");
		if (!prepareFrameMd80Register(frameType, regId, (char*)&regValue))
			return false;
		return prepareMd80Register(canId, frameType, vs...);
	}

	template <typename T2, typename... Ts>
	bool readMd80Register(uint16_t canId, mab::Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		/* prepare and send the request frame */
		if (!prepareMd80Register(canId, mab::Md80FrameId_E::FRAME_READ_REGISTER, regId, regValue, vs...))
			return false;
		/* interpret the frame */
		return interpretMd80Register(canId, regId, regValue, vs...);
	}

	template <typename T2, typename... Ts>
	bool writeMd80Register(uint16_t canId, mab::Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		return prepareMd80Register(canId, mab::Md80FrameId_E::FRAME_WRITE_REGISTER, regId, regValue, vs...);
	}

#if BENCHMARKING == 1
	bool benchGetFlagRx();
	bool benchGetFlagTx();
	void benchSetFlagRx(bool state);
	void benchSetFlagTx(bool state);
	long long benchGetTimeDelta();
#endif
};

}  // namespace mab
