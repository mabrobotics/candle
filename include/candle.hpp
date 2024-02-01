#pragma once

#include <semaphore.h>

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "bus.hpp"
#include "candle_protocol.hpp"
#include "mab_types.hpp"
#include "md80.hpp"
#include "spiDevice.hpp"
#include "uartDevice.hpp"
#include "usbDevice.hpp"
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
	\brief Class for communicating with CANdle (USB-CAN converter) and MD80 drives.

	This class is an connector between non-realtime user code and real-time CANdle firmware. It can be used to
	configure MD80's via FDCAN, as well as control them.
	It can automatically communicate with CANdle/MD80s to relieve the user from requiring them to manually
	control USB and FDCAN communications.
*/
class Candle
{
   public:
	/**
	 * @brief A constructor of Candle class
	 * @param canBaudrate Sets a baudrate that CANdle will use to talk to drives
	 * @param printVerbose if true, additional printing will be enables. Useful for debugging
	 * @return A functional CANdle class object if succesfull, a nullptr if critical failure occured.
	 */
	explicit Candle(CANdleBaudrate_E canBaudrate, bool printVerbose = true, mab::BusType_E busType = BusType_E::USB, const std::string device = "");
	/**
	 * @brief A constructor of Candle class used for testing purposes
	 * @param canBaudrate Sets a baudrate that CANdle will use to talk to drives
	 * @param printVerbose if true, additional printing will be enables. Usefull for debugging
	 * @param bus a bus object pointer to be used in CANdle class instance
	 * @return A functional CANdle class object if succesfull, a nullptr if critical failure occured.
	 */
	explicit Candle(CANdleBaudrate_E canBaudrate, bool printVerbose, std::shared_ptr<Bus> bus);
	/**
	 * @brief A destructor of Candle class. Takes care of all started threads that need to be stopped before clean exit
	 */
	~Candle();

	/**
	 * @brief Getter for version number
	 * @return std::string with version in format "vMAJOR.MINOR.REVISION.TAG"
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
	 * @brief Sets transmit thread sleep time (can be used to free resources when highest communication frequency is not needed)
	 */
	void setTransmitDelayUs(uint32_t delayUs);

	/**
	@brief Sends a FDCAN Frame to IDs in range (10 - 2047), and checks for valid responses from Md80 at 1M baudrate.
	@return the vector FDCAN IDs of drives that were found. If no drives were found, the vector is empty
	*/
	std::vector<uint16_t> ping();
	/**
	@brief Sends a FDCAN Frame to IDs in range (10 - 2047), and checks for valid responses from MD80; Pings at specific baudrate
	@param baudrate specific baudrate to be pinged.
	@return the vector FDCAN IDs of drives that were found. If no drives were found, the vector is empty
	*/
	std::vector<uint16_t> ping(mab::CANdleBaudrate_E baudrate);

	/**
	@brief Sends a Generic FDCAN Frame to the IDs in range (10 - 2047), and checks for valid responses from MD80;
	@param canId FDCAN ID of the device
	@param msgLen length of FDCAN message
	@param txBuffer pointer to data buffer to be transmited
	@param rxBuffer pointer to data buffer for storing a response. Buffer should be 64 bytes long.
	@param timeoutMs timeout for receiving in milliseconds
	@return true if received response, false otherwise
	*/
	bool sendGenericFDCanFrame(uint16_t canId, int msgLen, const char* txBuffer, char* rxBuffer, int timeoutMs = 100);

	/**
	@brief Adds MD80 to auto update vector.
	@param canId FDCAN ID of the drive to be added
	@param printFailure when false the function will not display fail messages
	@return true if drive has been found and was added, false otherwise
	*/
	bool addMd80(uint16_t canId, bool printFailure = true);
	/**
	@brief Changes FDCAN baudrate that CANdle uses to talk to MD80s.
	@param canBaudrate enum listing all available baudrates. CAN_BAUD_1M is equal to baudrate of 1 000 000 bits per second.
	@param printVersionInfo(optional) checks CANdle firmware version
	@return true if baudrate was changed, false otherwise
	*/
	bool configCandleBaudrate(CANdleBaudrate_E canBaudrate, bool printVersionInfo = false);

	/**
	@brief Changes FDCAN parameters of the MD80.
	@param canId ID of the drive to be modified
	@param newId ID that the drive shall change to
	@param newBaudrateMbps FDCAN baudrate that the drive shall use
	@param newTimeout FDCAN watchdog timeout in milliseconds. If set to 0 the watchdog is disabled.
	@return true if all parameters were changed succesfully, false otherwise
	*/
	bool configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout, bool canTermination = false);
	/**
	@brief Changes max phase-to-phase motor current.
	@param canId ID of the drive
	@param currentLimit phase-to-phase current limit in Amps
	@return true if setting was succesfull, false otherwise
	*/
	bool configMd80SetCurrentLimit(uint16_t canId, float currentLimit);
	/**
	@brief Saves FDCAN and Current Limiter settings to MD80's non-volatile memory
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
	@param drive reference to a MD80 class (candle.md80s member)
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80SetEncoderZero(Md80& drive);
	/**
	@brief Sets current motor position as zero position -> reference for any future movements.
	@param canId uint16_t drive CAN bus ID
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80SetEncoderZero(uint16_t canId);

	/**
	@brief Sets control mode of the MD80
	@param drive reference to a MD80 class (candle.md80s memeber)
	@param mode Control mode to be used on the drive
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80Mode(Md80& drive, Md80Mode_E mode);
	/**
	@brief Sets control mode of the MD80
	@param canId ID of the drive
	@param mode Control mode to be used on the drive
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80Mode(uint16_t canId, Md80Mode_E mode);

	/**
	@brief Enables/disabled actuaction of the MD80
	@param drive reference to a MD80 class (candle.md80s memeber)
	@param enable if true the drive will be enabled, if false the drive will be disabled
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80Enable(Md80& drive, bool enable);
	/**
	@brief Enables/disabled actuaction of the MD80
	@param canId ID of the drive
	@param enable if true the drive will be enabled, if false the drive will be disabled
	@return true if setting was succesfull, false otherwise
	*/
	bool controlMd80Enable(uint16_t canId, bool enable);

	/**
	@brief Searched if the drive with provided FDCAN canId exists in `Md80s` list (exists only if was previously added
	by `addMd80` method)
	@param canId ID of the drive
	@return a reference to a drive if found, exception will be thronw if not found
	*/
	Md80& getMd80FromList(uint16_t canId);

	/**
	@brief Begins auto update mode. In this mode, host and CANdle will automatically exchange USB messages with md80 commands
	and states. In this mode CANdle will automatically send commands and gather state from all MD80's added to update
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
	@brief Triggers an output encoder calibration routine of the drive's internal electronics.
	@param canId ID of the drive
	@return true if the calibration started succesfully, false otherwise
	*/
	bool setupMd80CalibrationOutput(uint16_t canId);
	/**
	@brief Triggers an output encoder check routine. After routine completion min, max and stdDev error can be read from registers.
	@param canId ID of the drive
	@return true if the check routine started succesfully, false otherwise
	*/
	bool setupMd80TestOutputEncoder(uint16_t canId);
	/**
	@brief Triggers a main encoder check routine. After routine completion min, max and stdDev error can be read from registers.
	@param canId ID of the drive
	@return true if the check routine started succesfully, false otherwise
	*/
	bool setupMd80TestMainEncoder(uint16_t canId);
	/**
	@brief Triggers a homing routine.
	@return true if the homing routine started succesfully, false otherwise.
	*/
	bool setupMd80PerformHoming(uint16_t canId);
	/**
	@brief Triggers a controller reset.
	@return true if the reset routine started succesfully, false otherwise.
	*/
	bool setupMd80PerformReset(uint16_t canId);
	/**
	@brief Clears all non-critical errors.
	@return true if succeded, false otherwise.
	*/
	bool setupMd80ClearErrors(uint16_t canId);
	/**
	@brief Clears all warnings.
	@return true if succeded, false otherwise.
	*/
	bool setupMd80ClearWarnings(uint16_t canId);
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
	@brief reads single-field registers
	@param canId ID of the drive
	@param regId first register's ID
	@param value first reference to a variable where the read value should be stored
	@param ...	remaining regId-value pairs to be read
	@return true if register was read
	*/
	template <typename T2, typename... Ts>
	bool readMd80Register(uint16_t canId, Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		return md80Register->read(canId, regId, regValue, vs...);
	}
	/**
	@brief writes single-field registers
	@param canId ID of the drive
	@param regId first register's ID
	@param value first reference to a value that should be written
	@param ...	remaining regId-value pairs to be written
	@return true if register was written
	*/
	template <typename T2, typename... Ts>
	bool writeMd80Register(uint16_t canId, Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		return md80Register->write(canId, regId, regValue, vs...);
	}

   protected:
	std::shared_ptr<Register> md80Register;

   private:
	/* TODO make a proper version class as the reverse initalization is not elegant */
	const version_ut candleDeviceCompatibleVersion = {'r', 0, 2, 2};
	const version_ut md80CompatibleVersion = {'r', 0, 3, 2};

	static std::vector<Candle*> instances;

	std::thread receiverThread;
	std::thread transmitterThread;
	sem_t transmitted;
	sem_t received;

	bool printVerbose = true;

	CANdleMode_E mode = CANdleMode_E::CONFIG;

	std::shared_ptr<Bus> bus = nullptr;

	static constexpr uint16_t idMax = 2000;
	static constexpr int maxDevices = 16;
	bool shouldStopReceiver;
	bool shouldStopTransmitter;
	mab::CANdleBaudrate_E canBaudrate;

	int msgsReceived = 0;
	int msgsSent = 0;
	float usbCommsFreq = 0.0f;
	uint32_t transmitterDelay = 20;

	void transmitNewStdFrame();

	void receive();
	void manageReceivedFrame();
	void transmit();

	bool inUpdateMode();

	void updateMd80State(mab::Md80& drive);

	std::shared_ptr<Bus> makeBus(mab::BusType_E busType, std::string device);

	bool executeCommand(uint16_t canId, Md80Reg_E reg, const char* failMsg, const char* successMsg);
	bool sendBusFrame(BusFrameId_t id, uint32_t timeout, char* payload = nullptr, uint32_t cmdLen = 2, uint32_t respLen = 2);

	/* virtual methods for testing purposes */
	virtual std::shared_ptr<Bus> createSpi() { return std::make_shared<SpiDevice>(); }
	virtual std::shared_ptr<Bus> createUart() { return std::make_shared<UartDevice>(); }
	virtual std::shared_ptr<Bus> createUsb(const std::string idVendor, const std::string idProduct, std::vector<unsigned long> instances) { return std::make_shared<UsbDevice>(idVendor, idProduct, instances); }
};

std::string getVersionString(const version_ut& ver);

}  // namespace mab
