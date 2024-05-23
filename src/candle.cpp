#include "candle.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include "candle_protocol.hpp"
#include "register.hpp"

namespace mab
{
class mystreambuf : public std::streambuf
{
};
mystreambuf nostreambuf;
std::ostream nocout(&nostreambuf);
#define vout ((this->printVerbose) ? std::cout << "[CANDLE] " : nocout)
/* note: this will not work on all terminals */
std::string statusOK = "  \033[1;32m[OK]\033[0m";
std::string statusFAIL = "  \033[1;31m[FAILED]\033[0m";

uint64_t getTimestamp()
{
	using namespace std::chrono;
	return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

std::vector<Candle*> Candle::instances = std::vector<Candle*>();

Candle::Candle(CANdleBaudrate_E canBaudrate, bool printVerbose, mab::BusType_E busType, const std::string device)
	: Candle(canBaudrate, printVerbose, makeBus(busType, device)) {}

Candle::Candle(CANdleBaudrate_E canBaudrate, bool printVerbose, std::shared_ptr<Bus> bus)
	: printVerbose(printVerbose), bus(bus)
{
	vout << "CANdle library version: v" << getVersion() << std::endl;

	reset();
	usleep(5000);

	if (!configCandleBaudrate(canBaudrate, true))
	{
		vout << "Failed to set up CANdle baudrate @" << canBaudrate << "Mbps!" << statusFAIL << std::endl;
		throw "Failed to set up CANdle baudrate!";
	}

	if (bus->getType() == mab::BusType_E::USB)
		vout << "CANdle at " << bus->getDeviceName() << ", ID: 0x" << std::hex << getDeviceId() << std::dec << " ready (USB)" << std::endl;
	else if (bus->getType() == mab::BusType_E::SPI)
		vout << "CANdle ready (SPI)" << std::endl;
	else if (bus->getType() == mab::BusType_E::UART)
		vout << "CANdle ready (UART)" << std::endl;

	md80Register = std::make_shared<Register>(this);
	Candle::instances.push_back(this);
}

Candle::~Candle()
{
	if (inUpdateMode())
		end();
}

std::shared_ptr<Bus> Candle::makeBus(mab::BusType_E busType, std::string device)
{
	switch (busType)
	{
		case mab::BusType_E::USB:
		{
			std::vector<unsigned long> a;
			for (auto& instance : instances)
				a.push_back(instance->getDeviceId());
			return std::make_shared<UsbDevice>("MAB_Robotics", "MD_USB-TO-CAN", a);
		}
		case mab::BusType_E::SPI:
		{
			if (device != "")
				return std::make_shared<SpiDevice>(device);
			else
				return std::make_shared<SpiDevice>();
		}

		case mab::BusType_E::UART:
		{
			if (device != "")
				return std::make_shared<UartDevice>(device);
			else
				return std::make_shared<UartDevice>();
		}
		default:
			throw "Error wrong bus type specified!";
	}
	return nullptr;
}

const std::string Candle::getVersion()
{
	return getVersionString({CANDLE_VTAG, CANDLE_VREVISION, CANDLE_VMINOR, CANDLE_VMAJOR});
}

int Candle::getActualCommunicationFrequency()
{
	return static_cast<int>(usbCommsFreq);
}

void Candle::setTransmitDelayUs(uint32_t delayUs)
{
	transmitterDelay = delayUs < 20 ? 20 : delayUs;
}

void Candle::receive()
{
	while (!shouldStopReceiver)
	{
		/* wait for a frame to be transmitted */
		sem_wait(&transmitted);

		if (!shouldStopReceiver && bus->receive(sizeof(StdMd80ResponseFrame_t) * md80s.size() + 1))
		{
			/* notify a frame was received */
			sem_post(&received);

			if (*bus->getRxBuffer() == BUS_FRAME_UPDATE)
				manageReceivedFrame();
		}
		else
		{
			if (!shouldStopReceiver)
				vout << "Did not receive response from CANdle!" << statusFAIL << std::endl;
			shouldStopReceiver = true;
			shouldStopTransmitter = true;
			sem_post(&received);
		}
	}
}
void Candle::transmit()
{
	int txCounter = 0;
	uint64_t freqCheckStart = getTimestamp();
	while (!shouldStopTransmitter)
	{
		if (++txCounter == 250)
		{
			usbCommsFreq = 250.0 / (float)(getTimestamp() - freqCheckStart) * 1000000.0f;
			freqCheckStart = getTimestamp();
			txCounter = 0;
		}

		transmitNewStdFrame();

		/* notify a frame was sent */
		if (bus->getType() != mab::BusType_E::SPI)
			sem_post(&transmitted);

		/* transmit thread is also the receive thread for SPI in update mode */
		if (bus->getType() == mab::BusType_E::SPI && *bus->getRxBuffer() == BUS_FRAME_UPDATE)
			manageReceivedFrame();

		msgsSent++;

		if (bus->getType() == mab::BusType_E::SPI)
		{
			switch (canBaudrate)
			{
				case CAN_BAUD_1M:
				{
					usleep(750 * md80s.size());
					break;
				}
				case CAN_BAUD_2M:
				{
					usleep(450 * md80s.size());
					break;
				}
				case CAN_BAUD_5M:
				{
					usleep(250 * md80s.size());
					break;
				}
				case CAN_BAUD_8M:
				{
					usleep(200 * md80s.size());
					break;
				}
			}
		}
		/* wait for a frame to be received */
		else
			sem_wait(&received);
		usleep(transmitterDelay);
	}
}

void Candle::manageReceivedFrame()
{
	for (size_t i = 0; i < md80s.size(); i++)
		md80s[i].__updateResponseData((StdMd80ResponseFrame_t*)bus->getRxBuffer(1 + i * sizeof(StdMd80ResponseFrame_t)));
}

void Candle::setVebose(bool enable)
{
	printVerbose = enable;
}

unsigned long Candle::getDeviceId()
{
	return bus->getId();
}

void Candle::updateMd80State(mab::Md80& drive)
{
	Md80::State state{};

	md80Register->read(drive.getId(), Md80Reg_E::mainEncoderPosition, state.position,
					   Md80Reg_E::mainEncoderVelocity, state.velocity,
					   Md80Reg_E::motorTorque, state.torque,
					   Md80Reg_E::outputEncoderPosition, state.outputEncoderPosition,
					   Md80Reg_E::outputEncoderVelocity, state.outputEncoderVelocity,
					   Md80Reg_E::motorTemperature, state.temperature,
					   Md80Reg_E::quickStatus, state.quickStatus);

	drive.__updateResponseData(state);
}

bool Candle::addMd80(uint16_t canId, bool printFailure)
{
	if (inUpdateMode())
		return false;

	for (auto& md : md80s)
	{
		if (md.getId() == canId)
		{
			vout << "MD80 with ID: " << canId << " is already on the update list." << statusOK << std::endl;
			return true;
		}
	}

	if (md80s.size() >= maxDevices)
	{
		vout << "Cannot add more than " << maxDevices << " MD80s" << statusFAIL << std::endl;
		return false;
	}

	char payload[2]{};
	*(uint16_t*)payload = canId;

	if (sendBusFrame(BUS_FRAME_MD80_ADD, 2, payload, 3, 2))
	{
		version_ut firmwareVersion = {{0, 0, 0, 0}};

		if (!md80Register->read(canId, Md80Reg_E::firmwareVersion, firmwareVersion.i))
		{
			vout << "Unable to read MD80's firmware version! Please check the ID, or update the MD80 with MAB_CAN_Flasher." << statusFAIL << std::endl;
			return false;
		}

		if (firmwareVersion.i < md80CompatibleVersion.i)
		{
			vout << "MD80's firmware with ID: " + std::to_string(canId) + " is outdated. Please update it using MAB_CAN_Flasher." << statusFAIL << std::endl;
			return false;
		}
		else if (firmwareVersion.s.major > md80CompatibleVersion.s.major || firmwareVersion.s.minor > md80CompatibleVersion.s.minor)
		{
			vout << "MD80's firmware with ID: " + std::to_string(canId) + " is a future version. Please update your CANdle library." << statusFAIL << std::endl;
			return false;
		}
		vout << "Added MD80 with ID: " + std::to_string(canId) << statusOK << std::endl;
		md80s.push_back(Md80(canId));
		mab::Md80& newDrive = md80s.back();
		updateMd80State(newDrive);
		return true;
	}

	if (printFailure) vout << "Failed to add MD80 with ID: " + std::to_string(canId) << statusFAIL << std::endl;
	return false;
}
std::vector<uint16_t> Candle::ping(mab::CANdleBaudrate_E baudrate)
{
	if (!configCandleBaudrate(baudrate))
		return std::vector<uint16_t>();
	vout << "Starting pinging drives at baudrate: " << std::to_string(baudrate) << "M" << std::endl;
	std::vector<uint16_t> ids{};

	if (sendBusFrame(BUS_FRAME_PING_START, 2000, nullptr, 2, 33))
	{
		uint16_t* idsPointer = (uint16_t*)bus->getRxBuffer(1);
		for (int i = 0; i < maxDevices; i++)
		{
			uint16_t id = idsPointer[i];
			if (id == 0x00)
				break;
			ids.push_back(id);
		}
		if (ids.size() == 0)
		{
			vout << "No drives found." << std::endl;
			return ids;
		}
		vout << "Found drives." << std::endl;
		for (size_t i = 0; i < ids.size(); i++)
		{
			if (ids[i] == 0)
				break;	// No more ids in the message

			vout << std::to_string(i + 1) << ": ID = " << ids[i] << " (0x" << std::hex << ids[i] << std::dec << ")" << std::endl;
			if (ids[i] > idMax)
			{
				vout << "Error! This ID is invalid! Probably two or more drives share same ID."
					 << "Communication will most likely be broken until IDs are unique!" << statusFAIL << std::endl;
				std::vector<uint16_t> empty;
				return empty;
			}
		}
	}
	return ids;
}
std::vector<uint16_t> Candle::ping()
{
	return ping(canBaudrate);
}
bool Candle::sendGenericFDCanFrame(uint16_t canId, int msgLen, const char* txBuffer, char* rxBuffer, int timeoutMs)
{
	GenericMd80Frame64 frame;
	frame.frameId = mab::BusFrameId_t::BUS_FRAME_MD80_GENERIC_FRAME;
	frame.driveCanId = canId;
	frame.canMsgLen = msgLen;
	frame.timeoutMs = timeoutMs < 3 ? 3 : timeoutMs - 3;
	memcpy(frame.canMsg, txBuffer, msgLen);
	char tx[96];
	int len = sizeof(frame) - sizeof(frame.canMsg) + msgLen;
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, timeoutMs, 66, false))	 // Got some response
	{
		if (*bus->getRxBuffer(0) == tx[0] &&  // USB Frame ID matches
			*bus->getRxBuffer(1) == true &&
			bus->getBytesReceived() <= 64 + 2)	// response can ID matches
		{
			memcpy(rxBuffer, bus->getRxBuffer(2), bus->getBytesReceived() - 2);
			return true;
		}
	}
	return false;
}

bool Candle::configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout, bool canTermination)
{
	if (newId < 10 || newId > idMax)
	{
		vout << "CAN config change failed, ID out of range! Please use a valid ID [10-2000]" << statusFAIL << std::endl;
		return false;
	}

	if (!md80Register->write(canId, Md80Reg_E::canId, (uint32_t)newId,
							 Md80Reg_E::canBaudrate, newBaudrateMbps * 1000000,
							 Md80Reg_E::canWatchdog, newTimeout,
							 Md80Reg_E::canTermination, (uint8_t)canTermination,
							 Md80Reg_E::runCanReinit, true))
	{
		vout << "CAN config change failed!" << statusFAIL << std::endl;
		return false;
	}

	vout << "Drive ID: " << std::to_string(canId) << " was changed to ID: " << std::to_string(newId) << std::endl;
	vout << "It's baudrate is now " << std::to_string(newBaudrateMbps) << "Mbps" << std::endl;
	vout << "It's CAN timeout (watchdog) is now " << (newTimeout == 0 ? "disabled" : std::to_string(newTimeout) + "ms") << std::endl;
	vout << "It's CAN termination resistor is " << (canTermination == true ? "enabled" : "disabled") << std::endl;
	vout << "CAN config change successful!" << statusOK << std::endl;
	return true;
}

bool Candle::configMd80Save(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runSaveCmd, "Saving in flash failed at ID: ", "Saving in flash successful at ID: ");
}

bool Candle::configMd80Blink(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runBlink, "Blinking failed at ID: ", "LEDs blining at ID:");
}

bool Candle::controlMd80SetEncoderZero(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runZero, "Setting new zero position failed at ID: ", "Setting new zero position successful at ID: ");
}

bool Candle::configMd80SetCurrentLimit(uint16_t canId, float currentLimit)
{
	if (inUpdateMode() || !md80Register->write(canId, Md80Reg_E::motorIMax, currentLimit))
	{
		vout << "Setting new current limit failed at ID: " << canId << statusFAIL << std::endl;
		return false;
	}
	vout << "Setting new current limit successful at ID: " << canId << statusOK << std::endl;
	return true;
}

bool Candle::configCandleBaudrate(CANdleBaudrate_E canBaudrate, bool printVersionInfo)
{
	this->canBaudrate = canBaudrate;

	char payload[1]{};
	payload[0] = static_cast<uint8_t>(canBaudrate);

	if (sendBusFrame(BUS_FRAME_CANDLE_CONFIG_BAUDRATE, 50, payload, 2, 6))
	{
		version_ut candleDeviceVersion{};
		candleDeviceVersion.i = *(uint32_t*)bus->getRxBuffer(2);

		if (printVersionInfo)
		{
			if (candleDeviceVersion.i < candleDeviceCompatibleVersion.i)
			{
				vout << "Your CANdle device firmware seems to be out-dated. Please see the manual for intructions on how to update." << std::endl;
				return false;
			}
			vout << "Device firmware version: v" << mab::getVersionString(candleDeviceVersion) << std::endl;
		}
		return true;
	}
	return false;
}

bool Candle::configMd80TorqueBandwidth(uint16_t canId, uint16_t torqueBandwidth)
{
	if (inUpdateMode() || !md80Register->write(canId, Md80Reg_E::motorTorgueBandwidth, torqueBandwidth,
											   Md80Reg_E::runCalibratePiGains, true))
	{
		vout << "Bandwidth change failed at ID: " << canId << statusFAIL << std::endl;
		return false;
	}
	vout << "Bandwidth succesfully changed at ID: " << canId << statusOK << std::endl;
	return true;
}

Md80& Candle::getMd80FromList(uint16_t id)
{
	for (auto& md : md80s)
		if (md.getId() == id)
			return md;
	throw "getMd80FromList(id): Id not found on the list!";
}
bool Candle::controlMd80SetEncoderZero(Md80& drive)
{
	return controlMd80SetEncoderZero(drive.getId());
}
bool Candle::controlMd80Enable(Md80& drive, bool enable)
{
	return controlMd80Enable(drive.getId(), enable);
}
bool Candle::controlMd80Mode(Md80& drive, Md80Mode_E mode)
{
	return controlMd80Mode(drive.getId(), mode);
}
bool Candle::controlMd80Mode(uint16_t canId, Md80Mode_E mode)
{
	Md80& drive = getMd80FromList(canId);

	if (inUpdateMode() || !md80Register->write(canId, Md80Reg_E::motionModeCommand, static_cast<uint8_t>(mode)))
	{
		vout << "Setting control mode failed at ID: " << canId << statusFAIL << std::endl;
		return false;
	}

	vout << "Setting control mode successful at ID: " << canId << statusOK << std::endl;
	drive.__setControlMode(mode);
	return true;
}
bool Candle::controlMd80Enable(uint16_t canId, bool enable)
{
	uint16_t controlword = enable ? 39 : 64;
	if (inUpdateMode() || !md80Register->write(canId, Md80Reg_E::state, controlword))
	{
		vout << "Enabling/Disabling failed at ID: " << canId << statusFAIL << std::endl;
		return false;
	}

	if (enable)
		vout << "Enabling successful at ID: " << canId << statusOK << std::endl;
	else
		vout << "Disabling successful at ID: " << canId << statusOK << std::endl;

	return true;
}
bool Candle::begin()
{
	if (mode == CANdleMode_E::UPDATE)
	{
		vout << "Cannot run 'begin', already in update mode." << statusFAIL << std::endl;
		return false;
	}

	if (sendBusFrame(BUS_FRAME_BEGIN, 10))
	{
		vout << "Beginnig auto update loop mode" << statusOK << std::endl;
		mode = CANdleMode_E::UPDATE;
		shouldStopTransmitter = false;
		shouldStopReceiver = false;
		msgsSent = 0;
		msgsReceived = 0;

		sem_init(&transmitted, 0, 0);
		sem_init(&received, 0, 0);

		if (bus->getType() != mab::BusType_E::SPI)
			receiverThread = std::thread(&Candle::receive, this);

		transmitterThread = std::thread(&Candle::transmit, this);

		return true;
	}
	vout << "Failed to begin auto update loop mode" << statusFAIL << std::endl;
	return false;
}
bool Candle::end()
{
	if (mode == CANdleMode_E::CONFIG)
		return false;

	shouldStopTransmitter = true;
	sem_post(&received);
	if (transmitterThread.joinable())
		transmitterThread.join();

	shouldStopReceiver = true;
	if (bus->getType() != mab::BusType_E::SPI)
	{
		sem_post(&transmitted);
		if (receiverThread.joinable())
			receiverThread.join();
	}

	bus->flushReceiveBuffer();

	if (sendBusFrame(BUS_FRAME_END, 100))
		mode = CANdleMode_E::CONFIG;

	for (auto& md : md80s)
		controlMd80Enable(md, false);

	vout << "Ending auto update loop mode" << (mode == CANdleMode_E::CONFIG ? statusOK : statusFAIL) << std::endl;

	return mode == CANdleMode_E::CONFIG ? true : false;
}
bool Candle::reset()
{
	return sendBusFrame(BUS_FRAME_RESET, 100);
}
bool Candle::inUpdateMode()
{
	return mode == CANdleMode_E::UPDATE;
}
void Candle::transmitNewStdFrame()
{
	char tx[1 + sizeof(StdMd80CommandFrame_t) * maxDevices];
	tx[0] = BUS_FRAME_UPDATE;

	for (size_t i = 0; i < md80s.size(); i++)
	{
		md80s[i].__updateCommandFrame();
		*(StdMd80CommandFrame_t*)&tx[1 + i * sizeof(StdMd80CommandFrame_t)] = md80s[i].__getCommandFrame();
	}

	uint32_t cmdSize = 1 + md80s.size() * sizeof(StdMd80CommandFrame_t);
	uint32_t respSize = 1 + md80s.size() * sizeof(StdMd80ResponseFrame_t);

	if (bus->getType() == BusType_E::SPI)
		bus->transfer(tx, cmdSize, respSize);
	else
		bus->transmit(tx, cmdSize, false, 100, respSize);
}

bool Candle::setupMd80Calibration(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runCalibrateCmd, "Starting calibration failed at ID: ", "Starting calibration at ID: ");
}

bool Candle::setupMd80CalibrationOutput(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runCalibrateOutpuEncoderCmd, "Starting output encoder calibration failed at ID: ", "Starting output encoder calibration at ID: ");
}

bool Candle::setupMd80TestOutputEncoder(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runTestOutputEncoderCmd, "Output encoder test failed at ID: ", "Output encoder test in progress at ID: ");
}

bool Candle::setupMd80TestMainEncoder(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runTestMainEncoderCmd, "Main encoder test failed at ID: ", "Main encoder test in progress at ID: ");
}

bool Candle::setupMd80PerformHoming(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runHoming, "Homing test failed at ID: ", "Homing test in progress at ID: ");
}

bool Candle::setupMd80PerformReset(uint16_t canId)
{
	return executeCommand(canId, Md80Reg_E::runReset, "Reset failed at ID: ", "Reset in progress at ID: ");
}

bool Candle::setupMd80ClearErrors(uint16_t canId)
{
	return !inUpdateMode() && md80Register->write(canId, Md80Reg_E::runClearErrors, true);
}

bool Candle::setupMd80ClearWarnings(uint16_t canId)
{
	return !inUpdateMode() && md80Register->write(canId, Md80Reg_E::runClearWarnings, true);
}

bool Candle::setupMd80DiagnosticExtended(uint16_t canId)
{
	regRead_st& regR = getMd80FromList(canId).getReadReg();

	if (inUpdateMode() || !md80Register->read(canId,
											  Md80Reg_E::motorName, regR.RW.motorName,
											  Md80Reg_E::buildDate, regR.RO.buildDate,
											  Md80Reg_E::commitHash, regR.RO.commitHash,
											  Md80Reg_E::firmwareVersion, regR.RO.firmwareVersion,
											  Md80Reg_E::motorResistance, regR.RO.resistance,
											  Md80Reg_E::motorInductance, regR.RO.inductance))
	{
		vout << "Extended diagnostic failed at ID: " << canId << std::endl;
		return false;
	}

	if (!md80Register->read(canId,
							Md80Reg_E::motorIMax, regR.RW.iMax,
							Md80Reg_E::motorPolePairs, regR.RW.polePairs,
							Md80Reg_E::motorKt, regR.RW.motorKt,
							Md80Reg_E::motorGearRatio, regR.RW.gearRatio,
							Md80Reg_E::bridgeType, regR.RO.bridgeType,
							Md80Reg_E::canWatchdog, regR.RW.canWatchdog,
							Md80Reg_E::motorTorgueBandwidth, regR.RW.torqueBandwidth,
							Md80Reg_E::canBaudrate, regR.RW.canBaudrate,
							Md80Reg_E::quickStatus, regR.RO.quickStatus,
							Md80Reg_E::mosfetTemperature, regR.RO.mosfetTemperature,
							Md80Reg_E::motorKV, regR.RW.motorKV,
							Md80Reg_E::hardwareVersion, regR.RO.hardwareVersion))
	{
		vout << "Extended diagnostic failed at ID: " << canId << std::endl;
		return false;
	}

	if (!md80Register->read(canId,
							Md80Reg_E::motorStiction, regR.RW.stiction,
							Md80Reg_E::motorFriction, regR.RW.friction,
							Md80Reg_E::outputEncoder, regR.RW.outputEncoder,
							Md80Reg_E::outputEncoderDir, regR.RW.outputEncoderDir,
							Md80Reg_E::outputEncoderDefaultBaud, regR.RW.outputEncoderDefaultBaud,
							Md80Reg_E::motorTemperature, regR.RO.motorTemperature,
							Md80Reg_E::motorShutdownTemp, regR.RW.motorShutdownTemp,
							Md80Reg_E::canTermination, regR.RW.canTermination,
							Md80Reg_E::outputEncoderPosition, regR.RO.outputEncoderPosition,
							Md80Reg_E::outputEncoderVelocity, regR.RO.outputEncoderVelocity))
	{
		vout << "Extended diagnostic failed at ID: " << canId << std::endl;
		return false;
	}

	if (!md80Register->read(canId,
							Md80Reg_E::outputEncoderMode, regR.RW.outputEncoderMode,
							Md80Reg_E::calOutputEncoderStdDev, regR.RO.calOutputEncoderStdDev,
							Md80Reg_E::calOutputEncoderMinE, regR.RO.calOutputEncoderMinE,
							Md80Reg_E::calOutputEncoderMaxE, regR.RO.calOutputEncoderMaxE,
							Md80Reg_E::calMainEncoderStdDev, regR.RO.calMainEncoderStdDev,
							Md80Reg_E::calMainEncoderMinE, regR.RO.calMainEncoderMinE,
							Md80Reg_E::calMainEncoderMaxE, regR.RO.calMainEncoderMaxE))
	{
		vout << "Extended diagnostic failed at ID: " << canId << std::endl;
		return false;
	}

	if (!md80Register->read(canId,
							Md80Reg_E::mainEncoderErrors, regR.RO.mainEncoderErrors,
							Md80Reg_E::outputEncoderErrors, regR.RO.outputEncoderErrors,
							Md80Reg_E::calibrationErrors, regR.RO.calibrationErrors,
							Md80Reg_E::bridgeErrors, regR.RO.bridgeErrors,
							Md80Reg_E::hardwareErrors, regR.RO.hardwareErrors,
							Md80Reg_E::communicationErrors, regR.RO.communicationErrors,
							Md80Reg_E::motionErrors, regR.RO.motionErrors))
	{
		vout << "Extended diagnostic failed at ID: " << canId << std::endl;
		return false;
	}

	if (!md80Register->read(canId, Md80Reg_E::outputEncoderCalibrationMode, regR.RW.outputEncoderCalibrationMode,
							Md80Reg_E::brakeMode, regR.RW.brakeMode))
	{
		vout << "Extended diagnostic failed at ID: " << canId << " while reading outputEncoderCalibrationMode register" << std::endl;
		return false;
	}
	if (!md80Register->read(canId, Md80Reg_E::motorCalibrationMode, regR.RW.motorCalibrationMode))
	{
		vout << "Extended diagnostic failed at ID: " << canId << " while reading motorCalibrationMode register" << std::endl;
		return false;
	}
	if (!md80Register->read(canId, Md80Reg_E::shuntResistance, regR.RO.shuntResistance))
	{
		vout << "Extended diagnostic failed at ID: " << canId << " while reading shuntResistance register" << std::endl;
		return false;
	}

	if (!md80Register->read(canId, Md80Reg_E::homingMode, regR.RW.homingMode,
							Md80Reg_E::homingMaxTravel, regR.RW.homingMaxTravel,
							Md80Reg_E::homingTorque, regR.RW.homingTorque,
							Md80Reg_E::homingVelocity, regR.RW.homingVelocity,
							Md80Reg_E::homingErrors, regR.RO.homingErrors))
	{
		vout << "Extended diagnostic failed at ID: " << canId << " while reading homing registers" << std::endl;
		return false;
	}

	if (!md80Register->read(canId, Md80Reg_E::positionLimitMin, regR.RW.positionLimitMin,
							Md80Reg_E::positionLimitMax, regR.RW.positionLimitMax))
	{
		vout << "Extended diagnostic failed at ID: " << canId << " while reading position limits registers" << std::endl;
		return false;
	}

	if (!md80Register->read(canId, Md80Reg_E::maxAcceleration, regR.RW.maxAcceleration,
							Md80Reg_E::maxDeceleration, regR.RW.maxDeceleration,
							Md80Reg_E::maxTorque, regR.RW.maxTorque,
							Md80Reg_E::maxVelocity, regR.RW.maxVelocity))
	{
		vout << "Extended diagnostic failed at ID: " << canId << " while reading motion limit registers" << std::endl;
		return false;
	}

	if (!md80Register->read(canId, Md80Reg_E::profileAcceleration, regR.RW.profileAcceleration,
							Md80Reg_E::profileDeceleration, regR.RW.profileDeceleration,
							Md80Reg_E::quickStopDeceleration, regR.RW.quickStopDeceleration,
							Md80Reg_E::profileVelocity, regR.RW.profileVelocity))
	{
		vout << "Extended diagnostic failed at ID: " << canId << " while reading acceleration control data registers" << std::endl;
		return false;
	}

	return true;
}
mab::CANdleBaudrate_E Candle::getCurrentBaudrate()
{
	return canBaudrate;
}
bool Candle::checkMd80ForBaudrate(uint16_t canId)
{
	uint16_t status;
	return md80Register->read(canId, Md80Reg_E::quickStatus, status);
}

std::string getVersionString(const version_ut& ver)
{
	if (ver.s.tag == 'r' || ver.s.tag == 'R')
		return std::string(std::to_string(ver.s.major) + '.' + std::to_string(ver.s.minor) + '.' + std::to_string(ver.s.revision));
	else
		return std::string(std::to_string(ver.s.major) + '.' + std::to_string(ver.s.minor) + '.' + std::to_string(ver.s.revision) + '.' + ver.s.tag);
}

bool Candle::executeCommand(uint16_t canId, Md80Reg_E reg, const char* failMsg, const char* successMsg)
{
	if (inUpdateMode() || !md80Register->write(canId, reg, true))
	{
		vout << failMsg << canId << statusFAIL << std::endl;
		return false;
	}
	vout << successMsg << canId << statusOK << std::endl;
	return true;
}

bool Candle::sendBusFrame(BusFrameId_t id, uint32_t timeout, char* payload, uint32_t cmdLen, uint32_t respLen)
{
	char tx[128]{};
	tx[0] = id;
	tx[1] = 0x00;

	if (payload)
		memcpy(&tx[1], payload, cmdLen - 1);

	char* rx = bus->getRxBuffer(0);

	if (bus->transmit(tx, cmdLen, true, timeout, respLen))
		return ((rx[0] == id && rx[1] == true) || (rx[0] == BUS_FRAME_PING_START));
	return false;
}

}  // namespace mab
