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

Candle::Candle(CANdleBaudrate_E canBaudrate, bool printVerbose, mab::Bus* bus)
	: printVerbose(printVerbose), bus(bus)
{
	vout << "CANdle library version: " << getVersion() << std::endl;

	this->reset();
	usleep(5000);

	if (!configCandleBaudrate(canBaudrate, true))
	{
		vout << "Failed to set up CANdle baudrate @" << canBaudrate << "Mbps!" << std::endl;
		return;
	}

	if (bus->getType() == mab::BusType_E::USB)
		vout << "CANdle at " << bus->getDeviceName() << ", ID: 0x" << std::hex << this->getDeviceId() << std::dec << " ready (USB)" << std::endl;
	else if (bus->getType() == mab::BusType_E::SPI)
		vout << "CANdle ready (SPI)" << std::endl;
	else if (bus->getType() == mab::BusType_E::UART)
		vout << "CANdle ready (UART)" << std::endl;

	md80Register = new Register(this);

	Candle::instances.push_back(this);
}

Candle::~Candle()
{
	if (this->inUpdateMode())
		this->end();
}

Bus* Candle::makeBus(mab::BusType_E busType, std::string device)
{
	switch (busType)
	{
		case mab::BusType_E::USB:
		{
			std::vector<unsigned long> a;
			for (auto& instance : instances)
				a.push_back(instance->getDeviceId());
			return new UsbDevice("MAB_Robotics", "MD_USB-TO-CAN", a);
		}
		case mab::BusType_E::SPI:
		{
			if (device != "")
				return new SpiDevice(device);
			else
				return new SpiDevice();
		}

		case mab::BusType_E::UART:
		{
			if (device != "")
				return new UartDevice(device);
			else
				return new UartDevice();
		}
		default:
			throw "Error wrong bus type specified!";
	}
	return nullptr;
}

const std::string Candle::getVersion()
{
	return version;
}
int Candle::getActualCommunicationFrequency()
{
	return (int)this->usbCommsFreq;
}

void Candle::setTransmitDelayUs(uint32_t delayUs)
{
	if (delayUs < 20) delayUs = 20;
	transmitterDelay = delayUs;
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
			this->usbCommsFreq = 250.0 / (float)(getTimestamp() - freqCheckStart) * 1000000.0f;
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

#if BENCHMARKING == 1
		long long timestampStart = getTimestamp();
#if BENCHMARKING_VERBOSE == 1
		if (!flag_glob_rx) std::cout << "TX:" << timestampStart << std::endl;
#endif
		if (flag_glob_tx == true)
		{
			flag_glob_tx = false;
			txTimestamp = timestampStart;
		}
#endif
		msgsSent++;

		if (bus->getType() == mab::BusType_E::SPI)
		{
			for (int i = 1; i < (int)md80s.size(); i++)
				usleep(20);
		}
		/* wait for a frame to be received */
		else
			sem_wait(&received);
		usleep(transmitterDelay);
	}
}

void Candle::manageReceivedFrame()
{
#if BENCHMARKING == 1
	bool flag = false;
#endif
	for (int i = 0; i < (int)md80s.size(); i++)
	{
		md80s[i].__updateResponseData((StdMd80ResponseFrame_t*)bus->getRxBuffer(1 + i * sizeof(StdMd80ResponseFrame_t)));
#if BENCHMARKING == 1
		StdMd80ResponseFrame_t* _responseFrame = (StdMd80ResponseFrame_t*)bus->getRxBuffer(1 + i * sizeof(StdMd80ResponseFrame_t));
		if (*(uint16_t*)&_responseFrame->fromMd80.data[1] & (1 << 15)) flag = true;
#endif
	}
#if BENCHMARKING == 1
	long long timestampStart = getTimestamp();
#if BENCHMARKING_VERBOSE == 1
	if (!flag || !flag_glob_rx) std::cout << "RX:" << timestampStart << std::endl;
#endif

	if (flag && !flag_glob_rx)
	{
		flag_glob_rx = true;
		time_delta = timestampStart - txTimestamp;
		std::cout << "got the flag!"
				  << " time:" << time_delta << std::endl;
	}
#endif
}

void Candle::setVebose(bool enable)
{
	printVerbose = enable;
}
unsigned long Candle::getDeviceId()
{
	return bus->getId();
}
GenericMd80Frame32 _packMd80Frame(int canId, int msgLen, Md80FrameId_E canFrameId)
{
	GenericMd80Frame32 frame;
	frame.frameId = BUS_FRAME_MD80_GENERIC_FRAME;
	frame.driveCanId = canId;
	frame.canMsgLen = msgLen;
	frame.timeoutMs = 10;
	frame.canMsg[0] = canFrameId;
	frame.canMsg[1] = 0x00;
	return frame;
}
void Candle::sendGetInfoFrame(mab::Md80& drive)
{
	GenericMd80Frame32 getInfoFrame = _packMd80Frame(drive.getId(), 2, Md80FrameId_E::FRAME_GET_INFO);
	if (bus->transmit((char*)&getInfoFrame, sizeof(getInfoFrame), true, 100, 66))
	{
		uint8_t cheaterBuffer[72];
		memcpy(&cheaterBuffer[1], bus->getRxBuffer(), bus->getBytesReceived());
		*(uint16_t*)&cheaterBuffer[0] = drive.getId();
		cheaterBuffer[2] = 16;	// Cheater buffer is a dirty trick to make BUS_FRAME_MD80_GENERIC_FRAME response compatibile with __updateResponseData
		drive.__updateResponseData((mab::StdMd80ResponseFrame_t*)cheaterBuffer);
	}
}
void Candle::sendMotionCommand(mab::Md80& drive, float pos, float vel, float torque)
{
	GenericMd80Frame32 motionCommandFrame = _packMd80Frame(drive.getId(), 16, Md80FrameId_E::FRAME_SET_MOTION_TARGETS);
	*(float*)&motionCommandFrame.canMsg[2] = vel;
	*(float*)&motionCommandFrame.canMsg[6] = pos;
	*(float*)&motionCommandFrame.canMsg[10] = torque;
	if (bus->transmit((char*)&motionCommandFrame, sizeof(motionCommandFrame), true, 100, 66))
	{
		uint8_t cheaterBuffer[72];
		memcpy(&cheaterBuffer[1], bus->getRxBuffer(), bus->getBytesReceived());
		*(uint16_t*)&cheaterBuffer[0] = drive.getId();
		cheaterBuffer[2] = 16;	// Cheater buffer is a dirty trick to make BUS_FRAME_MD80_GENERIC_FRAME response compatibile with __updateResponseData
		drive.__updateResponseData((mab::StdMd80ResponseFrame_t*)cheaterBuffer);
	}
}
bool Candle::addMd80(uint16_t canId, bool printFailure)
{
	if (inUpdateMode())
		return false;
	for (auto& d : md80s)
		if (d.getId() == canId)
		{
			vout << "MD80 with ID: " << canId << " is already on the update list." << statusOK << std::endl;
			return true;
		}
	if ((int)md80s.size() >= maxDevices)
	{
		vout << "Cannot add more drives in current FAST_MODE. Max devices in current mode: " << maxDevices << statusFAIL << std::endl;
		return false;
	}
	AddMd80Frame_t add = {BUS_FRAME_MD80_ADD, canId};
	if (bus->transmit((char*)&add, sizeof(AddMd80Frame_t), true, 2, 2))
		if (*bus->getRxBuffer(0) == BUS_FRAME_MD80_ADD)
			if (*bus->getRxBuffer(1) == true)
			{
				uint32_t firmwareVersion = 0;
				if (!md80Register->read(canId, mab::Md80Reg_E::firmwareVersion, firmwareVersion))
				{
					vout << "Unable to read MD80's firmware version! Probably MD80's firmware is outdated. Please update it using MAB_CAN_Flasher." << statusFAIL << std::endl;
					return false;
				}

				if (firmwareVersion / 10 < md80CompatibleMajorVersion)
				{
					vout << "MD80's firmware with ID: " + std::to_string(canId) + " is outdated. Please update it using MAB_CAN_Flasher." << statusFAIL << std::endl;
					return false;
				}
				else if (firmwareVersion / 10 > md80CompatibleMajorVersion)
				{
					vout << "MD80's firmware with ID: " + std::to_string(canId) + " is a future version. Please update your CANdle library." << statusFAIL << std::endl;
					return false;
				}
				vout << "Added MD80 with ID: " + std::to_string(canId) << statusOK << std::endl;
				md80s.push_back(Md80(canId));
				mab::Md80& newDrive = md80s.back();
				sendGetInfoFrame(newDrive);
				sendMotionCommand(newDrive, newDrive.getPosition(), 0.0f, 0.0f);
				newDrive.setTargetPosition(newDrive.getPosition());
				return true;
			}
	if (printFailure) vout << "Failed to add MD80 with ID: " + std::to_string(canId) << statusFAIL << std::endl;
	return false;
}
std::vector<uint16_t> Candle::ping(mab::CANdleBaudrate_E baudrate)
{
	if (!this->configCandleBaudrate(baudrate))
		return std::vector<uint16_t>();
	vout << "Starting pinging drives at baudrate: " << baudrate << "M" << std::endl;
	char tx[128];
	tx[0] = BUS_FRAME_PING_START;
	tx[1] = 0x00;
	std::vector<uint16_t> ids;
	if (bus->transmit(tx, 2, true, 2000, 33))
	{
		uint16_t* idsPointer = (uint16_t*)bus->getRxBuffer(1);
		for (int i = 0; i < 12; i++)
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
bool Candle::sengGenericFDCanFrame(uint16_t canId, int msgLen, const char* txBuffer, char* rxBuffer, int timeoutMs)
{
	int fdcanTimeout = timeoutMs - 3;
	if (timeoutMs < 3)
	{
		timeoutMs = 3;
		fdcanTimeout = 1;
	}
	GenericMd80Frame64 frame;
	frame.frameId = mab::BusFrameId_t::BUS_FRAME_MD80_GENERIC_FRAME;
	frame.driveCanId = canId;
	frame.canMsgLen = msgLen;
	frame.timeoutMs = fdcanTimeout;
	memcpy(frame.canMsg, txBuffer, msgLen);
	char tx[96];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, timeoutMs, 66))  // Got some response
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

	GenericMd80Frame32 frame = _packMd80Frame(canId, 11, Md80FrameId_E::FRAME_CAN_CONFIG);
	frame.frameId = BUS_FRAME_MD80_CONFIG_CAN;
	*(uint16_t*)&frame.canMsg[2] = newId;
	*(uint32_t*)&frame.canMsg[4] = newBaudrateMbps * 1000000;
	*(uint16_t*)&frame.canMsg[8] = newTimeout;
	*(uint8_t*)&frame.canMsg[10] = (uint8_t)canTermination == true ? 1 : 0;

	char tx[63];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 100, 2))
		if (*bus->getRxBuffer(1) == 1)
		{
			vout << "CAN config change successful!" << statusOK << std::endl;
			vout << "Drive ID: " << std::to_string(canId) << " was changed to ID: " << std::to_string(newId) << std::endl;
			vout << "It's baudrate is now " << std::to_string(newBaudrateMbps) << "Mbps" << std::endl;
			vout << "It's CAN timeout (watchdog) is now " << (newTimeout == 0 ? "Disabled" : std::to_string(newTimeout) + "ms") << std::endl;
			vout << "It's CAN termination resistor is " << (canTermination == true ? "enabled" : "disabled") << std::endl;
			return true;
		}
	vout << "CAN config change failed!" << statusFAIL << std::endl;
	return false;
}
bool Candle::configMd80Save(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_CAN_SAVE);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 500, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			vout << "Saving in flash successful at ID: " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Saving in flash failed at ID: " << canId << statusFAIL << std::endl;
	return false;
}
bool Candle::configMd80Blink(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_FLASH_LED);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 500, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			vout << "LEDs blining at ID: " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Blinking failed at ID: " << canId << statusFAIL << std::endl;
	return false;
}

bool Candle::controlMd80SetEncoderZero(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_ZERO_ENCODER);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 50, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			/* set target position to 0.0f to avoid jerk at startup */
			Md80& drive = getMd80FromList(canId);
			sendMotionCommand(drive, 0.0f, 0.0f, 0.0f);
			drive.setTargetPosition(0.0f);
			vout << "Setting new zero position successful at ID: " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Setting new zero position failed at ID: " << canId << statusFAIL << std::endl;
	return false;
}
bool Candle::configMd80SetCurrentLimit(uint16_t canId, float currentLimit)
{
	if (currentLimit > driverMaxCurrent)
	{
		vout << "Current setting above limit (" << driverMaxCurrent << " A)! Setting current limit to maximum (" << driverMaxCurrent << " A)" << std::endl;
		currentLimit = driverMaxCurrent;
	}
	else if (currentLimit < driverMinCurrent)
	{
		vout << "Current setting below limit (" << driverMinCurrent << " A)! Setting current limit to minimum (" << driverMinCurrent << " A)" << std::endl;
		currentLimit = driverMinCurrent;
	}

	GenericMd80Frame32 frame = _packMd80Frame(canId, 6, Md80FrameId_E::FRAME_BASE_CONFIG);
	*(float*)&frame.canMsg[2] = currentLimit;
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 50, 66))
		if (*bus->getRxBuffer(0) == BUS_FRAME_MD80_GENERIC_FRAME && *bus->getRxBuffer(1) == true)
		{
			vout << "Setting new current limit successful at ID: " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Setting new current limit failed at ID: " << canId << statusFAIL << std::endl;
	return false;
}

bool Candle::configCandleBaudrate(CANdleBaudrate_E canBaudrate, bool printVersionInfo)
{
	this->canBaudrate = canBaudrate;
	char tx[10];
	tx[0] = BUS_FRAME_CANDLE_CONFIG_BAUDRATE;
	tx[1] = (uint8_t)canBaudrate;
	if (bus->transmit(tx, 2, true, 50, 3))
		if (*bus->getRxBuffer(0) == BUS_FRAME_CANDLE_CONFIG_BAUDRATE && *bus->getRxBuffer(1) == true)
		{
			candleDeviceVersion = *bus->getRxBuffer(2);
			if (printVersionInfo)
			{
				vout << "Device firmware version: v" << candleDeviceVersion / 10 << "." << candleDeviceVersion % 10 << std::endl;
				if (candleDeviceVersion < candleCompatibleVersion)
					std::cout << "Your CANdle firmware seems to be out-dated. Contact MAB: support@mabrobotics.pl , for intructions how to update." << std::endl;
			}
			return true;
		}
	return false;
}

bool Candle::configMd80TorqueBandwidth(uint16_t canId, uint16_t torqueBandwidth)
{
	if (torqueBandwidth > driverMaxBandwidth)
	{
		vout << "Bandwidth setting above limit (" << driverMaxBandwidth << " Hz)! Setting bandwidth to maximum (" << driverMaxBandwidth << " Hz)" << std::endl;
		torqueBandwidth = driverMaxBandwidth;
	}
	else if (torqueBandwidth < driverMinBandwidth)
	{
		vout << "Bandwidth setting below limit (" << driverMinBandwidth << " Hz)! Setting bandwidth to minimum (" << driverMinBandwidth << " Hz)" << std::endl;
		torqueBandwidth = driverMinBandwidth;
	}

	GenericMd80Frame32 frame = _packMd80Frame(canId, 4, Md80FrameId_E::FRAME_SET_BANDWIDTH);
	char tx[64];
	frame.canMsg[2] = (uint8_t)(torqueBandwidth & 0xff);
	frame.canMsg[3] = (uint8_t)(torqueBandwidth >> 8);
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 500, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			vout << "Bandwidth succesfully changed at ID: " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Bandwidth change failed at ID: " << canId << statusFAIL << std::endl;
	return false;
}

Md80& Candle::getMd80FromList(uint16_t id)
{
	for (int i = 0; i < (int)md80s.size(); i++)
		if (md80s[i].getId() == id)
			return md80s[i];
	throw "getMd80FromList(id): Id not found on the list!";
}
bool Candle::controlMd80SetEncoderZero(Md80& drive)
{
	return this->controlMd80SetEncoderZero(drive.getId());
}
bool Candle::controlMd80Enable(Md80& drive, bool enable)
{
	return this->controlMd80Enable(drive.getId(), enable);
}
bool Candle::controlMd80Mode(Md80& drive, Md80Mode_E mode)
{
	return this->controlMd80Mode(drive.getId(), mode);
}
bool Candle::controlMd80Mode(uint16_t canId, Md80Mode_E mode)
{
	if (mode == DEPRECATED)
	{
		vout << "This control mode is DEPRECATED. Please do not use it! " << statusFAIL << std::endl;
		return false;
	}
	try
	{
		Md80& drive = getMd80FromList(canId);
		GenericMd80Frame32 frame = _packMd80Frame(canId, 3, Md80FrameId_E::FRAME_CONTROL_SELECT);
		frame.canMsg[2] = mode;
		char tx[64];
		int len = sizeof(frame);
		memcpy(tx, &frame, len);
		if (bus->transmit(tx, len, true, 50, 66))
			if (*bus->getRxBuffer(1) == true)
			{
				vout << "Setting control mode successful at ID: " << canId << statusOK << std::endl;
				drive.__setControlMode(mode);
				return true;
			}
		vout << "Setting control mode failed at ID: " << canId << statusFAIL << std::endl;
		return false;
	}
	catch (const char* msg)
	{
		vout << msg << std::endl;
		return false;
	}
}
bool Candle::controlMd80Enable(uint16_t canId, bool enable)
{
	try
	{
		GenericMd80Frame32 frame = _packMd80Frame(canId, 3, Md80FrameId_E::FRAME_MOTOR_ENABLE);
		frame.canMsg[2] = enable;
		char tx[64];
		int len = sizeof(frame);
		memcpy(tx, &frame, len);
		if (bus->transmit(tx, len, true, 50, 66))
			if (*bus->getRxBuffer(1) == true)
			{
				if (enable)
					vout << "Enabling successful at ID: " << canId << statusOK << std::endl;
				else
				{
					vout << "Disabling successful at ID: " << canId << statusOK << std::endl;
					this->getMd80FromList(canId).__updateRegulatorsAdjusted(false);	 // Drive will operate at default params
				}
				return true;
			}
		vout << "Enabling/Disabling failed at ID: " << canId << statusFAIL << std::endl;
		return false;
	}
	catch (const char* msg)
	{
		vout << msg << std::endl;
		return false;
	}
}
bool Candle::begin()
{
	if (mode == CANdleMode_E::UPDATE)
	{
		vout << "Cannot run 'begin', already in update mode." << statusFAIL << std::endl;
		return false;
	}
	char tx[128];
	tx[0] = BUS_FRAME_BEGIN;
	tx[1] = 0x00;
	if (bus->transmit(tx, 2, true, 10, 2))
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
	if (transmitterThread.joinable())
		transmitterThread.join();

	shouldStopReceiver = true;
	/* this is to make the receiver thread exit cleanly */
	sem_post(&transmitted);
	sem_post(&received);

	if (bus->getType() != mab::BusType_E::SPI)
	{
		if (receiverThread.joinable())
			receiverThread.join();
	}

	char tx[128];
	tx[0] = BUS_FRAME_END;
	tx[1] = 0x00;

	bus->flushReceiveBuffer();

	if (bus->transmit(tx, 2, true, 100, 2))
		if (*bus->getRxBuffer(0) == BUS_FRAME_END && *bus->getRxBuffer(1) == 1)
			mode = CANdleMode_E::CONFIG;

	vout << "Ending auto update loop mode" << (mode == CANdleMode_E::CONFIG ? statusOK : statusFAIL) << std::endl;

	return mode == CANdleMode_E::CONFIG ? true : false;
}
bool Candle::reset()
{
	char tx[128];
	tx[0] = BUS_FRAME_RESET;
	tx[1] = 0x00;
	if (bus->transmit(tx, 2, true, 100, 2))
		return true;

	return false;
}
bool Candle::inUpdateMode()
{
	if (mode == CANdleMode_E::UPDATE)
		return true;
	return false;
}
bool Candle::inConfigMode()
{
	if (mode == CANdleMode_E::CONFIG)
		return true;
	return false;
}
void Candle::transmitNewStdFrame()
{
	char tx[512];
	tx[0] = BUS_FRAME_UPDATE;
	for (int i = 0; i < (int)md80s.size(); i++)
	{
		md80s[i].__updateCommandFrame();
		*(StdMd80CommandFrame_t*)&tx[1 + i * sizeof(StdMd80CommandFrame_t)] = md80s[i].__getCommandFrame();
	}

	int length = 1 + md80s.size() * sizeof(StdMd80CommandFrame_t);

	if (bus->getType() == BusType_E::SPI)
		bus->transfer(tx, length, sizeof(StdMd80ResponseFrame_t) * md80s.size() + 1);
	else
		bus->transmit(tx, length, false, 100, sizeof(StdMd80ResponseFrame_t) * md80s.size() + 1);
}

bool Candle::setupMd80Calibration(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_CALIBRATION);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 50, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			vout << "Starting calibration at ID: " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Starting calibration failed at ID: " << canId << statusFAIL << std::endl;
	return false;
}

/* legacy */
bool Candle::setupMd80Diagnostic(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_DIAGNOSTIC);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 50, 66))
	{
		vout << "Library version: " << getVersion() << std::endl;
		vout << "DIAG at ID: " << canId << ": " << std::string(bus->getRxBuffer(2)) << std::endl;
		return true;
	}
	vout << "Diagnostic failed at ID: " << canId << std::endl;
	return false;
}
bool Candle::setupMd80DiagnosticExtended(uint16_t canId)
{
	regRead_st& regR = getMd80FromList(canId).getReadReg();

	if (!md80Register->read(canId,
							mab::Md80Reg_E::motorName, regR.RW.motorName,
							mab::Md80Reg_E::buildDate, regR.RO.buildDate,
							mab::Md80Reg_E::commitHash, regR.RO.commitHash,
							mab::Md80Reg_E::firmwareVersion, regR.RO.firmwareVersion,
							mab::Md80Reg_E::motorResistance, regR.RO.resistance,
							mab::Md80Reg_E::motorInductance, regR.RO.inductance))
	{
		vout << "Extended diagnostic failed at ID: " << canId << std::endl;
		return false;
	}

	if (!md80Register->read(canId,
							mab::Md80Reg_E::motorIMax, regR.RW.iMax,
							mab::Md80Reg_E::motorPolePairs, regR.RW.polePairs,
							mab::Md80Reg_E::motorKt, regR.RW.motorKt,
							mab::Md80Reg_E::motorGearRatio, regR.RW.gearRatio,
							mab::Md80Reg_E::bridgeType, regR.RO.bridgeType,
							mab::Md80Reg_E::canWatchdog, regR.RW.canWatchdog,
							mab::Md80Reg_E::motorTorgueBandwidth, regR.RW.torqueBandwidth,
							mab::Md80Reg_E::canBaudrate, regR.RW.canBaudrate,
							mab::Md80Reg_E::errorVector, regR.RO.errorVector,
							mab::Md80Reg_E::mosfetTemperature, regR.RO.mosfetTemperature,
							mab::Md80Reg_E::motorKV, regR.RW.motorKV,
							mab::Md80Reg_E::hardwareVersion, regR.RO.hardwareVersion))
	{
		vout << "Extended diagnostic failed at ID: " << canId << std::endl;
		return false;
	}

	if (!md80Register->read(canId,
							mab::Md80Reg_E::motorStiction, regR.RW.stiction,
							mab::Md80Reg_E::motorFriction, regR.RW.friction,
							mab::Md80Reg_E::outputEncoder, regR.RW.outputEncoder,
							mab::Md80Reg_E::outputEncoderDir, regR.RW.outputEncoderDir,
							mab::Md80Reg_E::outputEncoderDefaultBaud, regR.RW.outputEncoderDefaultBaud,
							mab::Md80Reg_E::motorTemperature, regR.RO.motorTemperature,
							mab::Md80Reg_E::motorShutdownTemp, regR.RW.motorShutdownTemp,
							mab::Md80Reg_E::canTermination, regR.RW.canTermination,
							mab::Md80Reg_E::outputEncoderPosition, regR.RO.outputEncoderPosition,
							mab::Md80Reg_E::outputEncoderVelocity, regR.RO.outputEncoderVelocity))
	{
		vout << "Extended diagnostic failed at ID: " << canId << std::endl;
		return false;
	}

	return true;
}
mab::CANdleBaudrate_E Candle::getCurrentBaudrate()
{
	return this->canBaudrate;
}
bool Candle::checkMd80ForBaudrate(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_GET_INFO);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transmit(tx, len, true, 300, 66, false))
		if (*bus->getRxBuffer(1) == true)
			return true;
	return false;
}

#if BENCHMARKING == 1
bool Candle::benchGetFlagRx()
{
	return flag_glob_rx;
}
bool Candle::benchGetFlagTx()
{
	return flag_glob_tx;
}
void Candle::benchSetFlagRx(bool state)
{
	flag_glob_rx = state;
}
void Candle::benchSetFlagTx(bool state)
{
	flag_glob_tx = state;
}
long long Candle::benchGetTimeDelta()
{
	return time_delta;
}
#endif

}  // namespace mab
