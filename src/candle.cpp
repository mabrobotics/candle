#include "candle.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include "candle_protocol.hpp"

namespace mab
{
class mystreambuf : public std::streambuf
{
};
mystreambuf nostreambuf;
std::ostream nocout(&nostreambuf);
#define vout ((this->printVerbose) ? std::cout << "[CANDLE] " : nocout)
std::string statusOK = "  [OK]";
std::string statusFAIL = "  [FAILED]";

uint64_t getTimestamp()
{
	using namespace std::chrono;
	return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::vector<Candle*> Candle::instances = std::vector<Candle*>();

Candle::Candle(CANdleBaudrate_E canBaudrate, bool _printVerbose, mab::CANdleFastMode_E _fastMode, bool printFailure, mab::BusType_E busType)
{
	bus = new Bus(busType);
	printVerbose = _printVerbose;

	if (bus->getType() == mab::BusType_E::USB)
	{
		auto listOfCANdle = UsbDevice::getConnectedACMDevices("MAB_Robotics", "MD_USB-TO-CAN");
		if (listOfCANdle.size() == 0)
		{
			vout << "No CANdle found!" << std::endl;
			delete (bus);
			throw "No Candle found!";
			return;
		}
		if (instances.size() == 0)
			bus->usb = new UsbDevice(listOfCANdle[0], "MAB_Robotics", "MD_USB-TO-CAN", bus->getRxBuffer(), bus->getRxBufferSize());
		else
		{
			for (auto& entry : listOfCANdle)
			{
				unsigned int newIdCount = 0;
				for (auto instance : instances)
				{
					if (UsbDevice::getConnectedDeviceId(entry) != instance->getUsbDeviceId())
						newIdCount++;
				}
				/* only if all instances were different from the current one -> create new device */
				if (newIdCount == instances.size())
				{
					bus->usb = new UsbDevice(entry, "MAB_Robotics", "MD_USB-TO-CAN", bus->getRxBuffer(), bus->getRxBufferSize());
					goto loopdone;	// Only legit use of goto left in C++
				}
			}
			if (printFailure)
				vout << "Failed to create CANdle object." << statusFAIL << std::endl;
			delete (bus);
			throw "Failed to create CANdle object";
			return;
		}
	loopdone:
		vout << "CANdle library version: " << getVersion() << std::endl;
		std::string setSerialCommand = "setserial " + bus->usb->getSerialDeviceName() + " low_latency";
		if (system(setSerialCommand.c_str()) != 0)
			std::cout << "Could not execute command '" << setSerialCommand << "'. Communication in low-speed mode." << std::endl;
	}
	else if (bus->getType() == mab::BusType_E::SPI)
	{
		bus->spi = new SpiDevice(bus->getRxBuffer(), bus->getRxBufferSize());
	}
	else if (bus->getType() == mab::BusType_E::UART)
	{
		bus->uart = new UartDevice(bus->getRxBuffer(), bus->getRxBufferSize());
	}

	this->reset();
	usleep(5000);

	if (!configCandleBaudrate(canBaudrate, true))
	{
		vout << "Failed to set up CANdle baudrate @" << canBaudrate << "Mbps!" << std::endl;
		return;
	}

	if (bus->getType() == mab::BusType_E::USB)
		vout << "CANdle at " << this->bus->usb->getSerialDeviceName() << ", ID: 0x" << std::hex << this->getUsbDeviceId() << std::dec << " ready (USB)" << std::endl;
	else if (bus->getType() == mab::BusType_E::SPI)
		vout << "CANdle ready (SPI)" << std::endl;
	else if (bus->getType() == mab::BusType_E::UART)
		vout << "CANdle ready (UART)" << std::endl;

	fastMode = _fastMode;
	switch (fastMode)
	{
		case CANdleFastMode_E::FAST1:
			maxDevices = (int)CANdleMaxDevices_E::MAX_DEV_FAST1;
			break;
		case CANdleFastMode_E::FAST2:
			maxDevices = (int)CANdleMaxDevices_E::MAX_DEV_FAST2;
			break;
		default:
			maxDevices = (int)CANdleMaxDevices_E::MAX_DEV_NORMAL;
			break;
	}
	Candle::instances.push_back(this);
}
Candle::~Candle()
{
	if (this->inUpdateMode())
		this->end();
	delete (bus);
}
void Candle::updateModeBasedOnMd80List()
{
	if (md80s.size() <= (int)CANdleMaxDevices_E::MAX_DEV_FAST2)
	{
		fastMode = CANdleFastMode_E::FAST2;
		vout << "Set current speed mode to FAST2" << std::endl;
	}
	else if (md80s.size() <= (int)CANdleMaxDevices_E::MAX_DEV_FAST1)
	{
		fastMode = CANdleFastMode_E::FAST1;
		vout << "Set current speed mode to FAST1" << std::endl;
	}
	else if (md80s.size() <= (int)CANdleMaxDevices_E::MAX_DEV_NORMAL)
	{
		fastMode = CANdleFastMode_E::NORMAL;
		vout << "Set current speed mode to NORMAL" << std::endl;
	}
}
const std::string Candle::getVersion()
{
	return version;
}
int Candle::getActualCommunicationFrequency()
{
	return (int)this->usbCommsFreq;
}

void Candle::receive()
{
	while (!shouldStopReceiver)
	{
		if (bus->receive())
		{
			if (*bus->getRxBuffer() == BUS_FRAME_UPDATE)
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
				long long microseconds_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
#if BENCHMARKING_VERBOSE == 1
				if (!flag || !flag_glob_rx) std::cout << "RX:" << microseconds_since_epoch << std::endl;
#endif

				if (flag && !flag_glob_rx)
				{
					flag_glob_rx = true;
					time_delta = microseconds_since_epoch - txTimestamp;
					std::cout << "got the flag!"
							  << " time:" << time_delta << std::endl;
				}
#endif
			}
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
			this->usbCommsFreq = 250.0 / (float)(getTimestamp() - freqCheckStart) * 1000.0f;
			freqCheckStart = getTimestamp();
			txCounter = 0;
		}
		transmitNewStdFrame();
		/* transmit thread is also the receive thread for SPI in update mode */
		if (bus->getType() == mab::BusType_E::SPI && *bus->getRxBuffer() == BUS_FRAME_UPDATE)
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

			long long microseconds_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
#if BENCHMARKING_VERBOSE == 1
			if (!flag && !flag_glob_rx) std::cout << "RX:" << microseconds_since_epoch << std::endl;
#endif

			if (flag && !flag_glob_rx)
			{
				flag_glob_rx = true;
				time_delta = microseconds_since_epoch - txTimestamp;
				std::cout << "got the flag!"
						  << " time:" << time_delta << std::endl;
			}
#endif
		}

#if BENCHMARKING == 1
		long long microseconds_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
#if BENCHMARKING_VERBOSE == 1
		if (!flag_glob_rx) std::cout << "TX:" << microseconds_since_epoch << std::endl;
#endif
		if (flag_glob_tx == true)
		{
			flag_glob_tx = false;
			txTimestamp = microseconds_since_epoch;
		}
#endif
		msgsSent++;
		switch (fastMode)
		{
			case CANdleFastMode_E::FAST1:
				if (bus->getType() == mab::BusType_E::SPI)
					usleep(200);
				else
					usleep(1990 * 2);
				break;
			case CANdleFastMode_E::FAST2:
				if (bus->getType() == mab::BusType_E::SPI)
					usleep(50);
				else
					usleep(1950);
				break;
			default:
				if (bus->getType() == mab::BusType_E::SPI)
					usleep(400);
				else
					usleep(10000);
				break;
		}
	}
}
void Candle::setVebose(bool enable)
{
	printVerbose = enable;
}
unsigned long Candle::getUsbDeviceId()
{
	return bus->usb->getId();
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
	if (bus->transfer((char*)&getInfoFrame, sizeof(getInfoFrame), true, 100, 66))
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
	if (bus->transfer((char*)&motionCommandFrame, sizeof(motionCommandFrame), true, 100, 66))
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
			vout << "Md80 with ID: " << canId << " is already on the update list." << statusOK << std::endl;
			return true;
		}
	if ((int)md80s.size() >= maxDevices)
	{
		vout << "Cannot add more drives in current FAST_MODE. Max devices in current mode: " << maxDevices << statusFAIL << std::endl;
		return false;
	}
	AddMd80Frame_t add = {BUS_FRAME_MD80_ADD, canId};
	if (bus->transfer((char*)&add, sizeof(AddMd80Frame_t), true, 2, 2))
		if (*bus->getRxBuffer(0) == BUS_FRAME_MD80_ADD)
			if (*bus->getRxBuffer(1) == true)
			{
				vout << "Added Md80." << statusOK << std::endl;
				md80s.push_back(Md80(canId));
				mab::Md80& newDrive = md80s.back();
				sendGetInfoFrame(newDrive);
				sendMotionCommand(newDrive, newDrive.getPosition(), 0.0f, 0.0f);
				newDrive.setTargetPosition(newDrive.getPosition());
				return true;
			}
	if (printFailure) vout << "Failed to add Md80." << statusFAIL << std::endl;
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
	if (bus->transfer(tx, 2, true, 2500, 33))  // Scanning 2047 FDCAN ids, takes ~2100ms, thus wait for 2.5 sec
	{
		uint16_t* idsPointer = (uint16_t*)&*bus->getRxBuffer(1);
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
			if (ids[i] > 2047)
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
	return ping(mab::CANdleBaudrate_E::CAN_BAUD_1M);
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
	if (bus->transfer(tx, len, true, timeoutMs, 66))  // Got some response
	{
		if (*bus->getRxBuffer(0) == tx[0] &&  // USB Frame ID matches
			*bus->getRxBuffer(1) == true &&
			bus->getBytesReceived() <= 64 + 2)	// response can ID matches
		{
			memcpy(rxBuffer, &*bus->getRxBuffer(2), bus->getBytesReceived() - 2);
			return true;
		}
	}
	return false;
}

bool Candle::configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 10, Md80FrameId_E::FRAME_CAN_CONFIG);
	frame.frameId = BUS_FRAME_MD80_CONFIG_CAN;
	*(uint16_t*)&frame.canMsg[2] = newId;
	*(uint32_t*)&frame.canMsg[4] = newBaudrateMbps * 1000000;
	*(uint16_t*)&frame.canMsg[8] = newTimeout;
	char tx[63];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transfer(tx, len, true, 100, 2))
		if (*bus->getRxBuffer(1) == 1)
		{
			vout << "CAN config change successful!" << statusOK << std::endl;
			vout << "Drive ID = " << std::to_string(canId) << " was changed to ID = " << std::to_string(newId) << std::endl;
			vout << "It's baudrate is now " << std::to_string(newBaudrateMbps) << "Mbps" << std::endl;
			vout << "It's CAN timeout (watchdog) is now " << (newTimeout == 0 ? "Disabled" : std::to_string(newTimeout) + "ms") << std::endl;
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
	if (bus->transfer(tx, len, true, 500, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			vout << "Saving in flash successful at ID = " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Saving in flash failed at ID = " << canId << statusFAIL << std::endl;
	return false;
}
bool Candle::configMd80Blink(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_FLASH_LED);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transfer(tx, len, true, 500, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			vout << "LEDs blining at ID = " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Blinking failed at ID = " << canId << statusFAIL << std::endl;
	return false;
}

bool Candle::controlMd80SetEncoderZero(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_ZERO_ENCODER);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transfer(tx, len, true, 50, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			/* set target position to 0.0f to avoid jerk at startup */
			Md80& drive = getMd80FromList(canId);
			sendMotionCommand(drive, 0.0f, 0.0f, 0.0f);
			drive.setTargetPosition(0.0f);
			vout << "Setting new zero position successful at ID = " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Setting new zero position failed at ID = " << canId << statusFAIL << std::endl;
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
	if (bus->transfer(tx, len, true, 50, 66))
		if (*bus->getRxBuffer(0) == BUS_FRAME_MD80_GENERIC_FRAME && *bus->getRxBuffer(1) == true)
		{
			vout << "Setting new current limit successful at ID = " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Setting new current limit failed at ID = " << canId << statusFAIL << std::endl;
	return false;
}

bool Candle::configCandleBaudrate(CANdleBaudrate_E canBaudrate, bool printVersionInfo)
{
	this->canBaudrate = canBaudrate;
	char tx[10];
	tx[0] = BUS_FRAME_CANDLE_CONFIG_BAUDRATE;
	tx[1] = (uint8_t)canBaudrate;
	if (bus->transfer(tx, 2, true, 50, 3))
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
		if (bus->transfer(tx, len, true, 50, 66))
			if (*bus->getRxBuffer(1) == true)
			{
				vout << "Setting control mode successful at ID = " << canId << statusOK << std::endl;
				drive.__setControlMode(mode);
				return true;
			}
		vout << "Setting control mode failed at ID = " << canId << statusFAIL << std::endl;
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
		if (bus->transfer(tx, len, true, 50, 66))
			if (*bus->getRxBuffer(1) == true)
			{
				if (enable)
					vout << "Enabling successful at ID = " << canId << statusOK << std::endl;
				else
				{
					vout << "Disabling successful at ID = " << canId << statusOK << std::endl;
					this->getMd80FromList(canId).__updateRegulatorsAdjusted(false);	 // Drive will operate at default params
				}
				return true;
			}
		vout << "Enabling/Disabling failed at ID = " << canId << statusFAIL << std::endl;
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
	if (bus->transfer(tx, 2, true, 10, 2))
	{
		vout << "Beginnig auto update loop mode" << statusOK << std::endl;
		mode = CANdleMode_E::UPDATE;
		shouldStopTransmitter = false;
		shouldStopReceiver = false;
		msgsSent = 0;
		msgsReceived = 0;
		transmitterThread = std::thread(&Candle::transmit, this);
		if (bus->getType() != mab::BusType_E::SPI)
			receiverThread = std::thread(&Candle::receive, this);
		return true;
	}
	vout << "Failed to begin auto update loop mode" << statusFAIL << std::endl;
	return false;
}
bool Candle::end()
{
	if (mode == CANdleMode_E::CONFIG)
		return false;

	if (bus->getType() != mab::BusType_E::SPI)
	{
		shouldStopReceiver = true;
		if (receiverThread.joinable())
			receiverThread.join();
	}

	shouldStopTransmitter = true;
	if (transmitterThread.joinable())
		transmitterThread.join();

	char tx[128];
	tx[0] = BUS_FRAME_END;
	tx[1] = 0x00;

	if (bus->getType() == mab::BusType_E::USB)
		bus->transfer(tx, 2, true, 10, 2);	// Stops update but produces garbage output

	if (bus->getType() == mab::BusType_E::UART)
	{
		bus->transfer(tx, 2, false, 10, 2);	 // ensure there's something to receive
		bus->receive(100, false);			 // receive remaining bytes and do not check crc cause it will be garbage
	}

	if (bus->transfer(tx, 2, true, 10, 2))
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
	if (bus->transfer(tx, 2, true, 100, 2))
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
	bus->transfer(tx, length, false, 100, sizeof(StdMd80ResponseFrame_t) * md80s.size() + 1);
}

bool Candle::setupMd80Calibration(uint16_t canId, uint16_t torqueBandwidth)
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

	GenericMd80Frame32 frame = _packMd80Frame(canId, 4, Md80FrameId_E::FRAME_CALIBRATION);
	char tx[64];
	frame.canMsg[2] = (uint8_t)(torqueBandwidth & 0xff);
	frame.canMsg[3] = (uint8_t)(torqueBandwidth >> 8);
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transfer(tx, len, true, 50, 66))
		if (*bus->getRxBuffer(1) == true)
		{
			vout << "Starting calibration at ID = " << canId << statusOK << std::endl;
			return true;
		}
	vout << "Starting calibration failed at ID = " << canId << statusFAIL << std::endl;
	return false;
}
bool Candle::setupMd80Diagnostic(uint16_t canId)
{
	GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_DIAGNOSTIC);
	char tx[64];
	int len = sizeof(frame);
	memcpy(tx, &frame, len);
	if (bus->transfer(tx, len, true, 50, 66))
	{
		vout << "Library version: " << getVersion() << std::endl;
		vout << "DIAG at ID = " << canId << ": " << std::string(&*bus->getRxBuffer(2)) << std::endl;
		return true;
	}
	vout << "Diagnostic failed at ID = " << canId << std::endl;
	return false;
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
	if (bus->transfer(tx, len, true, 10, 66, false))
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
