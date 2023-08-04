#include <unistd.h>

#include <iostream>

#include "candle.hpp"

int main()
{
	// Create CANdle object and set FDCAN baudrate to 1Mbps
	mab::Candle candle(mab::CAN_BAUD_1M, true);

	// Ping FDCAN bus in search of drives
	auto ids = candle.ping();

	if (ids.size() == 0)  // If no drives found -> quit
		return EXIT_FAILURE;

	// Add all found to the update list
	for (auto& id : ids)
		candle.addMd80(id);

	candle.writeMd80Register(ids[0], mab::Md80Reg_E::positionWindow, 0.05f);
	candle.writeMd80Register(ids[0], mab::Md80Reg_E::velocityWindow, 1.0f);
	candle.writeMd80Register(ids[0], mab::Md80Reg_E::profileAcceleration, 5.0f);
	candle.writeMd80Register(ids[0], mab::Md80Reg_E::profileDeceleration, 10.0f);
	candle.writeMd80Register(ids[0], mab::Md80Reg_E::profileVelocity, 10.0f);
	candle.writeMd80Register(ids[0], mab::Md80Reg_E::quickStopDeceleration, 200.0f);

	candle.controlMd80SetEncoderZero(ids[0]);							// Reset encoder at current position
	candle.controlMd80Mode(ids[0], mab::Md80Mode_E::VELOCITY_PROFILE);	// Set mode to position PID
	candle.controlMd80Enable(ids[0], true);								// Enable the drive

	// Begin update loop (it starts in the background)
	candle.begin();

	candle.md80s[0].setTargetVelocity(10.0f);

	while (!candle.md80s[0].isTargetVelocityReached())
	{
		sleep(1);
	};

	candle.md80s[0].setProfileAcceleration(20.0f);
	candle.md80s[0].setProfileVelocity(50.0f);
	candle.md80s[0].setTargetVelocity(20.0f);

	while (!candle.md80s[0].isTargetVelocityReached())
	{
		sleep(1);
	};

	// Close the update loop
	candle.end();

	return EXIT_SUCCESS;
}