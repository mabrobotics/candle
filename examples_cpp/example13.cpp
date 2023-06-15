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

	candle.writeMd80Register(ids[0], mab::Md80Reg_E::maxAcceleration, 250.0f);
	candle.writeMd80Register(ids[0], mab::Md80Reg_E::maxDeceleration, 250.0f);
	candle.writeMd80Register(ids[0], mab::Md80Reg_E::maxVelocity, 110.0f);
	// candle.writeMd80Register(ids[0], mab::Md80Reg_E::profileAcceleration, 250.0f);
	// candle.writeMd80Register(ids[0], mab::Md80Reg_E::profileDeceleration, 250.0f);
	// candle.writeMd80Register(ids[0], mab::Md80Reg_E::profileVelocity, 10.0f);

	candle.controlMd80SetEncoderZero(ids[0]);							// Reset encoder at current position
	candle.controlMd80Mode(ids[0], mab::Md80Mode_E::POSITION_PROFILE);	// Set mode to position PID
	candle.controlMd80Enable(ids[0], true);								// Enable the drive

	float t = 0.0f;
	float dt = 0.02f;

	candle.md80s[0].setProfileAcceleration(50.0f);
	candle.md80s[0].setProfileVelocity(100.0f);

	// Begin update loop (it starts in the background)
	candle.begin();

	candle.md80s[0].setTargetPosition(300.0f);
	sleep(10);
	candle.md80s[0].setTargetPosition(-300.0f);
	sleep(10);

	// Close the update loop
	candle.end();
	candle.controlMd80Enable(ids[0], false);

	return EXIT_SUCCESS;
}