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

	candle.controlMd80SetEncoderZero(ids[0]);  // Reset encoder at current position

	candle.controlMd80Mode(ids[0], mab::Md80Mode_E::POSITION_PID);	// Set mode to position PID
	candle.controlMd80Enable(ids[0], true);							// Enable the drive

	// We will run both Position PID and Velocity PID at default settings. If you wish you can play with the parameters
	// Using the methods below:
	// candle.md80s[0].setPositionControllerParams(20.0f, 0.2f, 0.0f, 15.0f);
	// candle.md80s[0].setVelocityControllerParams(0.5f, 0.1f, 0.0f, 1.5f);
	// candle.md80s[0].setProfileVelocity(5.0);
	// candle.md80s[0].setMaxTorque(0.5f);

	// To reload default controller parameters, simply disable the drive (contorlMd80Enable(id, false)),
	// stop the communications (candle.end()) or power cycle the drive (off-on).

	float t = 0.0f;
	float dt = 0.02f;

	// Begin update loop (it starts in the background)
	candle.begin();

	for (int i = 0; i < 1000; i++)
	{
		t += dt;
		candle.md80s[0].setTargetPosition(sin(t) * 2.0f);
		std::cout << "Drive ID = " << candle.md80s[0].getId() << " Velocity: " << candle.md80s[0].getVelocity() << std::endl;
		usleep(10000);
	}

	// Close the update loop
	candle.end();

	return EXIT_SUCCESS;
}