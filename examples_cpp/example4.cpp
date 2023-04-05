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

	candle.controlMd80Mode(ids[0], mab::Md80Mode_E::IMPEDANCE);	 // Set mode to impedance control
	candle.controlMd80Enable(ids[0], true);						 // Enable the drive

	float t = 0.0f;
	float dt = 0.04f;

	// Begin update loop (it starts in the background)
	candle.begin();

	for (int i = 0; i < 1000; i++)
	{
		t += dt;
		// After powerup the drive will load set of default parameters for every controller.
		// To make the drive move all we got to do is set mode (.controlMd80Mode) and set target
		// (.setTargetPosition, .setTargetVelocity, .setTargetTorque)
		candle.md80s[0].setTargetPosition(sin(t) * 3.0f);
		usleep(10000);	// Add some delay
	}

	// Close the update loop
	candle.end();
	return EXIT_SUCCESS;
}