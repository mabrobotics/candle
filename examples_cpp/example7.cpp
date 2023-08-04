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

	candle.controlMd80Mode(ids[0], mab::Md80Mode_E::VELOCITY_PID);	// Set mode to velocity PID
	candle.controlMd80Enable(ids[0], true);							// Enable the drive

	// Uncomment the line below to change the *.cfg file defaults
	// candle.md80s[0].setVelocityControllerParams(0.1f, 0.25f, 0.0, 1.0f);
	// To reload default controller parameters, simply disable the drive (contorlMd80Enable(id, false)),
	// stop the communications (candle.end()) or power cycle the drive (off-on).

	float t = 0.0f;
	float dt = 0.04f;

	// Begin update loop (it starts in the background)
	candle.begin();

	float targetVelocity = 1.0f;
	for (int i = 0; i < 2000; i++)
	{
		if (i % 200 == 0)
			targetVelocity += 1.0f;
		t += dt;
		candle.md80s[0].setTargetVelocity(targetVelocity);
		std::cout << "Drive ID = " << candle.md80s[0].getId() << " Velocity: " << candle.md80s[0].getVelocity() << std::endl;
		usleep(10000);
	}

	// Close the update loop
	candle.end();

	return EXIT_SUCCESS;
}