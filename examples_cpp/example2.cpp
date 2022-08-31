#include <unistd.h>

#include <iostream>

#include "candle.hpp"

int main()
{
	// Create CANdle object and set FDCAN baudrate to 1Mbps
	mab::Candle candle(mab::CAN_BAUD_1M, true);

	// Ping FDCAN bus in search of drives
	auto ids = candle.ping();

	// Add all found to the update list
	for (auto& id : ids)
		candle.addMd80(id);

	// Begin update loop (it starts in the background)
	candle.begin();

	// Auto update loop is running in the background updating data in candle.md80s vector. Each md80 object can be
	// called for data at any time
	for (int i = 0; i < 1000; i++)
	{
		std::cout << "Drive Id: " << candle.md80s[0].getId() << " Position: " << candle.md80s[0].getPosition() << std::endl;
		usleep(100000);
	}

	// Close the update loop
	candle.end();

	return EXIT_SUCCESS;
}