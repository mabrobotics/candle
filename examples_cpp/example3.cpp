#include <iostream>
#include <random>

#include "candle.hpp"

int main()
{
	// Create CANdle object and set FDCAN baudrate to 1Mbps
	mab::Candle candle(mab::CAN_BAUD_1M, true);

	// Ping FDCAN bus in search of drives
	auto ids = candle.ping();

	if (ids.size() == 0)
		return EXIT_FAILURE;

	srand(time(NULL));
	uint16_t newFDCanId = rand() % 990 + 10;  // Generate random id in range 10 - 1000

	// any commands starting with config* can be accessed without adding Md80 to List (addMd80 method)
	candle.configMd80SetCurrentLimit(ids[0], 2.5f);					  // Set motor current limit
	candle.configMd80Can(ids[0], newFDCanId, mab::CAN_BAUD_1M, 250);  // Set custom FDCAN parameters of  the drive

	// Save current limit setting and CAN configuration. Note this is commented out by default not to mess your drives
	// candle.configMd80Save(newFDCanId);

	return EXIT_SUCCESS;
}