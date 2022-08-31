#include "candle.hpp"

int main()
{
	// Create CANdle object and ping FDCAN bus in search of drives.
	// Any found drives will be printed out by the ping() method.

	mab::Candle candle(mab::CAN_BAUD_1M, true);
	auto ids = candle.ping();

	// Blink LEDs on each drive found
	for (auto& id : ids)
		candle.configMd80Blink(id);

	return EXIT_SUCCESS;
}