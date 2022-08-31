#include <unistd.h>

#include "candle.hpp"
int main()
{
	// Create CANdle object and ping FDCAN bus in search of drives.
	// Any found drives will be printed out by the ping() method.

	mab::Candle candle1(mab::CAN_BAUD_1M, true, mab::CANdleFastMode_E::FAST1);
	mab::Candle candle2(mab::CAN_BAUD_1M, true, mab::CANdleFastMode_E::FAST1);

	std::cout << "Candle 1 ID is: " << candle1.getUsbDeviceId() << std::endl;
	std::cout << "Candle 2 ID is: " << candle2.getUsbDeviceId() << std::endl;

	for (auto& id : candle1.ping(mab::CAN_BAUD_1M))
		candle1.addMd80(id);
	for (auto& id : candle2.ping(mab::CAN_BAUD_1M))
		candle2.addMd80(id);

	return EXIT_SUCCESS;
}