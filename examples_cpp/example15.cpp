#include <unistd.h>

#include <iostream>

#include "candle.hpp"

bool getExampleConfirmation();

int main()
{
	// Create CANdle object and set FDCAN baudrate to 1Mbps
	mab::Candle candle(mab::CAN_BAUD_1M, true);

	// Ping FDCAN bus in search of drives
	auto ids = candle.ping();

	if (ids.size() == 0 || !getExampleConfirmation())
		return EXIT_FAILURE;

	candle.addMd80(ids[0]);
	candle.controlMd80Mode(ids[0], mab::Md80Mode_E::RAW_TORQUE);  // Set mode to torque
	candle.controlMd80Enable(ids[0], true);						  // Enable the drive

	// Begin update loop (it starts in the background)
	candle.begin();
	candle.md80s[0].setTargetTorque(0.1f);
	sleep(5);
	candle.end();

	return EXIT_SUCCESS;
}

bool getExampleConfirmation()
{
	std::cout << "This example will spin the motor very fast if the output shaft is unloaded." << std::endl;
	std::cout << "Are you sure you want to proceed? [Y/n]" << std::endl;
	char x;
	std::cin >> x;
	if (x != 'Y')
	{
		std::cout << "Confirmation failed." << std::endl;
		return false;
	}
	return true;
}