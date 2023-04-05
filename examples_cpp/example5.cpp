#include <unistd.h>

#include <iostream>

#include "candle.hpp"

float _range(float x, float min, float max);

int main(int argc, char** argv)
{
	float kp = 0.0f;
	float kd = 0.0f;
	if (argc == 3)
	{
		kp = atof(argv[1]);
		kd = atof(argv[2]);
		kp = _range(kp, 0, 5.0f);
		kd = _range(kd, 0, 0.5f);
		std::cout << "Arguments incorrect!" << std::endl;
	}
	else
	{
		std::cout << "Not correct arguments!" << std::endl;
		std::cout << "Arguments are: kp kd" << std::endl;
		std::cout << "kp = <0, 5.0>, kd = <0, 0.5f>" << std::endl;
		std::cout << "For example:" << std::endl;
		std::cout << "./example5 1.0 0.1" << std::endl;
		return EXIT_FAILURE;
	}

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

	// Now we modify the Impedance controller parameters - the drive will behave much different than in
	// previous examples. The drive will change default params to the ones we select below.
	candle.md80s[0].setImpedanceControllerParams(kp, kd);

	// To reload default controller parameters, simply disable the drive (contorlMd80Enable(id, false)),
	// stop the communications (candle.end()) or power cycle the drive (off-on).

	float t = 0.0f;
	float dt = 0.04f;

	// Begin update loop (it starts in the background)
	candle.begin();

	for (int i = 0; i < 1000; i++)
	{
		t += dt;
		candle.md80s[0].setTargetPosition(sin(t));
		usleep(10000);
	}

	// Close the update loop
	candle.end();

	return EXIT_SUCCESS;
}

float _range(float x, float min, float max)
{
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}