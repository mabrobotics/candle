#include "candle.hpp"
#include "md80.hpp"
#include "unistd.h"
#include <iostream>
#include <cmath>

int main (int argc, char ** argv)
{
	mab::Candle can("/dev/ttyACM1", 1000000);
	mab::Md80::initalize(&can);
	can.setCanSpeed(2500000);
	mab::Md80::sendPing(&can, 0,70);

	mab::Md80::deinitalize();
	return 0;
}
