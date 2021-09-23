#include "candle.hpp"
#include "md80.hpp"
#include "unistd.h"
#include <iostream>
#include <cmath>

int main (int argc, char ** argv)
{
	mab::Candle can("/dev/ttyACM1", 4000000);
	mab::Md80::initalize(&can);
	auto ids = mab::Md80::sendPing(&can, 29, 80);
	return 1;
	if(ids.size() == 1)
	{
		mab::Md80 xd(69);
		xd.setZeroPosition();
		mab::Md80 x(ids[0]);
		x.configSetNewCanConfig(ids[0], 4000000);
		sleep(1);
		can.setCanSpeed(4000000);
		if (mab::Md80::sendPing(&can, 29, 80).size() == 1)
		{
			if (x.configSaveNewConfig())
				std::cout << "SUCCESS!" << std::endl;
		}
	}

	mab::Md80::deinitalize();
	return 0;
}
