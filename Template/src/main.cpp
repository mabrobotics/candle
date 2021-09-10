#include "candle.hpp"
#include "md80.hpp"
#include "unistd.h"
int main (int argc, char ** argv)
{
	mab::Candle can("/dev/ttyUSB0", 460800, 1000000);
	mab::Md80::initalize(&can);
	mab::Md80 x0(0);
	mab::Md80 x1(1);
	mab::Md80 x2(2);
	mab::Md80 x3(3);
	mab::Md80 x4(4);
	mab::Md80 x5(5);
	mab::Md80 x6(6);
	sleep(1);
	x0.setCurrentLimit(4);
	x1.enable();
	x2.restart();
	x0.restart();
	sleep(1);
	mab::Md80::deinitalize();
	return 0;
}
