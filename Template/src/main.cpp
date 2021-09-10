#include "candle.hpp"
#include "md80.hpp"
#include "unistd.h"
int main (int argc, char ** argv)
{
	mab::Candle can("/dev/ttyUSB0", 460800, 1000000);
	mab::Md80::initalize(&can);

	mab::Md80::deinitalize();
	return 0;
}
