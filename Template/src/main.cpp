#include "md80.hpp"

int main (int argc, char ** argv)
{
	mab::Candle can("/dev/ttyACM0", 8000000);
	mab::Md80::initalize(&can);

	mab::Md80::sendPing(&can, 0 , 100);
	
	mab::Md80::deinitalize();
}