#include "candle.hpp"
#include "md80.hpp"

int main (int argc, char ** argv)
{
	mab::Candle can("/dev/ttyUSB0", 460800, 1000000);
	mab::md80::Md80::sendPing(&can, 0x70, 0x90);
	return 0;
}