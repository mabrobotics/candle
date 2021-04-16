#include "candle.hpp"
#include "md80.hpp"

int main (int argc, char ** argv)
{
	mab::Candle can("/dev/ttyUSB0", 460800, 1000000);
	//mab::md80::Md80::sendPing(&can, 0x90, 0x9A);
	mab::md80::Md80::initalize(&can);
	mab::md80::Md80 x(0x10);
	x.setMode(mab::md80::Mode::IMPEDANCE_CTL);
	x.setImpedance(5.5, 0.05, 0.0, 0.0, 0.0f, 0.5f);
	if(x.enableMotor(true))
		x.printInfo();

	return 0;
}