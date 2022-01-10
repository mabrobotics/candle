#include "md80.hpp"
#include <cmath>
int main (int argc, char ** argv)
{
	mab::Candle can("/dev/ttyACM0", 8000000);
	

	auto ids = mab::Md80::sendPing(&can, 0 , 100);
	if(ids.size() == 0)
		return -1;
	mab::Md80::initalize(&can);
	mab::Md80 x0(ids[0]);
	x0.enable();
	x0.setCurrentLimit(5.0);
	x0.setControlMode(mab::Mode::IMPEDANCE_CTL);
	x0.setZeroPosition();
	float t = 0.0;

	while(1)
	{
		t+=0.001f;
		x0.setImpedanceController(0.0f, 0.01f, 0.0f, sin(t) * 180.0f, 0.0f, 0.2f);
	}
	mab::Md80::deinitalize();
}