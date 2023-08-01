#include "candle.hpp"

bool readAndDisplayRegisters(mab::Candle& candle, uint16_t id);
void error();

int main()
{
	mab::Candle candle(mab::CAN_BAUD_1M, true);
	auto ids = candle.ping();

	/* add the first MD80 to the list */
	candle.addMd80(ids[0]);

	std::cout << std::endl;

	// Lets first read some registers NOTE: registers cannot be accessed after candle.begin();
	if (!readAndDisplayRegisters(candle, ids[0]))
		std::cout << "Error while reading registers!" << std::endl;

	std::cout << std::endl;

	// Then change some parameters
	if (!candle.writeMd80Register(ids[0], mab::Md80Reg_E::motorName, "EXAMPLE12")) error();
	if (!candle.writeMd80Register(ids[0], mab::Md80Reg_E::motorImpPidKp, 1.0f)) error();
	if (!candle.writeMd80Register(ids[0], mab::Md80Reg_E::motorImpPidKd, 0.01f)) error();

	// And read one more time NOTE: these settings are not saved!
	if (!readAndDisplayRegisters(candle, ids[0]))
		std::cout << "Error while reading registers!" << std::endl;

	return EXIT_SUCCESS;
}

bool readAndDisplayRegisters(mab::Candle& candle, uint16_t id)
{
	/* get the reference to the regR struct that holds fields of registers */
	mab::regRead_st& reg = candle.getMd80FromList(id).getReadReg();

	if (!candle.readMd80Register(id, mab::Md80Reg_E::canId, reg.RW.canId)) return false;
	if (!candle.readMd80Register(id, mab::Md80Reg_E::motorName, reg.RW.motorName)) return false;
	if (!candle.readMd80Register(id, mab::Md80Reg_E::motorImpPidKp, reg.RW.impedancePdGains.kp)) return false;
	if (!candle.readMd80Register(id, mab::Md80Reg_E::motorImpPidKd, reg.RW.impedancePdGains.kd)) return false;

	std::cout << "Drive ID: " << unsigned(reg.RW.canId) << std::endl;
	std::cout << "Motor name: " << std::string(reg.RW.motorName) << std::endl;
	std::cout << "Impedance mode Kp gain : " << reg.RW.impedancePdGains.kp << " Nm/rad" << std::endl;
	std::cout << "Impedance mode Kd gain : " << reg.RW.impedancePdGains.kd << " Nm*s/rad" << std::endl;

	return true;
}

void error()
{
	std::cout << "Error while writing register!" << std::endl;
}