#include "candle.hpp"

int main()
{
	mab::Candle candle(mab::CAN_BAUD_1M, true);
	auto ids = candle.ping();

	/* add the first MD80 to the list */
	candle.addMd80(ids[0]);

	/* get the reference to the regR struct that holds fields of registers */
	mab::regRead_st& regR = candle.getMd80FromList(ids[0]).getReadReg();

	/* read registers of your choice NOTE: remember that a single read function call cannot exceed 62 bytes (runtime exception will be thrown otherwise),
	 including the 2 byte reg ID that is attached per register.
	 In this example the length is: 2 (resistance reg ID) + 4 (float value) + 2 (inductance reg ID) + 4 (float value)  = 12 bytes */
	if (!candle.readMd80Register(ids[0],
								 mab::Md80Reg_E::motorResistance, regR.RO.resistance,
								 mab::Md80Reg_E::motorInductance, regR.RO.inductance))
	{
		std::cout << "Reading register failed at ID: " << ids[0] << std::endl;
		return false;
	}

	/* NOTE: please make sure the motor is calibrated or otherwise you will read zeros in these registers */

	std::cout << "Motor d-axis resistance: " << regR.RO.resistance << " Ohms " << std::endl;
	std::cout << "Motor d-axis inductance: " << regR.RO.inductance << " H " << std::endl;

	return EXIT_SUCCESS;
}