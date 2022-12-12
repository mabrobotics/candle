#include <gtest/gtest.h>

#include "bus.hpp"
#include "candle.hpp"
#include "candle_protocol.hpp"
#include "gmock/gmock.h"

using namespace testing;

class MockSpiDevice : public mab::Bus
{
   public:
	MockSpiDevice()
	{
		busType = mab::BusType_E::SPI;
	}
	MOCK_METHOD(bool, transmit, (char* buffer, int len, bool waitForResponse, int timeout, int responseLen, bool faultVerbose));
	MOCK_METHOD(bool, transfer, (char* buffer, int commandLen, int responseLen), (override));
	MOCK_METHOD(unsigned long, getId, (), (override));
	MOCK_METHOD(std::string, getDeviceName, (), (override));

	bool setRxBuf()
	{
		const char rx1[3] = {mab::BusFrameId_t::BUS_FRAME_CANDLE_CONFIG_BAUDRATE, 1, 0x14};
		memcpy(rxBuffer, rx1, 3);
		return true;
	}
};

TEST(CANdle_test, Test_Dummy)
{
	ASSERT_EQ(1, true);
}

TEST(TestCandle, test_contructor)
{
	MockSpiDevice* spiDeviceM(new MockSpiDevice());

	static const char ex1[2] = {mab::BusFrameId_t::BUS_FRAME_RESET, 0x00};

	EXPECT_CALL(*spiDeviceM, transmit(_, 2, true, 100, 2, _))
		.With(Args<0, 1>(ElementsAreArray(ex1)))
		.WillOnce(Return(true));

	static const char ex2[2] = {mab::BusFrameId_t::BUS_FRAME_CANDLE_CONFIG_BAUDRATE, mab::CAN_BAUD_1M};

	const char rx1[3] = {mab::BusFrameId_t::BUS_FRAME_CANDLE_CONFIG_BAUDRATE, 1, 0x14};

	EXPECT_CALL(*spiDeviceM, transmit(_, 2, true, 50, 3, _))
		.With(Args<0, 1>(ElementsAreArray(ex2)))
		// .WillOnce(Return(true))
		.WillOnce(InvokeWithoutArgs(spiDeviceM, &MockSpiDevice::setRxBuf));

	mab::Candle candle(mab::CAN_BAUD_1M, true, spiDeviceM);

	delete spiDeviceM;
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}