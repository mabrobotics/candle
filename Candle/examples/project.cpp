#include "candle.hpp"

int main()
{
    Candle candle(mab::CAN_BAUD_1M);
    candle.ping();
    return 1;
}