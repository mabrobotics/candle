#pragma once
#include "mab_types.hpp"

class Crc
{
   public:
	uint32_t addCrcToBuf(char* buffer, uint32_t dataLength);
	bool checkCrcBuf(char* buffer, uint32_t dataLength);
	uint32_t getCrcLen() { return crcLen; };

   private:
	static const uint32_t crcLen = 4;

	typedef union
	{
		char u8[4];
		uint32_t u32;
	} CRC_ut;

	uint32_t calcCrc(char* pData, uint32_t dataLength);
};