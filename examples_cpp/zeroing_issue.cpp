#include "candle.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

using namespace mab;

constexpr Md80Reg_E LUT_INDEX_ADDR = Md80Reg_E::reserved1;
constexpr Md80Reg_E LUT_VAL_ADDR = Md80Reg_E::reserved2;

inline bool writeLutInstance(Candle *candle, size_t index, float value) {
  constexpr uint16_t canId = 100;
  constexpr float tolerance = 2.0f * M_PI / 32768.f;
  size_t indexBuffer = 0;
  float valueBuffer = 0.f;
  candle->writeMd80Register(canId, LUT_INDEX_ADDR, index);
  candle->readMd80Register(canId, LUT_INDEX_ADDR, indexBuffer);
  if (indexBuffer != index) {
    std::cerr << "Error writing LUT index register! Wrote: " << index
              << " Read: " << indexBuffer << std::endl;
    return false;
  }
  candle->writeMd80Register(canId, LUT_VAL_ADDR, value);
  candle->readMd80Register(canId, LUT_VAL_ADDR, valueBuffer);
  if (std::abs(valueBuffer - value) > tolerance) {
    std::cerr << "Error writing LUT value register! Wrote: " << value
              << " Read: " << valueBuffer << std::endl;
    return false;
  }
  return true;
}

inline bool clearLut(Candle *candle) {
  constexpr size_t lutSize = 512;
  for (size_t i = 0; i < lutSize; i++) {
    if (!writeLutInstance(candle, i, 0.0f)) {
      return false;
    }
  }
  return true;
}

inline float getPositionBasedOnIndex(size_t index) {
    constexpr size_t lutSize = 512;
    if(index >= lutSize) {
        index = index % lutSize;
    }
    return (float)index * (2.0f * M_PI / (float)lutSize);
}

int main() {

  constexpr float sinAmplitude = 1.0f;
  Candle candle(CANdleBaudrate_E::CAN_BAUD_1M, true);

  std::array<float, 512> lutRepresentation = {0};

  for (size_t i = 0; i < lutRepresentation.size(); i++) {
    writeLutInstance(&candle, i, 0.0f);

    lutRepresentation[i] = 0.0f;
  }

//   candle.writeMd80Register(100, Md80Reg_E::runSaveCmd, 1);
//   std::this_thread::sleep_for(std::chrono::seconds(5));

  // Scan for the beginning of the lut
  float initialAuxEncoderPosition = 0.0f;
  candle.readMd80Register(100, Md80Reg_E::outputEncoderPosition,
                          initialAuxEncoderPosition);
  std::cout << "Initial aux encoder position: " << initialAuxEncoderPosition
            << std::endl;

  size_t lutStartIndex = 0;

  for (size_t i = 0; i < lutRepresentation.size(); i++) {
    writeLutInstance(&candle, i, 1.0f);
    lutRepresentation[i] = 1.0f;

    float newAuxEncoderPosition = 0.0f;
    candle.readMd80Register(100, Md80Reg_E::outputEncoderPosition,
                            newAuxEncoderPosition);
    std::cout << "Wrote LUT index: " << i
              << ", aux encoder position: " << newAuxEncoderPosition
              << std::endl;
    std::cout << "The current position suggest the Lut start is at index: "
              << initialAuxEncoderPosition * 512 / (M_PI * 2) << std::endl;
    if (newAuxEncoderPosition - 0.2f > initialAuxEncoderPosition) {
      std::cout << "Found LUT start at index: " << i << std::endl;
      lutStartIndex = i;
      break;
    }
  }
  // Clear the LUT
  clearLut(&candle);

  // Fill the lut representation with a sine wave from the start index
  for (size_t i = 0; i < lutRepresentation.size(); i++) {
    size_t lutIndex = (i + lutStartIndex) % lutRepresentation.size();
    lutRepresentation[lutIndex] =
        std::sin(2 * M_PI * i / lutRepresentation.size()) * sinAmplitude;
    writeLutInstance(&candle, lutIndex, lutRepresentation[lutIndex]);
  }
  candle.writeMd80Register(100, Md80Reg_E::runZero, 1);
  candle.writeMd80Register(100, Md80Reg_E::targetPosition, 0.0f);
  candle.writeMd80Register(100, Md80Reg_E::motionModeCommand, static_cast<uint8_t>(Md80Mode_E::IMPEDANCE));
  candle.controlMd80Enable(100, true);

  Md80Mode_E currentMode = Md80Mode_E::IDLE;
    candle.readMd80Register(100, Md80Reg_E::motionModeStatus, currentMode);
    if(currentMode != Md80Mode_E::IMPEDANCE) {
        std::cerr << "Error: Drive not in impedance mode! Mode is: " << (int)currentMode << std::endl;
        return -1;
    }

  //Move through the LUT
  for(size_t i = 0; i < lutRepresentation.size(); i++) {
    float targetPosition = i * (2.0f * M_PI / (float)lutRepresentation.size());
    candle.writeMd80Register(100, Md80Reg_E::targetPosition, targetPosition);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    float auxEncoderPosition = 0.0f;
    float subtractSine = lutRepresentation[(i + lutStartIndex) % lutRepresentation.size()];
    candle.readMd80Register(100, Md80Reg_E::outputEncoderPosition, auxEncoderPosition);
    float error = auxEncoderPosition - (targetPosition - initialAuxEncoderPosition);
    std::cout << "Target position: " << targetPosition << ", aux encoder position: " << auxEncoderPosition 
              << ", LUT value: " << subtractSine << ", Error: " << error << std::endl;
  }
  candle.controlMd80Enable(100, false);


  return 0;
}