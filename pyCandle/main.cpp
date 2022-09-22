#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <cstdint>
#include "multipleCandles.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pyCandle, m)
{

  m.doc() = "pyCandle module for interfacing with md80 drives using Python";

  py::enum_<mab::CANdleBaudrate_E>(m, "CANdleBaudrate_E")
      .value("CAN_BAUD_1M", mab::CAN_BAUD_1M)
      .value("CAN_BAUD_2M", mab::CAN_BAUD_2M)
      .value("CAN_BAUD_5M", mab::CAN_BAUD_5M)
      .value("CAN_BAUD_8M", mab::CAN_BAUD_8M)
      .export_values();

  py::enum_<mab::CANdleFastMode_E>(m, "CANdleFastMode_E")
      .value("NORMAL", mab::CANdleFastMode_E::NORMAL)
      .value("FAST1", mab::CANdleFastMode_E::FAST1)
      .value("FAST2", mab::CANdleFastMode_E::FAST2)
      .export_values();

  py::enum_<mab::Md80Mode_E>(m, "Md80Mode_E")
      .value("IDLE", mab::IDLE)
      .value("POSITION_PID", mab::POSITION_PID)
      .value("VELOCITY_PID", mab::VELOCITY_PID)
      .value("TORQUE", mab::TORQUE)
      .value("IMPEDANCE", mab::IMPEDANCE)
      .export_values();

  py::enum_<mab::CANdleMode_E>(m, "CANdleMode_E")
      .value("CONFIG", mab::CONFIG)
      .value("UPDATE", mab::UPDATE)
      .export_values();

  py::class_<mab::Md80>(m, "Md80")
      .def("setPositionControllerParams", &mab::Md80::setPositionControllerParams)
      .def("setVelocityControllerParams", &mab::Md80::setVelocityControllerParams)
      .def("setImpedanceControllerParams", &mab::Md80::setImpedanceRequestedControllerParams)
      .def("setMaxTorque", &mab::Md80::setMaxTorque)
      .def("setMaxVelocity", &mab::Md80::setMaxVelocity)
      .def("setTargetPosition", &mab::Md80::setTargetPosition)
      .def("setTargetVelocity", &mab::Md80::setTargetVelocity)
      .def("setTorque", &mab::Md80::setTorque)
      .def("getErrorVector", &mab::Md80::getErrorVector)
      .def("getId", &mab::Md80::getId)
      .def("getPosition", &mab::Md80::getPosition)
      .def("getVelocity", &mab::Md80::getVelocity)
      .def("getTorque", &mab::Md80::getTorque)
      .def("getMotorStatus", &mab::Md80::getMotorStatus)
      .def("getTemperature", &mab::Md80::getTemperature)
      .def("setSavgolCoeffs", &mab::Md80::setSavgolCoeffs);

  py::class_<mab::Candle>(m, "Candle")
      .def(py::init<mab::CANdleBaudrate_E, bool>())
      .def(py::init<mab::CANdleBaudrate_E, bool, bool>())
      .def(py::init<mab::CANdleBaudrate_E, bool, bool, mab::CANdleFastMode_E>())
      .def_readwrite("md80s", &mab::Candle::md80s)
      .def("setVebose", &mab::Candle::setVebose)
      .def("getVersion", &mab::Candle::getVersion)
      .def("getUsbDeviceId", &mab::Candle::getUsbDeviceId)
      .def("ping", py::overload_cast<>(&mab::Candle::ping))
      .def("ping", py::overload_cast<mab::CANdleBaudrate_E>(&mab::Candle::ping))
      .def("getActualCommunicationFrequency", &mab::Candle::getActualCommunicationFrequency)
      .def("sendGenericFDCanFrame", &mab::Candle::sendGenericFDCanFrame)
      .def("addMd80", &mab::Candle::addMd80)
      .def("configCandleBaudrate", &mab::Candle::configCandleBaudrate)
      .def("configMd80Can", &mab::Candle::configMd80Can)
      .def("configMd80SetCurrentLimit", &mab::Candle::configMd80SetCurrentLimit)
      .def("configMd80Save", &mab::Candle::configMd80Save)
      .def("configMd80Blink", &mab::Candle::configMd80Blink)
      .def("controlMd80SetEncoderZero", py::overload_cast<mab::Md80 &>(&mab::Candle::controlMd80SetEncoderZero))
      .def("controlMd80SetEncoderZero", py::overload_cast<uint16_t>(&mab::Candle::controlMd80SetEncoderZero))
      .def("controlMd80Mode", py::overload_cast<mab::Md80 &, mab::Md80Mode_E>(&mab::Candle::controlMd80Mode))
      .def("controlMd80Mode", py::overload_cast<uint16_t, mab::Md80Mode_E>(&mab::Candle::controlMd80Mode))
      .def("controlMd80Enable", py::overload_cast<mab::Md80 &, bool>(&mab::Candle::controlMd80Enable))
      .def("controlMd80Enable", py::overload_cast<uint16_t, bool>(&mab::Candle::controlMd80Enable))
      .def("begin", &mab::Candle::begin)
      .def("end", &mab::Candle::end)
      .def("reset", &mab::Candle::reset)
      .def("setupMd80Calibration", &mab::Candle::setupMd80Calibration)
      .def("setupMd80Diagnostic", &mab::Candle::setupMd80Diagnostic)
      .def("updateModeBasedOnMd80List", &mab::Candle::updateModeBasedOnMd80List);

  py::class_<mab::MultipleCandles>(m, "MultipleCandles")
      .def(py::init<bool>())
      .def("zeroMd80t", &mab::MultipleCandles::zeroMd80t)
      .def("addMd80", &mab::MultipleCandles::addMd80)
      .def("setModeMd80", &mab::MultipleCandles::setModeMd80)
      .def("enableAllMotors", &mab::MultipleCandles::enableAllMotors)
      .def("enableSomeMotors", &mab::MultipleCandles::enableSomeMotors)
      .def("disableAllMotors", &mab::MultipleCandles::disableAllMotors)
      .def("disableSomeMotors", &mab::MultipleCandles::disableSomeMotors)
      .def("getMotorsData", &mab::MultipleCandles::getMotorsData)
      .def("getAllMotorsData", &mab::MultipleCandles::getAllMotorsData)
      .def("sendMotorCommand", &mab::MultipleCandles::sendMotorCommand)
      .def("setImpedanceParameters", &mab::MultipleCandles::setImpedanceParameters)
      .def("setPositionPIDParameters", &mab::MultipleCandles::setPositionPIDParameters)
      .def("setPIDParams", &mab::MultipleCandles::setPIDParams)
      .def("setSavgol", &mab::MultipleCandles::setSavgol)
      .def("setKalmanFilter", &mab::MultipleCandles::setKalmanFilter);


}
