#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <cstdint>
#include "candle.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pyCandle, m) {

  m.doc() = "pyCandle module for interfacing with md80 drives using Python";
  
  py::enum_<mab::CANdleBaudrate_E>(m, "CANdleBaudrate_E")
      .value("CAN_BAUD_1M", mab::CAN_BAUD_1M)
      .value("CAN_BAUD_2M", mab::CAN_BAUD_2M)
      .value("CAN_BAUD_5M", mab::CAN_BAUD_5M)
      .value("CAN_BAUD_8M", mab::CAN_BAUD_8M)
      .export_values();

  py::enum_<mab::Md80Mode_E>(m, "Md80Mode_E")
    .value("IDLE", mab::IDLE)
    .value("POSITION_PID", mab::POSITION_PID)
    .value("ELOCITY_PID", mab::VELOCITY_PID)
    .value("TORQUE", mab::TORQUE)
    .value("IMPEDANCE", mab::IMPEDANCE)
    .export_values();

  py::enum_<mab::CANdleMode_E>(m, "CANdleMode_E")
    .value("CONFIG", mab::CONFIG)
    .value("UPDATE", mab::UPDATE)
    .export_values();
    
  py::class_<mab::Md80>(m, "Md80")
    .def("setPositionControllerParams",&mab::Md80::setPositionControllerParams)
    .def("setVelocityControllerParams",&mab::Md80::setVelocityControllerParams)
    .def("setImpedanceControllerParams",&mab::Md80::setImpedanceControllerParams)
    .def("setMaxTorque",&mab::Md80::setMaxTorque)
    .def("setMaxVelocity",&mab::Md80::setMaxVelocity)
    .def("setTargetPosition",&mab::Md80::setTargetPosition)
    .def("setTargetVelocity",&mab::Md80::setTargetVelocity)
    .def("setTorque",&mab::Md80::setTorque)
    .def("getErrorVector",&mab::Md80::getErrorVector)
    .def("getId",&mab::Md80::getId)
    .def("getPosition",&mab::Md80::getPosition)
    .def("getVelocity",&mab::Md80::getVelocity)
    .def("getTorque",&mab::Md80::getTorque)
  ;
  

  py::class_<mab::Candle>(m, "Candle")
    .def(py::init<mab::CANdleBaudrate_E, bool>())
    .def_readwrite("md80s", &mab::Candle::md80s)
    .def("setVebose",&mab::Candle::setVebose)
    .def("getVersion", &mab::Candle::getVersion)
    .def("ping",&mab::Candle::ping)
    .def("sengGenericFDCanFrame", &mab::Candle::sengGenericFDCanFrame)
    .def("addMd80", &mab::Candle::addMd80)
    .def("configCandleBaudrate", &mab::Candle::configCandleBaudrate)
    .def("configMd80Can", &mab::Candle::configMd80Can)
    .def("configMd80SetCurrentLimit", &mab::Candle::configMd80SetCurrentLimit)
    .def("configMd80Save", &mab::Candle::configMd80Save)
    .def("configMd80Blink", &mab::Candle::configMd80Blink)
    .def("controlMd80SetEncoderZero", py::overload_cast<mab::Md80&>(&mab::Candle::controlMd80SetEncoderZero))
    .def("controlMd80SetEncoderZero", py::overload_cast<uint16_t>(&mab::Candle::controlMd80SetEncoderZero))
    .def("controlMd80Mode", py::overload_cast<mab::Md80&, mab::Md80Mode_E>(&mab::Candle::controlMd80Mode))
    .def("controlMd80Mode", py::overload_cast<uint16_t, mab::Md80Mode_E>(&mab::Candle::controlMd80Mode))
    .def("controlMd80Enable", py::overload_cast<mab::Md80&, bool>(&mab::Candle::controlMd80Enable))
    .def("controlMd80Enable", py::overload_cast<uint16_t, bool>(&mab::Candle::controlMd80Enable))
    .def("getMd80FromList", &mab::Candle::getMd80FromList)
    .def("begin", &mab::Candle::begin)
    .def("end", &mab::Candle::end)
    ;
}
