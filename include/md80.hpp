#pragma once

#include <cmath>
#include <cstdint>
#include <functional>

#include "mab_types.hpp"
#include "register.hpp"
namespace mab
{
/**
 * @class Md80
 * @brief Contains settings and state of a single md80 drive.
 * Can be used to change control parameters and read the state of the drive.
 */
class Md80
{
   public:
	struct State
	{
		float position;
		float velocity;
		float torque;
		float outputEncoderPosition;
		float outputEncoderVelocity;
		uint8_t temperature;
		uint16_t errorVector;
	};

	struct Targets
	{
		float positionTarget;
		float velocityTarget;
		float torqueTarget;
		float maxTorque;
		float profileVelocity;
		float profileAcceleration;
	};

   private:
	uint16_t canId;
	Md80Mode_E controlMode = Md80Mode_E::IDLE;

	State state;
	Targets targets;

	RegPid_t velocityController = {};
	RegPid_t positionController = {};
	RegImpedance_t impedanceController = {};

	bool maxTorqueAdjusted = false;
	bool profileVelocityAdjusted = false;
	bool profileAccelerationAdjusted = false;
	bool regulatorsAdjusted = false;
	bool velocityRegulatorAdjusted = false;

	bool targetVelocityReached = false;
	bool targetPositionReached = false;

	StdMd80CommandFrame_t commandFrame;
	StdMd80ResponseFrame_t responseFrame;

	regRead_st regRead;
	regWrite_st regWrite;

	void packImpedanceFrame();
	void packPositionFrame();
	void packVelocityFrame();
	void packMotionTargetsFrame();

	void emptyCallback(){};
	std::function<void()> txCallback = std::bind(&Md80::emptyCallback, this);
	std::function<void()> rxCallback = std::bind(&Md80::emptyCallback, this);

   public:
	/**
	 * @brief Construct a new Md80 object
	 *
	 * @param canID FDACN Id of the drive
	 */
	Md80(uint16_t canID);
	/**
	 * @brief Set the Position PID Regulator parameters.
	 * @note Regulator output is target velocity in rad/s. The output is then passed as input to Velocity PID regulator.
	 * @param kp proportional gain
	 * @param ki integral gain
	 * @param kd derivative gain
	 * @param iWindup anti-windup - maximal output of the integral (i) part of the regulator
	 */
	void setPositionControllerParams(float kp, float ki, float kd, float iWindup);
	/**
	 * @brief Set the Velocity PID Regulator parameters.
	 * @note Regulator output is Torque in Nm. The output is then passed directly to internal current/torque regulator.
	 * @param kp proportional gain
	 * @param ki integral gain
	 * @param kd derivative gain
	 * @param iWindup anti-windup - maximal output of the integral (i) part of the regulator
	 */
	void setVelocityControllerParams(float kp, float ki, float kd, float iWindup);
	/**
	 * @brief Set the Impedance Regulator parameters.
	 * @param kp Displacement gain ( analogic to 'k' parameter of the spring-damper equation)
	 * @param kd Damping coefficient (analogin to 'b' parameter of the spring-damper equation)
	 */
	void setImpedanceControllerParams(float kp, float kd);

	// simple setters
	/**
	 * @brief Set the Max Torque object
	 * @note This can be overriden by current limiter set by 'Candle.configMd80CurrentLimit'. Current/torque
	 * will be limited to whichever limit has a lower value.
	 * @note This is only applied with CUSTOM Impedance/Velocity PID controller settings.
	 * @param maxTorque Torque limit for PID/Impedance regulators
	 */
	void setMaxTorque(float maxTorque);
	/**
	 * @brief Set the Max Velocity for Position PID and Velocity PID modes.
	 * @note This is only applied with CUSTOM Position PID and Velocity PID controller settings.
	 * @note Has no effect in Torque or Impedance mode.
	 * @param profileVelocity
	 */
	void setProfileVelocity(float profileVelocity);
	/**
	 * @brief Set the Max Acceleration and Deceleration for Position PID and Velocity PID modes.
	 * @note This is only applied with CUSTOM Position PID and Velocity PID controller settings.
	 * @note Has no effect in Torque or Impedance mode.
	 * @param _maxAcceleration Maximal acceleration in rad/s^2 (radians per second squared) if set to 0 acceleration is unlimited.
	 */
	void setProfileAcceleration(float newMaxAcceleration);
	/**
	 * @brief Set the Target Position for Position PID and Impedance modes.
	 * @param target target position in radians
	 */
	void setTargetPosition(float target)
	{
		targets.positionTarget = target;
		targetPositionReached = false;
	};
	/**
	 * @brief Set the Target Velocity for Velocity PID and Impedance modes.
	 * @param target target velocity in rad/s (radians per second)
	 */
	void setTargetVelocity(float target)
	{
		targets.velocityTarget = target;
		targetVelocityReached = false;
	};
	/**
	 * @brief Set the Torque target for RAW_TORQUE and IMPEDANCE modes.
	 * @param target target torque in Nm (Newton-meters)
	 */
	void setTorqueTarget(float target) { targets.torqueTarget = target; };

	// getters
	/**
	 * @brief Get the Error Vector of the md80
	 * @return uint16_t vector with per-bit coded errors. Refer to documentation for meaning of error codes.
	 */
	uint16_t getErrorVector() { return errorVector; };

	/**
	 * @brief Get the FDCAN Id of the drive
	 * @return uint16_t FDCAN Id (10 - 2047)
	 */
	uint16_t getId() { return canId; };
	/**
	 * @brief Get the Position of md80
	 * @return float angular position in radians
	 */
	float getPosition() { return state.position; };
	/**
	 * @brief Get the Velocity of md80
	 * @return float angular velocity in rad/s (radians per second)
	 */
	float getVelocity() { return state.velocity; };
	/**
	 * @brief Get the Torque of md80
	 * @return float torque in Nm (Newton-meters)
	 */
	float getTorque() { return state.torque; };

	/**
	 * @brief Get the Position of md80
	 * @return float angular position in radians
	 */
	float getOutputEncoderPosition() { return outputEncoderPosition; };
	/**
	 * @brief Get the Velocity of md80
	 * @return float angular velocity in rad/s (radians per second)
	 */
	float getOutputEncoderVelocity() { return outputEncoderVelocity; };
	/**
	 * @brief Get the exteral thermistor temperature reading (motor thermistor)
	 * @return uint8_t temperature value in *C
	 */
	uint8_t getTemperature() { return state.temperature; };

	/**
	 * @brief Get the read register struct
	 * @return reference to read register struct
	 */
	regRead_st& getReadReg() { return regRead; };

	/**
	 * @brief Get the write register struct
	 * @return reference to write register struct
	 */
	regWrite_st& getWriteReg() { return regWrite; };

	/**
	 * @brief Register a user callback that will be called on each bus receive frame
	 * @param T class instance pointer
	 * @param func member function pointer
	 */
	template <typename T>
	void registerRXCallback(T* p, void (T::*func)())
	{
		rxCallback = std::bind(func, p);
	}
	/**
	 * @brief Register a user callback that will be called on each bus transmit frame
	 * @param T class instance pointer
	 * @param func member function pointer
	 */
	template <typename T>
	void registerTXCallback(T* p, void (T::*func)())
	{
		txCallback = std::bind(func, p);
	}
	/**
	 * @brief check if actual target position is reached within positionWindow
	 * @return true if reached, false otherwise
	 */
	bool isTargetPositionReached() const
	{
		return targetPositionReached;
	}
	/**
	 * @brief check if actual target velocity is reached within velocityWindow
	 * @return true if reached, false otherwise
	 */
	bool isTargetVelocityReached() const
	{
		return targetVelocityReached;
	}

	/**
	 * @brief For internal use by CANdle only.
	 * @private
	 */
	StdMd80CommandFrame_t __getCommandFrame()
	{
		return commandFrame;
	};
	/**
	 * @brief For internal use by CANdle only. Updates FDCAN frame based on parameters.
	 * @private
	 */
	void __updateCommandFrame();
	/**
	 * @brief For internal use by CANdle only. Updates FDCAN frame parameters.
	 * @private
	 */
	void __updateResponseData(const State& state);
	/**
	 * @brief For internal use by CANdle only. Updates FDCAN frame parameters.
	 * @private
	 */
	void __updateResponseData(StdMd80ResponseFrame_t* _responseFrame);
	/**
	 * @brief For internal use by CANdle only.
	 * @private
	 */
	void __setControlMode(Md80Mode_E mode);
};
}  // namespace mab