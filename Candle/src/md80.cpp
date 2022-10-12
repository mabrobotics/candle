#include "md80.hpp"
#include "mab_types.hpp"
#include <iostream>
#include <assert.h> /* assert */

namespace mab
{
    Md80::Md80(uint16_t _canID, MotorCommand_T config)
    {
        canId = _canID;
        commandFrame.canId = _canID;
        motorStatus["position"] = 0.0;
        motorStatus["velocity"] = 0.0;
        motorStatus["savgol_vel"] = 0.0;
        motorStatus["kalman_vel"] = 0.0;
        motorStatus["torque"] = 0.0;
        motorStatus["time"] = 0.0;
        motorStatus["seq"] = 0.0;
        motorStatus["temperature"] = 0.0;

        // Get watchdog params
        watchdogKP = config["kp"];
        watchdogKD = config["kd"];
        watchdogTorqueOffset = config["torque_offset"];
        softLimitFactor = config["soft_limit"];
        maxMotorPosition = config["max_position"];
        minMotorPosition = config["min_position"];
        softMinPosition = minMotorPosition * softLimitFactor;
        softMaxPosition = maxMotorPosition * softLimitFactor;
        watchdogPosPercentage = config["pos_percent_wanted"];

        std::cout << "[MD80]: Watcdog params for motor " << std::to_string(canId) << " are: " << std::endl
                  << " min pos" << std::to_string(minMotorPosition) << " max pos: " << std::to_string(maxMotorPosition) << std::endl
                  << " soft min pos" << std::to_string(softMinPosition) << " soft max pos: " << std::to_string(softMaxPosition) << " percentage is: " << std::to_string(watchdogPosPercentage) << std::endl;

        // Initialize high pid params
    }

    Md80::Md80(uint16_t _canID)
    {
        canId = _canID;
        commandFrame.canId = _canID;
        motorStatus["position"] = 0.0;
        motorStatus["velocity"] = 0.0;
        motorStatus["torque"] = 0.0;
        motorStatus["time"] = 0.0;
        motorStatus["seq"] = 0.0;
        motorStatus["savgol_vel"] = 0.0;
        motorStatus["temperature"] = 0.0;
        motorStatus["kalman_vel"] = 0.0;
    }

    Md80::~Md80()
    {
    }

    void Md80::setTargetPosition(float target)
    {
        requestedPosition = target;
    }

    void Md80::updateTargets()
    {
        positionTarget = requestedPosition;
        velocityTarget = requestedVelocity;
        torqueSet = requestorqueSet;
    }

    void Md80::setPIDParams(MotorCommand_T pidParams)
    {

        useHighPid = true;
        std::cout << "[MD80] ########### Notice motor: " << canId << " is using High PID controle ##########" << std::endl;
        h_kp = pidParams["high_p_gain"];
        h_kd = pidParams["high_d_gain"];
        h_ki = pidParams["high_i_gain"];
        h_maxAggError = pidParams["high_max_agg"];
        h_limit_scale = pidParams["high_limit_scale"];
        if (pidParams["agg_window"] < 0)
            h_aggError.push_back(0.0);
        else
        {
            h_aggError.resize(int(pidParams["agg_window"]));
            std::fill(h_aggError.begin(), h_aggError.end(), 0.0);
        }
    }

    void Md80::setSavgolCoeffs(FilterVector coeffs)
    {

        do_savgol = true;
        savgolCoeffs = coeffs;
        for (int i = 0; i < int(coeffs.size()); i++)
            savgolPosBuffer.push_back(0);
        savgolSizeOfBuffer = coeffs.size();
        std::cout << "[MD80] Motor: " << canId << " is calculating savgol filter with coeffs: ";
        for (float x : savgolCoeffs)
            std::cout << x << " ";
        std::cout << std::endl;
    }

    float Md80::savgol(double newPos)
    {
        bool allzero = true;
        for (int i = 0; i < savgolSizeOfBuffer; ++i)
            allzero &= savgolPosBuffer[i] == 0;
        if (allzero)
        {
            // If all filter values are zero, initialize the signal array with the current pos value
            for (int i = 0; i < savgolSizeOfBuffer; ++i)
                savgolPosBuffer[i] = newPos;
        }

        // Shift and update unfiltered signal array
        for (int i = savgolSizeOfBuffer - 1; i > 0; --i)
            savgolPosBuffer[i] = savgolPosBuffer[i - 1];
        savgolPosBuffer[0] = newPos;

        // Compute filter value
        float y = 0;
        for (int i = 0; i < savgolSizeOfBuffer; ++i)
            y += savgolPosBuffer[i] * savgolCoeffs[i];
        return y;
    }

    void Md80::setKalmanFilter(FilterVector processNoiseCov, FilterVector measurmentNoiseCov, FilterVector initailStateError, int frequency)
    {

        dt = 1.0 / double(frequency);
        do_kalman_filter = true;
        std::cout << "[MD80] Motor: " << canId << " is calculating kalman filter. Frequency is " << frequency << "dt is " << dt << std::endl;
        // Kalman filter
        A = Eigen::MatrixXd(number_of_states, number_of_states);         // System dynamics matrix
        C = Eigen::MatrixXd(number_of_mesurments, number_of_states);     // Output matrix
        Q = Eigen::MatrixXd(number_of_states, number_of_states);         // Process noise covariance
        R = Eigen::MatrixXd(number_of_mesurments, number_of_mesurments); // Measurement noise covariance
        P = Eigen::MatrixXd(number_of_mesurments, number_of_mesurments); // Initial state error
                                                                         // Discrete LTI projectile motion, measuring position only
        A << 1, dt, 0, 1;
        C << 1, 0, 0, 1;

        // Reasonable covariance matrices
        Q << processNoiseCov[0], processNoiseCov[1], processNoiseCov[2], processNoiseCov[3];
        R << measurmentNoiseCov[0], measurmentNoiseCov[1], measurmentNoiseCov[2], measurmentNoiseCov[3];
        P << initailStateError[0], initailStateError[1], initailStateError[2], initailStateError[3];

        // std::cout << "[MD80] A: \n"
        //           << A << std::endl;
        // std::cout << "[MD80] C: \n"
        //           << C << std::endl;
        // std::cout << "[MD80] Q: \n"
        //           << Q << std::endl;
        // std::cout << "[MD80] R: \n"
        //           << R << std::endl;
        // std::cout << "[MD80] P: \n"
        //           << P << std::endl;
        kf = KalmanFilter(dt, A, C, Q, R, P);
    }

    float Md80::kalman_filter()
    {
        if (!kf.isIntialized())
        {
            // Best guess of initial states
            Eigen::VectorXd x0(number_of_states);
            double t = 0;
            x0 << position, velocity;
            kf.init(t, x0);
        }
        Eigen::VectorXd y(number_of_mesurments);
        y << position, velocity;
        kf.update(y);
        return kf.state()(1);
    }

    void Md80::pid()
    {
        float currPos = position;
        float posError = requestedPosition - currPos;
        // std::cout << "error: "<< posError << std::endl;
        if (h_aggError.size() == 1)
        {
            h_aggError[0] += posError;
            // std::cout << "agg error: "<< h_aggError[0] << std::endl;
        }
        else
        {
            h_aggError[aggErrCurrIndex] = posError;
            if (aggErrCurrIndex != int(h_aggError.size()))
                aggErrCurrIndex++;
        }
        float agg_err = std::accumulate(h_aggError.begin(), h_aggError.end(), 0.0);
        // std::cout << "agg error accumlate: "<< agg_err << std::endl;
        agg_err = std::clamp(agg_err, -h_maxAggError, h_maxAggError);
        float newPidPos = h_kp * posError - h_kd * velocity + h_ki * agg_err + currPos;
        pidPos = std::clamp(newPidPos, softMinPosition, softMaxPosition);
        // std::cout << "pid_pos: "<< pidPos << std::endl;
    }

    void Md80::watchdog()
    {
        float curr_pos = position;
        // If motor position within range
        if (curr_pos > softMinPosition && curr_pos < softMaxPosition)
        {
            printWatchdog = true;

            if (useHighPid)
                positionTarget = pidPos;
            else
                positionTarget = requestedPosition;

            if (requestorqueSet > maxTorque)
                torqueSet = maxTorque;
            else
                torqueSet = requestorqueSet;

            if (requestKpKdAdjusted)
                setImpedanceControllerParams(requestedImpedanceController.kp, requestedImpedanceController.kd);
        }
        // If motor position is out of range then force it back into the range
        else
        {
            setImpedanceControllerParams(watchdogKP, watchdogKD);
            if (softMinPosition > curr_pos)
            {
                positionTarget = watchdogPosPercentage * softMinPosition;
                torqueSet = watchdogTorqueOffset;
            }
            else // then curr_pos > softMaxPosition
            {
                positionTarget = watchdogPosPercentage * softMaxPosition;
                torqueSet = -1 * watchdogTorqueOffset;
            }
            if (printWatchdog)
            {
                std::cout << "[md80 WATCHDOG ] position of motor " << std::to_string(canId)
                          << " is out of range. Current Position " << std::to_string(curr_pos)
                          << " not in range " << std::to_string(softMinPosition) << " - " << std::to_string(softMaxPosition) << std::endl
                          << "[md80 Watchdog] new targets: " << std::endl
                          << "position: " << std::to_string(positionTarget) << std::endl
                          << "torque: " << std::to_string(torqueSet) << std::endl
                          << "kp: " << std::to_string(impedanceController.kp) << std::endl
                          << "kd: " << std::to_string(impedanceController.kd) << std::endl;
                printWatchdog = false;
            }
        }
    }

    void Md80::setPositionControllerParams(float kp, float ki, float kd, float iWindup)
    {
        regulatorsAdjusted = true;
        positionController.kp = kp;
        positionController.ki = ki;
        positionController.kd = kd;
        positionController.i_windup = iWindup;
    }

    void Md80::setVelocityControllerParams(float kp, float ki, float kd, float iWindup)
    {
        regulatorsAdjusted = true;
        velocityRegulatorAdjusted = true;
        velocityController.kp = kp;
        velocityController.ki = ki;
        velocityController.kd = kd;
        velocityController.i_windup = iWindup;
    }

    void Md80::setImpedanceRequestedControllerParams(float kp, float kd)
    {
        requestKpKdAdjusted = true;
        requestedImpedanceController.kp = kp;
        requestedImpedanceController.kd = kd;
    }

    void Md80::setImpedanceControllerParams(float kp, float kd)
    {
        assert(kp == kd && kp == 0);
        regulatorsAdjusted = true;
        impedanceController.kp = kp;
        impedanceController.kd = kd;
    }

    void Md80::__updateCommandFrame()
    {
        switch (controlMode)
        {
        case Md80Mode_E::IDLE:
            commandFrame.toMd80.length = 2;
            commandFrame.toMd80.data[0] = Md80FrameId_E::FRAME_GET_INFO;
            commandFrame.toMd80.data[1] = 0;
            break;
        case Md80Mode_E::IMPEDANCE:
            if (useHighPid)
                pid();
            watchdog();
            if (regulatorsAdjusted)
                packImpedanceFrame();
            else
                packMotionTargetsFrame();
            break;
        case Md80Mode_E::POSITION_PID:
            updateTargets();
            if (regulatorsAdjusted)
                if (velocityRegulatorAdjusted)
                {
                    velocityRegulatorAdjusted = false;
                    packVelocityFrame();
                }
                else
                    packPositionFrame();
            else
                packMotionTargetsFrame();
            break;
        case Md80Mode_E::VELOCITY_PID:
            updateTargets();
            if (regulatorsAdjusted)
                packVelocityFrame();
            else
                packMotionTargetsFrame();
            break;
        case Md80Mode_E::TORQUE:
            watchdog();
            packMotionTargetsFrame();
            break;
        default:
            break;
        }
    }

    void Md80::__updateResponseData(StdMd80ResponseFrame_t *_responseFrame)
    {
        if (_responseFrame->canId != canId || _responseFrame->fromMd80.data[0] != Md80FrameId_E::RESPONSE_DEFAULT)
            return;
        errorVector = *(uint16_t *)&_responseFrame->fromMd80.data[1];
        prevPosition = position;
        motorStatus["temperature"] = temperature = _responseFrame->fromMd80.data[3];
        motorStatus["position"] = position = *(float *)&_responseFrame->fromMd80.data[4];
        motorStatus["velocity"] = velocity = *(float *)&_responseFrame->fromMd80.data[8];
        motorStatus["torque"] = torque = *(float *)&_responseFrame->fromMd80.data[12];
    }

    void Md80::__updateResponseData(StdMd80ResponseFrame_t *_responseFrame, double time, int seq)
    {
        this->__updateResponseData(_responseFrame);
        if (do_savgol)
            motorStatus["savgol_vel"] = savgolVelocity = savgol(position);
        if (do_kalman_filter)
            motorStatus["kalman_vel"] = kalman_filter();
        motorStatus["our_velocity"] = (position - prevPosition) / (time - prevTime);
        motorStatus["time"] = time;
        motorStatus["seq"] = seq;
        prevTime = time;
    }

    void Md80::__updateRegulatorsAdjusted(bool adjusted)
    {
        this->regulatorsAdjusted = adjusted;
    }

    // advanced setters
    void Md80::setMaxTorque(float _maxTorque)
    {
        maxTorque = _maxTorque;
    }

    void Md80::setMaxVelocity(float _maxVelocity)
    {
        maxVelocity = _maxVelocity;
    }

    void Md80::__setControlMode(Md80Mode_E mode)
    {
        controlMode = mode;
    }

    void Md80::packImpedanceFrame()
    {
        commandFrame.toMd80.length = 32;
        commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_IMP_CONTROL;
        commandFrame.toMd80.data[1] = 0x00;
        *(float *)&commandFrame.toMd80.data[2] = impedanceController.kp;
        *(float *)&commandFrame.toMd80.data[6] = impedanceController.kd;
        *(float *)&commandFrame.toMd80.data[10] = positionTarget;
        *(float *)&commandFrame.toMd80.data[14] = velocityTarget;
        *(float *)&commandFrame.toMd80.data[18] = torqueSet;
        *(float *)&commandFrame.toMd80.data[22] = maxTorque;
    }

    void Md80::packPositionFrame()
    {
        commandFrame.toMd80.length = 32;
        commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_POS_CONTROL;
        commandFrame.toMd80.data[1] = 0x00;
        *(float *)&commandFrame.toMd80.data[2] = positionController.kp;
        *(float *)&commandFrame.toMd80.data[6] = positionController.ki;
        *(float *)&commandFrame.toMd80.data[10] = positionController.kd;
        *(float *)&commandFrame.toMd80.data[14] = positionController.i_windup;
        *(float *)&commandFrame.toMd80.data[18] = maxVelocity;
        *(float *)&commandFrame.toMd80.data[22] = positionTarget;
    }

    void Md80::packVelocityFrame()
    {
        commandFrame.toMd80.length = 32;
        commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_VEL_CONTROL;
        commandFrame.toMd80.data[1] = 0x00;
        *(float *)&commandFrame.toMd80.data[2] = velocityController.kp;
        *(float *)&commandFrame.toMd80.data[6] = velocityController.ki;
        *(float *)&commandFrame.toMd80.data[10] = velocityController.kd;
        *(float *)&commandFrame.toMd80.data[14] = velocityController.i_windup;
        *(float *)&commandFrame.toMd80.data[18] = maxTorque;
        *(float *)&commandFrame.toMd80.data[22] = velocityTarget;
    }

    void Md80::packMotionTargetsFrame()
    {
        commandFrame.toMd80.length = 16;
        commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_SET_MOTION_TARGETS;
        commandFrame.toMd80.data[1] = 0x00;
        *(float *)&commandFrame.toMd80.data[2] = velocityTarget;
        *(float *)&commandFrame.toMd80.data[6] = positionTarget;
        *(float *)&commandFrame.toMd80.data[10] = torqueSet;
    }
}
