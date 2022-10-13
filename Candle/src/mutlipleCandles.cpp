#include "multipleCandles.hpp"

#include <unistd.h>
namespace mab
{

    MultipleCandles::MultipleCandles(bool useLogs)
    {
        rclcpp::init(0, NULL);
        node_ = std::make_shared<rclcpp::Node>("candle_publisher");
        jointStatePub = node_->create_publisher<sensor_msgs::msg::JointState>("candle/joint_states", 10);
        while (1)
        {
            try
            {
                auto candle = new mab::Candle(mab::CAN_BAUD_1M, true, useLogs, mab::CANdleFastMode_E::NORMAL, false);
                std::cout << "[CANDLE] Found CANdle with ID: " << candle->getUsbDeviceId() << std::endl;
                candleInstances.push_back(candle);
            }
            catch (const char *eMsg)
            {
                break;
            }
        }
        _useLogs = useLogs;
        if (useLogs)
        {
            std::string homedir = getenv("HOME");
            std::string file_string = homedir + "/log/latest/candle_log.csv";
            candleHandlerOut << file_string << std::endl;
            logFile.open(file_string, std::fstream::out);
            logFile << "frame_id,time\n";
        }
    }

    CandleResponse_T MultipleCandles::addMd80(MotorCommands_T motorsConfig)
    {
        CandleResponse_T retVal;
        for (auto &[id, motorConf]: motorsConfig)
        {
            unsigned int md80Found = 0;
            for (int i = 0; i < (int)candleInstances.size(); i++)
            {

                if (candleInstances[i]->addMd80(id, motorConf, false) == true)
                {
                    retVal.push_back(true);
                    motorIdToCandleId[id] = i;
                    md80Found++;
                }
            }
            /* if the id was found on multiple CANdle devices */
            if (md80Found == 0)
            {
                candleHandlerOut << "Drive with ID " << id << "was not found" << std::endl;
                retVal.push_back(false);
            }
            if (md80Found > 1)
                candleHandlerOut << "Drive with ID " << id << "seem to be duplicated" << std::endl;
        }

        /* collect total number of drives from all CANdle devices */
        for (auto candle : candleInstances)
        {
            candle->updateModeBasedOnMd80List();
        }
        return retVal;
    }

    CandleResponse_T MultipleCandles::zeroMd80t(IdList_T idList)
    {
        CandleResponse_T retVal;
        for (auto &id : idList)
        {
            auto candle = findCandleByMd80Id(id);
            if (candle != NULL)
                retVal.push_back(candle->controlMd80SetEncoderZero(id));
            else
            {
                retVal.push_back(false);
                candleHandlerOut << "Drive with ID" << id << "was not found. No zero command add!" << std::endl;
            }
        }
        return retVal;
    }

    CandleResponse_T MultipleCandles::setModeMd80(IdList_T idList, std::string reqMode)
    {
        CandleResponse_T retVal;
        mab::Md80Mode_E mode = mab::Md80Mode_E::IDLE;

        if (reqMode == "IMPEDANCE")
            mode = mab::Md80Mode_E::IMPEDANCE;
        else if (reqMode == "POSITION_PID")
            mode = mab::Md80Mode_E::POSITION_PID;
        else if (reqMode == "VELOCITY_PID")
            mode = mab::Md80Mode_E::VELOCITY_PID;
        else if (reqMode == "TORQUE")
            mode = mab::Md80Mode_E::TORQUE;
        else
        {
            candleHandlerOut << "MODE" << reqMode << "not recognized setting to idle instead" << std::endl;
            reqMode = mab::Md80Mode_E::IDLE;
        }

        for (int i = 0; i < (int)idList.size(); i++)
        {
            auto candle = findCandleByMd80Id(idList[i]);
            if (candle != NULL)
                retVal.push_back(candle->controlMd80Mode(idList[i], mode));
            else
            {
                retVal.push_back(false);
                candleHandlerOut << "Drive with ID: " << idList[i] << " mode was not found so mode was not set!" << std::endl;
            }
        }
        return retVal;
    }

    void MultipleCandles::publish()
    {   
        while(!stopMe)
        {
            MultipleMotorsStatus_T motorsData = this->getAllMotorsData();
            sensor_msgs::msg::JointState jointStateMsg;
            jointStateMsg.header.stamp = rclcpp::Clock().now();
            for (auto const &[motorId, data] : motorsData)
            {
                jointStateMsg.name.push_back(std::string(std::to_string(motorId)));
                jointStateMsg.position.push_back(data.at("position"));
                jointStateMsg.velocity.push_back(data.at("velocity"));
                jointStateMsg.effort.push_back(data.at("torque"));
            }
            this->jointStatePub->publish(jointStateMsg);
            usleep(1990 * 2);
        }

    }

    CandleResponse_T MultipleCandles::enableAllMotors()
    {
        CandleResponse_T retVal;

        for (auto candle : candleInstances)
        {
            for (auto &id : candle->md80Ids)
            {
                bool enabled = candle->controlMd80Enable(id, true);
                retVal.push_back(enabled);
                if (!enabled)
                    candleHandlerOut << "Enabling Motor " << id << "failed" << std::endl;
            }
        }

        /* begin the communication on each CANdle */
        for (auto candle : candleInstances)
        {
            candle->begin();
        }
        stopMe = false;
        publishThread = std::thread(&MultipleCandles::publish, this);
        return retVal;
    }

    CandleResponse_T MultipleCandles::enableSomeMotors(IdList_T idList)
    {
        CandleResponse_T retVal;
        std::vector<mab::Candle *> candlesToBegin;

        for (auto &id : idList)
        {
            auto candle = findCandleByMd80Id(id);
            if (candle != NULL)
            {
                retVal.push_back(candle->controlMd80Enable(id, true));
                /* this is to ensure only one copy of CANdle object is present for the begin procedure */
                if (std::find(candlesToBegin.begin(), candlesToBegin.end(), candle) == candlesToBegin.end())
                    candlesToBegin.push_back(candle);
            }
            else
            {
                retVal.push_back(false);
                candleHandlerOut << "Drive with ID: " << id << "was not enabled. Was not found in any Candle" << std::endl;
            }
        }

        /* begin the communication on each CANdle */
        for (auto candle : candlesToBegin)
        {
            candle->begin();
        }
        stopMe = false;
        publishThread = std::thread(&MultipleCandles::publish, this);
        return retVal;
    }


    CandleResponse_T MultipleCandles::disableAllMotors()
    {
        CandleResponse_T retVal;
        for (auto candle : candleInstances)
        {
            candle->end();
            for (auto &id : candle->md80Ids)
            {
                retVal.push_back(candle->controlMd80Enable(id, false));
            }
        }
        stopMe = true;
        if (publishThread.joinable())
            publishThread.join();
        return retVal;
    }

    CandleResponse_T MultipleCandles::disableSomeMotors(IdList_T idList)
    {
        CandleResponse_T retVal;
        std::vector<mab::Candle *> candlesToEnd;

        /* just to find which CANdle devices are commanded to be disabled */
        for (auto &id : idList)
        {
            auto candle = findCandleByMd80Id(id);
            if (candle != NULL)
            {
                if (std::find(candlesToEnd.begin(), candlesToEnd.end(), candle) == candlesToEnd.end())
                    candlesToEnd.push_back(candle);
            }
        }

        /* ending CANdles */
        for (auto candle : candlesToEnd)
        {
            candle->end();
        }

        /* Actually disabling individual MD80s */
        for (auto &id : idList)
        {
            auto candle = findCandleByMd80Id(id);
            if (candle != NULL)
                retVal.push_back(candle->controlMd80Enable(id, false));
            else
            {
                retVal.push_back(false);
                candleHandlerOut << "Drive with ID: " << id << "was not disabled because was not found in any of the candles!" << std::endl;
            }
        }
        stopMe = true;
        if (publishThread.joinable())
            publishThread.join();
        return retVal;
    }

    MultipleMotorsStatus_T MultipleCandles::getAllMotorsData()
    {
        MultipleMotorsStatus_T retVal;
        for (auto candle : candleInstances)
        {
            for (auto &[canId, md] : candle->md80s)
            {
                retVal[canId] = md.getMotorStatus();
            }
        }

        return retVal;
    }

    MultipleMotorsStatus_T MultipleCandles::getMotorsData(IdList_T idList)
    {
        MultipleMotorsStatus_T retVal;
        for (auto &id : idList)
        {
            auto candle = findCandleByMd80Id(id);
            if (candle != NULL)
            {
                auto &md = candle->md80s.at(id);
                retVal[id] = md.getMotorStatus();
            }
            else
                candleHandlerOut << " [getMotorData] Drive with ID: " << id << " doesn't exist" << std::endl;
        }
        return retVal;
    }

    void MultipleCandles::sendMotorCommand(int frameId, MotorCommands_T motorCommands)
    {

        if (_useLogs)
        {
            uint64_t nsec = std::chrono::duration_cast<nsec_t>(std::chrono::system_clock::now().time_since_epoch()).count();
            double timeInSec = nsec * 1e-9;

            logFile << frameId << "," << std::to_string(timeInSec) << "\n";
        }
        for (auto const &[motorId, motorCommand] : motorCommands)
        {
            try
            {
                auto candle = findCandleByMd80Id(motorId);
                if (candle != NULL)
                {
                    auto &md = candle->md80s.at(motorId);
                    md.setFrameId(frameId);
                    md.setTargetPosition(motorCommand.at("position"));
                    md.setTargetVelocity(motorCommand.at("velocity"));
                    md.setTorque(motorCommand.at("torque"));
                    if (motorCommand.find("kp") != motorCommand.end())
                        md.setImpedanceRequestedControllerParams(motorCommand.at("kp"), motorCommand.at("kd"));
                }
                else
                    candleHandlerOut << "Drive with ID: " << motorId << "was not found so command not set" << std::endl;
            }
            catch (const char *eMsg)
            {
                candleHandlerOut << eMsg << std::endl;
            }
        }
    }

    void MultipleCandles::setSavgol(IdList_T idList, FilterVector coeffs)
    {
        for (auto &id : idList)
        {
            auto candle = findCandleByMd80Id(id);
            if (candle != NULL)
            {
                auto &md = candle->md80s.at(id);
                md.setSavgolCoeffs(coeffs);
            }
            else
                candleHandlerOut << " [getMotorData] Drive with ID: " << id << " doesn't exist" << std::endl;
        }
    }

    void MultipleCandles::setKalmanFilter(FilterConfig_T processNoiseCov, FilterConfig_T measurmentNoiseCov, FilterConfig_T initailStateError, int frequency)
    {
        for (auto const &[motorId, m_processNoiseCov]: processNoiseCov)
        {
            auto candle = findCandleByMd80Id(motorId);
            if (candle != NULL)
            {
                auto &md = candle->md80s.at(motorId);
                md.setKalmanFilter(m_processNoiseCov, measurmentNoiseCov[motorId], initailStateError[motorId], frequency);
            }
            else
                candleHandlerOut << " [setKalmanFilter] Drive with ID: " << motorId << " doesn't exist" << std::endl;
        }
    }

    void MultipleCandles::setPIDParams(MotorCommands_T pidParams)
    {
        for (auto const &[motorId, motorCommand] : pidParams)
        {
            try
            {
                auto candle = findCandleByMd80Id(motorId);
                if (candle != NULL)
                {
                    auto &md = candle->md80s.at(motorId);
                    md.setPIDParams(motorCommand);
                }
                else
                    candleHandlerOut << " [setPidParams] Drive with ID: " << motorId << " doesn't exist" << std::endl;
            }
            catch (const char *eMsg)
            {
                candleHandlerOut << eMsg << std::endl;
            }
        }
    }
    
    void MultipleCandles::setImpedanceParameters(MotorCommands_T impedanceParams)
    {
        for (auto const &[motorId, motorCommand] : impedanceParams)
        {
            try
            {
                auto candle = findCandleByMd80Id(motorId);
                if (candle != NULL)
                {
                    auto &md = candle->md80s.at(motorId);
                    md.setImpedanceRequestedControllerParams(motorCommand.at("kp"), motorCommand.at("kd"));
                    md.setMaxTorque(motorCommand.at("max_torque"));
                }
                else
                    candleHandlerOut << " [setImpedanceParamters] Drive with ID: " << motorId << " doesn't exist" << std::endl;
            }
            catch (const char *eMsg)
            {
                candleHandlerOut << eMsg << std::endl;
            }
        }
    }

    void MultipleCandles::setPositionPIDParameters(MotorCommands_T positionPIDParams)
    {
        for (auto const &[motorId, motorCommand] : positionPIDParams)
        {
            try
            {
                auto candle = findCandleByMd80Id(motorId);
                if (candle != NULL)
                {
                    auto &md = candle->md80s.at(motorId);
                    md.setPositionControllerParams(motorCommand.at("p_kp"), motorCommand.at("p_ki"), motorCommand.at("p_kd"), motorCommand.at("p_iwu"));
                    md.setMaxVelocity(motorCommand.at("max_vel"));
                    md.setVelocityControllerParams(motorCommand.at("v_kp"), motorCommand.at("v_ki"), motorCommand.at("v_kd"), motorCommand.at("v_iwu"));
                    md.setMaxTorque(motorCommand.at("max_torque"));
                }
                else
                    candleHandlerOut << " [setPositionParamters] Drive with ID: " << motorId << " doesn't exist" << std::endl;
            }
            catch (const char *eMsg)
            {
                candleHandlerOut << eMsg << std::endl;
            }
        }
    }

    MultipleCandles::~MultipleCandles()
    {
    }

    mab::Candle *MultipleCandles::findCandleByMd80Id(uint16_t md80Id)
    {
        auto it = motorIdToCandleId.find(md80Id);
        if (it == motorIdToCandleId.end())
            return NULL;
        else
            return candleInstances[it->second];
    }
} // namespace mab
