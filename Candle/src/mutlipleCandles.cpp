#include "multipleCandles.hpp"

namespace mab
{

    MultipleCandles::MultipleCandles(bool useLogs)
    {
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

    CandleResponse_T MultipleCandles::addMd80(IdList_T idList)
    {
        CandleResponse_T retVal;
        for (auto &id : idList)
        {
            unsigned int md80NotFound = 0;
            unsigned int md80Found = 0;
            mab::Candle *foundCandle;
            for (auto candle : candleInstances)
            {
                if (candle->addMd80(id, false) == true)
                {
                    retVal.push_back(true);
                    md80Found++;
                    foundCandle = candle;
                }
                else
                    md80NotFound++;
            }
            /* if the id was found on multiple CANdle devices */
            if (md80Found > 1)
                candleHandlerOut << "Drive with ID " << id << "seem to be duplicated" << std::endl;
            /* if the drive was not found on any of CANdle devices */
            else if (md80NotFound == candleInstances.size())
                retVal.push_back(false);
            else
            { // Then we found one candle
                md80Instances[id] = foundCandle->getMd80PointerFromList(id);
            }
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
                candleHandlerOut << "Drive with ID" << id << "is not added!" << std::endl;
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
                candleHandlerOut << "Drive with ID: " << id << "was not enabled because not found in Candle" << std::endl;
            }
        }

        /* begin the communication on each CANdle */
        for (auto candle : candlesToBegin)
        {
            candle->begin();
        }
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

        return retVal;
    }

    MultipleMotorsStatus_T MultipleCandles::getAllMotorsData()
    {
        MultipleMotorsStatus_T retVal;
        for (auto candle : candleInstances)
        {
            for (auto &md : candle->md80s)
            {
                retVal[md.getId()] = md.getMotorStatus();
            }
        }

        return retVal;
    }

    MultipleMotorsStatus_T MultipleCandles::getMotorsData(IdList_T idList)
    {
        MultipleMotorsStatus_T retVal;
        for (auto &id : idList)
        {
            if (md80Instances.find(id) != md80Instances.end())
                retVal[id] = md80Instances[id]->getMotorStatus();
        }
        return retVal;
    }

    void MultipleCandles::sendMotionCommand(int frameId, MotorCommands_T motorCommands)
    {

        if (_useLogs)
        {
            uint64_t nsec = std::chrono::duration_cast<nsec_t>(std::chrono::system_clock::now().time_since_epoch()).count();
            double timeInSec = nsec * 1e-9;

            logFile << frameId << "," << timeInSec << "\n";
        }
        for (auto i : motorCommands)
        {
            try
            {
                if (md80Instances.find(i.first) != md80Instances.end())
                {
                    auto md = md80Instances[i.first];
                    md->setTargetPosition(i.second["position"]);
                    md->setTargetVelocity(i.second["velocity"]);
                    md->setTorque(i.second["torque"]);
                    if (i.second.find("kp") != i.second.end())
                        md->setImpedanceControllerParams(i.second["kp"], i.second["kd"]);
                }
                else
                    candleHandlerOut << "Drive with ID: " << i.first << "was not found so command not set" << std::endl;
            }
            catch (const char *eMsg)
            {
                candleHandlerOut << eMsg << std::endl;
            }
        }
    }

    void MultipleCandles::setImpedanceParamters(MotorCommands_T impedanceParams)
    {
        for (auto i : impedanceParams)
        {
            try
            {
                if (md80Instances.find(i.first) != md80Instances.end())
                {
                    auto md = md80Instances[i.first];
                    md->setImpedanceControllerParams(i.second["kp"], i.second["kd"]);
                    md->setMaxTorque(i.second["max_torques"]);
                }
                else
                    candleHandlerOut << "Drive with ID: " << i.first << "was not found so command not set" << std::endl;
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
        for (auto candle : candleInstances)
        {
            for (auto id : candle->md80s)
            {
                if (id.getId() == md80Id)
                    return candle;
            }
        }
        return NULL;
    }
} // namespace mab
