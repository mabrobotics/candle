#include "candle.hpp"
#include <iostream>
#include <fstream>
#include <ctime>
#include <map>
#include <vector>

typedef std::map<int, MotorStatus_T> MultipleMotorsStatus_T;
typedef std::vector<bool> CandleResponse_T;
typedef std::vector<int> IdList_T;
typedef std::vector<float> CommandList_T;
typedef std::map<int, MotorCommand_T> MotorCommands_T;

#define candleHandlerOut std::cout << "[CANDLE HANDLER] "

namespace mab
{
    class MultipleCandles
    {

    private:
        std::vector<mab::Candle *> candleInstances;

        std::ofstream logFile;
        mab::Candle *findCandleByMd80Id(uint16_t md80Id);
        std::map<int, int> motorIdToCandleId;

        bool _useLogs;

    public:
        MultipleCandles(bool useLogs);
        CandleResponse_T addMd80(MotorCommands_T motorsConfig);
        CandleResponse_T zeroMd80t(IdList_T idList);
        CandleResponse_T setModeMd80(IdList_T idList, std::string reqMode);
        CandleResponse_T enableAllMotors();
        CandleResponse_T enableSomeMotors(IdList_T idList);
        CandleResponse_T disableAllMotors();
        CandleResponse_T disableSomeMotors(IdList_T idList);
        MultipleMotorsStatus_T getMotorsData(IdList_T idList);
        MultipleMotorsStatus_T getAllMotorsData();

        void sendMotorCommand(int frameId, MotorCommands_T motorCommands);
        void setImpedanceParameters(MotorCommands_T impedanceParams);
        void setPositionPIDParameters(MotorCommands_T positionPIDParams);

        ~MultipleCandles();
    };

}