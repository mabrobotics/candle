#pragma once

#include "usbDevice.hpp"
#include "mab_types.hpp"
#include "md80.hpp"

#include <string>
#include <thread>
#include <vector>
#include <iostream>
namespace mab
{
    enum CANdleMode_E
    {
        CONFIG,
        UPDATE
    };

    /**
     * @enum CANdleBaudrate_E
     * @brief Enum with coded FDCAN baudrates
     */
    enum CANdleBaudrate_E : uint8_t
    {
        CAN_BAUD_1M = 1,    /*!< FDCAN Baudrate of 1Mbps (1 000 000 bits per second) */
        CAN_BAUD_2M = 2,    /*!< FDCAN Baudrate of 2Mbps (2 000 000 bits per second) */
        CAN_BAUD_5M = 5,    /*!< FDCAN Baudrate of 5Mbps (5 000 000 bits per second) */
        CAN_BAUD_8M = 8,    /*!< FDCAN Baudrate of 8Mbps (8 000 000 bits per second) */
    };

    /*! \class Candle
        \brief Class for communicating with CANdle (USB-CAN converter) and Md80 drives.

        This class is an connector between non-realtime user code and real-time CANdle firmware. It can be used to 
        configure Md80's via FDCAN, as well as control them.
        It can automatically communicate with CANdle/Md80s to relieve the user from requiring them to manually 
        control USB and FDCAN communications.
    */
    class Candle
    {
    private:
        UsbDevice*usb;
        std::thread receiverThread;
        std::thread transmitterThread;
        CANdleMode_E mode = CANdleMode_E::CONFIG;

        const int MAX_DEVICES = 12;
        bool shouldStopReceiver;
        bool shouldStopTransmitter;

        int msgsReceived = 0;
        int msgsSent = 0;

        bool printVerbose = true;

        void transmitNewStdFrame();

        void receive();
        void transmit();

        bool inUpdateMode();
        bool inConfigMode();
    public:
        /**
         * @brief A constructor of Candle class
         * @param canBaudrate Sets a baudrate that CANdle will use to talk to drives
         * @param printVerbose if true, additional printing will be enables. Usefull for debugging
         * @return A functional CANdle class object if succesfull, a nullptr if critical failure occured.
        */
        Candle(CANdleBaudrate_E canBaudrate, bool printVerbose = false);
        /**
         * @brief A destructor of Candle class. Takes care of all started threads that need to be stopped before clean exit
        */
        ~Candle();

        /**
         * @brief A vector holding all md80 instances that were succesfully added via `addMd80` method. This vector
         * can be used to modify regulator and control parameters of the md80 drives.
         */
        std::vector<Md80> md80s;

        /**
        @brief Sends a FDCAN Frame to IDs in range (10 - 2047), and checks for valid responses from Md80;
        @return the vector FDCAN IDs of drives that were found. If no drives were found, the vector is empty
        */
        std::vector<uint16_t> ping();

        /**
        @brief Sends a Generic FDCAN Frame to the IDs in range (10 - 2047), and checks for valid responses from Md80;
        @param canId FDCAN ID of the device
        @param msgLen length of FDCAN message
        @param txBuffer pointer to data buffer to be transmited
        @param rxBuffer pointer to data buffer for storing a response. Buffer should be 64 bytes long.
        @param timeoutMs timeout for receiving in milliseconds
        @return true if received response, false otherwise
        */
        bool sengGenericFDCanFrame(uint16_t canId, int msgLen, const char*txBuffer, char*rxBuffer, int timeoutMs = 100);

        /**
        @brief Adds Md80 to auto update vector.
        @param canId FDCAN ID of the drive to be added
        @return true if drive has been found and was added, false otherwise
        */
        bool addMd80(uint16_t canId);
        /**
        @brief Changes FDCAN baudrate that CANdle uses to talk to Md80s.
        @param canBaudrate enum listing all available baudrates. CAN_BAUD_1M is equal to baudrate of 1 000 000 bits per second.
        @return true if baudrate was changed, false otherwise
        */
        bool configCandleBaudrate(CANdleBaudrate_E canBaudrate);

        /**
        @brief Changes FDCAN parameters of the Md80.
        @param canId ID of the drive to be modified
        @param newId ID that the drive shall change to
        @param newBaudrateMbps FDCAN baudrate that the drive shall use
        @param newTimeout FDCAN watchdof timeout in milliseconds. If set to 0 the watchdog is disabled.
        @return true if all parameters were changed succesfully, false otherwise
        */
        bool configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout);
        /**
        @brief Changes max phase-to-phase motor current. 
        @param canId ID of the drive
        @param currentLimit phase-to-phase current limit in Amps
        @return true if setting was succesfull, false otherwise
        */
        bool configMd80SetCurrentLimit(uint16_t canId, float currentLimit);
        /**
        @brief Saves FDCAN and Current Limiter settings to Md80's non-volatile memory
        @param canId ID of the drive
        @return true if saveing was succesfull, false otherwise
        */
        bool configMd80Save(uint16_t canId);
        
        /**
        @brief Sets current motor position as zero position -> reference for any future movements. 
        @param drive Pointer to a Md80 class (candle.md80s memeber)
        @return true if setting was succesfull, false otherwise
        */
        bool controlMd80SetEncoderZero(Md80*drive);
        /**
        @brief Changes max phase-to-phase motor current. 
        @param canId ID of the drive
        @param currentLimit phase-to-phase current limit in Amps
        @return true if setting was succesfull, false otherwise
        */
        bool controlMd80SetEncoderZero(uint16_t canId);

        /**
        @brief Sets control mode of the Md80
        @param drive Pointer to a Md80 class (candle.md80s memeber)
        @param mode Control mode to be used on the drive
        @return true if setting was succesfull, false otherwise
        */
        bool controlMd80Mode(Md80*drive, Md80Mode_E mode);
        /**
        @brief Sets control mode of the Md80
        @param canId ID of the drive
        @param mode Control mode to be used on the drive
        @return true if setting was succesfull, false otherwise
        */
        bool controlMd80Mode(uint16_t canId, Md80Mode_E mode);

        /**
        @brief Enables/disabled actuaction of the Md80
        @param drive Pointer to a Md80 class (candle.md80s memeber)
        @param enable if true the drive will be enabled, if false the drive will be disabled
        @return true if setting was succesfull, false otherwise
        */
        bool controlMd80Enable(Md80*drive, bool enable);
        /**
        @brief Enables/disabled actuaction of the Md80
        @param canId ID of the drive
        @param enable if true the drive will be enabled, if false the drive will be disabled
        @return true if setting was succesfull, false otherwise
        */
        bool controlMd80Enable(uint16_t canId, bool enable);

        /**
        @brief Searched if the drive with provided FDCAN canId exists in `Md80s` list (exists only if was previously added 
        by `addMd80` method)
        @param canId ID of the drive
        @return a pointer to a drive if found, nullptr otherwise
        */
        Md80* getMd80FromList(uint16_t canId);
        
        /**
        @brief Begins auto update mode. In this mode, host and CANdle will automatically exchange USB messages with md80 commands
        and states. In this mode CANdle will automatically send commands and gather state from all Md80's added to update
        vector with `::addMd80` method. In this mode no config* or control* methods can be called.
        @return true if mode was set succesfully, false otherwise
        */
        bool begin();
        /**
        @brief Ends auto update mode. Sets mode back to idle (config) mode. In this mode, control*, config* and other methods
        can be used.
        @return true if mode was set succesfully, false otherwise
        */
        bool end();
        /**
        @brief Clears all parameters set on the CANdle, returning it to the state same as after a powerup.
        @return true if reset was succesfull, false otherwise
        */
        bool reset();

        /**
        @brief Triggers a calibration routine of the drive's internal electronics.
        @note **This method should not be ever used without consultation with MAB Robotics**, it may make drive unusable and
        prone to fail if used incorrectly.
        @param canId ID of the drive
        @return true if the calibration started succesfully, false otherwise
        */
        bool setupMd80Calibration(uint16_t canId);
        /**
        @brief Prints diagnostic message from md80.
        @param canId ID of the drive
        @return true if the succesfull, false otherwise
        */
        bool setupMd80Diagnostic(uint16_t canId);
    };
}
