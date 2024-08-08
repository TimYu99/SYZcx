#ifndef SONARAPP_H_
#define SONARAPP_H_

//------------------------------------------ Includes ----------------------------------------------

#include "app.h"
#include "devices/sonar.h"
#include "helpers/sonarDataStore.h"
#include "helpers/sonarImage.h"
#include <string>
#include <vector>
#include <mutex>
#include <windows.h> // 添加 Windows API 头文件

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    struct Message {
        char header[7];
        uint16_t length;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t status;
        uint16_t angle;
        uint8_t speed;
        uint8_t minrange;
        uint8_t maxrange;
        uint16_t reserved;
        uint16_t checksum;
        uint8_t end1;
        uint8_t end2;
    };

    std::string messageToString(const Message& msg);


    class SonarApp : public App
    {
    public:

        SonarApp(void);

        //struct Message {
        //    char header[7];
        //    uint16_t length;
        //    uint16_t year;
        //    uint8_t month;
        //    uint8_t day;
        //    uint8_t hour;
        //    uint8_t minute;
        //    uint8_t second;
        //    uint8_t status;
        //    uint16_t angle;
        //    uint8_t speed;
        //    uint32_t reserved;
        //    uint16_t checksum;
        //    uint8_t end1;
        //    uint8_t end2;
        //};
        std::string header;
        uint16_t length;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t status;
        uint16_t angle;
        uint8_t speed;
        uint32_t reserved;
        uint16_t checksum;
        uint8_t end1;
        uint8_t end2;
        void processReceivedData(const std::vector<uint8_t>& data);
        void SonarApp::readUartData(const std::string& portName, uint32_t baudrate);
        void initializeTime();
        uint16_t addcalculateChecksum(const uint8_t* data, size_t size);
        void SonarApp::sendFormattedData
        (
            const std::string& portName,
            uint32_t baudrate,
            uint16_t year, uint8_t month, uint8_t day,
            uint8_t hour, uint8_t minute, uint8_t second,
            uint8_t status, uint16_t angle, uint8_t speed,
            uint8_t minrange,
            uint8_t maxrange
        );
        void processData(const std::vector<uint16_t>& beam_data);

        ~SonarApp(void);
        void renderPalette(const std::string& path);
        void connectSignals(Device& device) override;
        void disconnectSignals(Device& device) override;
        void doTask(int_t key, const std::string& path) override;
        //void callbackPingData(Sonar& iss360, const Sonar::Ping& ping) override;
        //新添加

        //

        Slot<Ahrs&, uint64_t, const Quaternion&, real_t, real_t> slotAhrsData{ this, &SonarApp::callbackAhrs };
        Slot<GyroSensor&, const Vector3&> slotGyroData{ this, &SonarApp::callbackGyroData };
        Slot<AccelSensor&, const Vector3&> slotAccelData{ this, &SonarApp::callbackAccelData };
        Slot<MagSensor&, const Vector3&> slotMagData{ this, &SonarApp::callbackMagData };
        Slot<AccelSensor&, Vector3::Axis, const Vector3&, uint_t> slotAccelCal{ this, &SonarApp::callbackAccelCal };

        Slot<Sonar&, bool_t, Sonar::Settings::Type> slotSettingsUpdated{ this, &SonarApp::callbackSettingsUpdated };
        Slot<Sonar&, const Sonar::HeadHome&> slotHeadHome{ this, &SonarApp::callbackHeadHome };
        Slot<Sonar&, const Sonar::Ping&> slotPingData{ this, &SonarApp::callbackPingData };
        Slot<Sonar&, const Sonar::Echos&> slotEchoData{ this, &SonarApp::callbackEchoData };
        Slot<Sonar&, const Sonar::CpuPowerTemp& > slotPwrAndTemp{ this, &SonarApp::callbackPwrAndTemp };;
        void sendUartData(const std::string& portName, uint32_t baudrate, const std::vector<uint8_t>& data);



    private:
        Palette m_palette;
        SonarImage m_circular;
        SonarImage m_texture;
        uint_t m_pingCount;
        SonarDataStore sonarDataStore;
        //std::string allPingDataFilename_;  // 用于存储 all_ping_data.txt 文件的完整路径

        //新添加
        std::string dataFolder_;
        std::string allPingDataFilename_;
        bool m_scanning;
        std::ofstream m_outputFile;
        int sequenceNumber_; // 序号变量

        void setDataFolder(const std::string& basePath);
        void updatePingDataFilename();
        void writeInitialLog();
        //void recordPingData(const Sonar::Ping& ping);
        void SonarApp::recordPingData(const Sonar& iss360, const Sonar::Ping& ping,  uint_t txPulseLengthMm);
        //void setDataFolder(const std::string& folderPath);
        //
        virtual void connectEvent(Device& device);
        void callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount);
        void callbackGyroData(GyroSensor& gyro, const Vector3& v);
        void callbackAccelData(AccelSensor& accel, const Vector3& v);
        void callbackMagData(MagSensor& mag, const Vector3& v);
        void callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress);

        void callbackSettingsUpdated(Sonar& iss360, bool_t ok, Sonar::Settings::Type settingsType);
        void callbackHeadHome(Sonar& iss360, const Sonar::HeadHome& data);
        void callbackEchoData(Sonar& iss360, const Sonar::Echos& data);
        void callbackPingData(Sonar& iss360, const Sonar::Ping& data);
        void callbackPwrAndTemp(Sonar& iss360, const Sonar::CpuPowerTemp& data);


    };
    class SerialPort {
    private:
        HANDLE hComm;  // 串口句柄
        DCB dcbSerialParams;  // 串口参数
        COMMTIMEOUTS timeouts;  // 超时参数

    public:
        SerialPort();
        ~SerialPort();
        bool open(const std::string& portName, int baudRate);
        bool close();
        bool read(char* buffer, int bufferSize, int& bytesRead);
        bool write(const char* buffer, int bufferSize, int& bytesWritten);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
