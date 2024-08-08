//------------------------------------------ Includes ----------------------------------------------

#include "sonarApp.h"
#include "maths/maths.h"
#include "platform/debug.h"
#include "files/bmpFile.h"
#include "utils/utils.h"
#include <iostream> // 包含头文件
#include <ctime> // For std::time_t, std::time, std::ctime
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <direct.h> 
#include <filesystem> // 使用 std::filesystem 来代替 _mkdir
#include "platform/windows/uart.h" // 确保包含了正确的头文件
#include <vector>
#include <string>
#include "devices/device.h"
#include "global.h" // 包含 global.h
#include <thread> // 包含线程库
#include <windows.h> // 包含 Windows API 头文件
#include <mutex>
//#include "global.h"
// 在类中添加一个标志来表示是否开始了 ping data 的记录
bool isRecording = false;
std::string dataFolder_;  // 文件夹路径
using namespace IslSdk;
unsigned char mesbag[28] = { 0 };
std::mutex uartMutex;
// 定义全局变量
Message  msg;

std::string IslSdk::messageToString(const Message& msg)
{
    std::ostringstream oss;

    // 将header转换为字符串
    oss.write(msg.header, sizeof(msg.header));

    // 将其他字段按顺序添加到字符串流中
    oss.write(reinterpret_cast<const char*>(&msg.length), sizeof(msg.length));
    oss.write(reinterpret_cast<const char*>(&msg.year), sizeof(msg.year));
    oss.write(reinterpret_cast<const char*>(&msg.month), sizeof(msg.month));
    oss.write(reinterpret_cast<const char*>(&msg.day), sizeof(msg.day));
    oss.write(reinterpret_cast<const char*>(&msg.hour), sizeof(msg.hour));
    oss.write(reinterpret_cast<const char*>(&msg.minute), sizeof(msg.minute));
    oss.write(reinterpret_cast<const char*>(&msg.second), sizeof(msg.second));
    oss.write(reinterpret_cast<const char*>(&msg.status), sizeof(msg.status));
    oss.write(reinterpret_cast<const char*>(&msg.angle), sizeof(msg.angle));
    oss.write(reinterpret_cast<const char*>(&msg.speed), sizeof(msg.speed));
    oss.write(reinterpret_cast<const char*>(&msg.minrange), sizeof(msg.minrange));
    oss.write(reinterpret_cast<const char*>(&msg.maxrange), sizeof(msg.maxrange));
    oss.write(reinterpret_cast<const char*>(&msg.reserved), sizeof(msg.reserved));
    oss.write(reinterpret_cast<const char*>(&msg.checksum), sizeof(msg.checksum));
    oss.write(reinterpret_cast<const char*>(&msg.end1), sizeof(msg.end1));
    oss.write(reinterpret_cast<const char*>(&msg.end2), sizeof(msg.end2));

    return oss.str();
}

SerialPort::SerialPort() {
    hComm = INVALID_HANDLE_VALUE;
    SecureZeroMemory(&dcbSerialParams, sizeof(DCB));
    SecureZeroMemory(&timeouts, sizeof(COMMTIMEOUTS));
}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& portName, int baudRate) {
    hComm = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hComm == INVALID_HANDLE_VALUE) {
        std::cout << "无法打开串口!" << std::endl;
        return false;
    }

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hComm, &dcbSerialParams)) {
        std::cout << "获取串口状态失败!" << std::endl;
        close();
        return false;
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hComm, &dcbSerialParams)) {
        std::cout << "设置串口状态失败!" << std::endl;
        close();
        return false;
    }

    timeouts.ReadIntervalTimeout = 5;
    timeouts.ReadTotalTimeoutConstant = 5;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 5;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    if (!SetCommTimeouts(hComm, &timeouts)) {
        std::cout << "设置串口超时失败!" << std::endl;
        close();
        return false;
    }

    return true;
}

bool SerialPort::close() {
    if (hComm != INVALID_HANDLE_VALUE) {
        CloseHandle(hComm);
        hComm = INVALID_HANDLE_VALUE;
    }
    return true;
}

bool SerialPort::read(char* buffer, int bufferSize, int& bytesRead) {
    return ReadFile(hComm, buffer, bufferSize, (LPDWORD)&bytesRead, NULL);
}

bool SerialPort::write(const char* buffer, int bufferSize, int& bytesWritten) {
    return WriteFile(hComm, buffer, bufferSize, (LPDWORD)&bytesWritten, NULL);
}
std::vector<uint8_t> data1 =
{
   0x24, 0x53, 0x4d, 0x53, 0x4e, 0x58, 0x58, 0x1c, 0x00, 0xe8, 0x07, 0x06, 0x19, 0x15, 0x01,
   0x28, 0x0f, 0x5a, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x0d, 0x0a
};
namespace msis_pkg 
{

    struct RangeImageBeam
    {
        float angle_du;
        float angle_stepSize;
        float minRange;
        float max_range;
        int data_count;
        std::vector<float> beam_data;

        // 声明构造函数
        RangeImageBeam();
    };

    // 定义构造函数
    RangeImageBeam::RangeImageBeam() : angle_du(0.0f), angle_stepSize(0.0f), minRange(0.0f), max_range(0.0f), data_count(0) {
        // 在这里进行其他的初始化操作
    }

} // namespace msis_pkg
//--------------------------------------------------------------------------------------------------
SonarApp::SonarApp(void) : App("SonarApp"), m_pingCount(0), m_scanning(false), sequenceNumber_(1)
{
    //setDataFolder("G:\\508 Project\\cunshu121");
    setDataFolder("C:\\ceshi"); 
    header = "$SMSNXX";
    length = 28;  // 计算消息长度，排除结束符
    initializeTime();
    status = 0x0F;  // 示例状态
    angle = 90;     // 示例角度
    speed = 2;      // 示例速度
    reserved = 0;
    end1 = 0x0D;    // CR
    end2 = 0x0A;    // LF
    Debug::log(Debug::Severity::Notice, name.c_str(), "created" NEW_LINE
        "d -> Set settings to defualt" NEW_LINE
        "s -> Save settings to file" NEW_LINE
        "r -> Start scaning" NEW_LINE
        "R -> Stop scaning" NEW_LINE
        "p -> Save palette" NEW_LINE
        "t -> Save sonar texture" NEW_LINE
        "i -> Save sonar image" NEW_LINE
        "q -> FasongDabaoxinxi" NEW_LINE);

    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒，确保设备已初始化
        this->doTask('r', dataFolder_); // 开始扫描
        std::this_thread::sleep_for(std::chrono::minutes(4)); // 等待4分钟
        this->doTask('R', dataFolder_); // 停止扫描
        }).detach();

}



//--------------------------------------------------------------------------------------------------
SonarApp::~SonarApp(void)
{

}
void SonarApp::initializeTime()
{
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);
    year = 1900 + now_tm->tm_year;
    month = now_tm->tm_mon + 1;
    day = now_tm->tm_mday;
    hour = now_tm->tm_hour;
    minute = now_tm->tm_min;
    second = now_tm->tm_sec;
}
uint16_t SonarApp::addcalculateChecksum(const uint8_t* data, size_t size)
{
    uint16_t checksum = 0;
    for (size_t i = 0; i < size; ++i) {
        checksum += data[i];
    }
    return checksum;
}

//--------------------------------------------------------------------------------------------------
void SonarApp::renderPalette(const std::string& path)
{
    const uint_t w = 100, h = 1000;

    std::unique_ptr<Palette> p = std::make_unique<Palette>();
    std::vector<uint32_t> buf(w * h);

    p->render(&buf[0], w, h, false);
    BmpFile::save(path + "palette.bmp", &buf[0], 32, w, h);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::connectSignals(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    sonar.ahrs.onData.connect(slotAhrsData);                // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //sonar.gyro.onData.connect(slotGyroData);              // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //sonar.accel.onData.connect(slotAccelData);            // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    //sonar.mag.onData.connect(slotMagData);                // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    sonar.accel.onCalProgress.connect(slotAccelCal);

    sonar.onSettingsUpdated.connect(slotSettingsUpdated);
    sonar.onHeadHomed.connect(slotHeadHome);
    sonar.onPingData.connect(slotPingData);                 // Subscribing to this event causes ping data to be sent
    sonar.onEchoData.connect(slotEchoData);                 // Subscribing to this event causes profiling data to be sent
    //  sonar.onPwrAndTemp.connect(slotPwrAndTemp);             // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()



    Sonar::SensorRates rates;
    rates.ahrs = 100;
    rates.gyro = 100;
    rates.accel = 100;
    rates.mag = 100;
    rates.voltageAndTemp = 1000;
    sonar.setSensorRates(rates);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::disconnectSignals(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    sonar.ahrs.onData.disconnect(slotAhrsData);
    sonar.gyro.onData.disconnect(slotGyroData);
    sonar.accel.onData.disconnect(slotAccelData);
    sonar.mag.onData.disconnect(slotMagData);
    sonar.accel.onCalProgress.disconnect(slotAccelCal);

    sonar.onSettingsUpdated.disconnect(slotSettingsUpdated);
    sonar.onHeadHomed.disconnect(slotHeadHome);
    sonar.onPingData.disconnect(slotPingData);
    sonar.onEchoData.disconnect(slotEchoData);
    sonar.onPwrAndTemp.disconnect(slotPwrAndTemp);


}
//--------------------------------------------------------------------------------------------------
void SonarApp::doTask(int_t key, const std::string& path)
{
    if (m_device)
    {
        Sonar& sonar = reinterpret_cast<Sonar&>(*m_device);

        switch (key)
        {
        case 'd':
            sonar.setSystemSettings(Sonar::System(), true);
            sonar.setAcousticSettings(Sonar::Acoustic(), true);
            sonar.setSetupSettings(Sonar::Setup(), true);
            break;

        case 's':
            sonar.saveConfig(path + m_device->info.pnSnAsStr() + " settings.xml");
            break;

        case 'r':
            sonar.startScanning();
            if (!isRecording)
            {
                isRecording = true;
                // 在记录开始时生成时间戳
                auto now = std::chrono::system_clock::now();
                std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                // 在开始记录时生成一个新的文件名
                std::stringstream filenameStream;
                filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                std::string filename = filenameStream.str();
                // 打开文件并写入时间戳和序号信息
                std::ofstream outputFile(filename);
                if (outputFile.is_open()) {
                    outputFile << "Recording Started:" << std::endl;
                    outputFile << "Timestamp: " << std::ctime(&timestamp);
                    outputFile << "Sequence Number: 1" << std::endl;
                    outputFile.close();
                    std::cout << "Recording started. Data will be written to " << filename << std::endl;


                }
                else
                {
                    std::cerr << "Unable to open the file for writing." << std::endl;
                }
            }
            break;

        case 'R':
            sonar.stopScanning();
            if (isRecording) {
                isRecording = false;
                // 在记录结束时生成时间戳
                auto now = std::chrono::system_clock::now();
                std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                // 在结束记录时生成一个新的文件名
                std::stringstream filenameStream;
                filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                std::string filename = filenameStream.str();
                // 打开文件并写入时间戳和序号信息
                std::ofstream outputFile(filename, std::ios::app);  // 使用追加模式打开文件
                if (outputFile.is_open()) {
                    // 在结束记录时生成序号信息
                    outputFile << "Recording Ended:" << std::endl;
                    outputFile << "Timestamp: " << std::ctime(&timestamp);
                    outputFile << "Sequence Number: " << sequenceNumber_ << std::endl;
                    outputFile.close();
                    std::cout << "Recording ended. Data written to " << filename << std::endl;
                }
                else {
                    std::cerr << "Unable to open the file for writing." << std::endl;
                }
                // 递增序号
                ++sequenceNumber_;
            }
            break;

        case 'p':
            renderPalette(path);
            break;

        case 'i':
            m_circular.render(sonarDataStore, m_palette, true);
            BmpFile::save(path + "texture.bmp", reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), 32, m_circular.width, m_circular.height);
            break;

        case 't':
            m_texture.renderTexture(sonarDataStore, m_palette, false);
            BmpFile::save(path + "sonar.bmp", reinterpret_cast<const uint32_t*>(&m_texture.buf[0]), 32, m_texture.width, m_texture.height);
            break;

        case 'q':
            //std::string portname = "COM4"; // 根据需要调整端口号
            //uint32_t baudrate = 115200; // 根据需要调整波特率

            //sendUartData("COM6", 115200, data1);
            break;

        default:
            break;
        }

        App::doTask(key, path);
    }
}
//--------------------------------------------------------------------------------------------------
void SonarApp::connectEvent(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    m_circular.setBuffer(1000, 1000, true);
    m_circular.setSectorArea(0, sonar.settings.setup.maxRangeMm, sonar.settings.setup.sectorStart, sonar.settings.setup.sectorSize);
    m_circular.useBilinerInterpolation = true;

    // Optimal texture size to pass to the GPU - each pixel represents a data point. The GPU can then map this texture to circle (triangle fan)
    m_texture.useBilinerInterpolation = false;
    m_texture.setBuffer(sonar.settings.setup.imageDataPoint, Sonar::maxAngle / Math::abs(sonar.settings.setup.stepSize), true);
    m_texture.setSectorArea(0, sonar.settings.setup.maxRangeMm, sonar.settings.setup.sectorStart, sonar.settings.setup.sectorSize);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackAhrs(Ahrs& ahrs, uint64_t timeUs, const Quaternion& q, real_t magHeadingRad, real_t turnsCount)
{
    EulerAngles euler = q.toEulerAngles(0);

    Debug::log(Debug::Severity::Info, name.c_str(), "H:%.1f    P:%.2f    R%.2f", Math::radToDeg(euler.heading), Math::radToDeg(euler.pitch), Math::radToDeg(euler.roll));
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackGyroData(GyroSensor& gyro, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Gyro x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackAccelData(AccelSensor& accel, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Accel x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackMagData(MagSensor& mag, const Vector3& v)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Mag x:%.2f, y:%.2f, z:%.2f", v.x, v.y, v.z);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackAccelCal(AccelSensor& accel, Vector3::Axis axis, const Vector3& v, uint_t progress)
{
    const char* lable[] = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };

    Debug::log(Debug::Severity::Info, name.c_str(), "accel %s   %.2f, %.2f, %.2f,   0x%02x", lable[static_cast<uint_t>(axis)], v.x, v.y, v.z, progress);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackSettingsUpdated(Sonar& iss360, bool_t ok, Sonar::Settings::Type settingsType)
{
    const char* settingsTypeStr[] = { "System", "Acostic", "Setup" };

    if (ok)
    {
        Debug::log(Debug::Severity::Info, name.c_str(), "%s settings updated", settingsTypeStr[static_cast<uint_t>(settingsType)]);

        if (settingsType == Sonar::Settings::Type::Setup)
        {
            m_circular.setSectorArea(0, iss360.settings.setup.maxRangeMm, iss360.settings.setup.sectorStart, iss360.settings.setup.sectorSize);

            // Optimal texture size to pass to the GPU - each pixel represents a data point. The GPU can then map this texture to circle (triangle fan)
            m_texture.setBuffer(iss360.settings.setup.imageDataPoint, Sonar::maxAngle / Math::abs(iss360.settings.setup.stepSize), true);
            m_texture.setSectorArea(0, iss360.settings.setup.maxRangeMm, iss360.settings.setup.sectorStart, iss360.settings.setup.sectorSize);
        }
    }
    else
    {
        Debug::log(Debug::Severity::Warning, name.c_str(), "%s settings failed to update", settingsTypeStr[static_cast<uint_t>(settingsType)]);
    }
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackHeadHome(Sonar& iss360, const Sonar::HeadHome& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Head Homed");
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackPingData(Sonar& iss360, const Sonar::Ping& ping)
{
    if (isRecording)
    {
        static int pingDataCount = 0; // 静态变量用于保持计数器的值
        //Debug::log(Debug::Severity::Info, name.c_str(), "Ping data");
        Debug::log(Debug::Severity::Info, name.c_str(), "Ping data, Count: %d", ++pingDataCount); // 显示计数器的值
        uint_t txPulseLengthMm = static_cast<uint_t>(iss360.settings.setup.speedOfSound * iss360.settings.acoustic.txPulseWidthUs * 0.001 * 0.5);
        txPulseLengthMm = Math::max<uint_t>(txPulseLengthMm, 150);

        sonarDataStore.add(ping, txPulseLengthMm);
        // 记录数据到文件
        recordPingData(iss360, ping,  txPulseLengthMm);
        m_pingCount++;
 
       // sendUartData("COM4", 115200, data1);
        if (m_pingCount % (Sonar::maxAngle / iss360.settings.setup.stepSize) == 0)
        {
            m_pingCount = 0;
            /*
            m_texture.renderTexture(sonarDataStore, m_palette, false);
            BmpFile::save("snrTex.bmp", reinterpret_cast<const uint32_t*>(&m_texture.buf[0]), 32, m_texture.width, m_texture.height);

            m_circular.render(sonarDataStore, m_palette, true);
            BmpFile::save("snrCi.bmp", reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), 32, m_circular.width, m_circular.height);
            */
        }
    }
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackEchoData(Sonar& iss360, const Sonar::Echos& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Echo data, size:%u", data.data.size());
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackPwrAndTemp(Sonar& iss360, const Sonar::CpuPowerTemp& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "CPU:%.1f, SYS:%.1f, 1.0V:%.2f, 1.8V:%.2f, 1.35V:%.2f", data.cpuTemperature, data.auxTemperature, data.core1V0, data.aux1V8, data.ddr1V35);
}
//--------------------------------------------------------------------------------------------------

//新添加




void SonarApp::recordPingData(const Sonar& iss360, const Sonar::Ping& ping,uint_t txPulseLengthMm)
{
    Device::Info adevice;
    msis_pkg::RangeImageBeam temp_ping_;
    temp_ping_.angle_du = float(ping.angle) * 360 / 12800;
    temp_ping_.angle_stepSize = float(ping.stepSize) * 360 / 12800;
    temp_ping_.minRange = float(ping.minRangeMm) / 1000;
    temp_ping_.max_range = float(ping.maxRangeMm) / 1000;
    temp_ping_.data_count = static_cast<int>(ping.data.size());
    
    for (int i_ = 0; i_ < ping.data.size(); i_++) {
        temp_ping_.beam_data.push_back(ping.data[i_]);
    }
    
    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&timestamp);
    uint16_t year = now_tm->tm_year + 1900; // 获取完整的年份
    uint8_t month = now_tm->tm_mon + 1;
    uint8_t day = now_tm->tm_mday;
    uint8_t hour = now_tm->tm_hour;
    uint8_t minute = now_tm->tm_min;
    uint8_t second = now_tm->tm_sec;

    uint8_t status = 0x0F; // 示例状态
    // 设置状态位：Bit0, Bit1, Bit2, Bit3
    // 使用全局变量进行判断
    //std::string deviceInfo = std::to_string(globalPn) + "." + std::to_string(globalSn);
    //if (globalPn == 2255 && globalSn == 10) {
    //    status |= 0x01; // 设置 Bit0 为 1
    //}
    // 计算 angle 和 speed
    uint16_t angle = static_cast<uint16_t>((ping.angle * 65535) / 12800);
    uint8_t speed;
    float stepSizeInDegrees = ping.stepSize * 360.0 / 12800.0;
    if (stepSizeInDegrees < 0.5) 
    {
        speed = 1;
    }
    else if (stepSizeInDegrees < 1.0)
    {
        speed = 2;
    }
    else 
    {
        speed = 3;
    }
    // 添加数据处理逻辑
    //processData(ping.data);
    //// 根据数据处理结果进行反馈
    //bool targetDetected = checkForTarget(temp_ping_.beam_data);
    //if (targetDetected) {
    //    status |= 0x04; // 设置 Bit2 为 1 表示有目标
    //}
    // 示例目标检测函数
    //bool SonarApp::checkForTarget(const std::vector<uint16_t>&beam_data)
    //{
    //    // 在这里添加目标检测逻辑
    //    // 返回 true 表示检测到目标，否则返回 false
    //    return std::any_of(beam_data.begin(), beam_data.end(), [](uint16_t value) {
    //        return value > 100; // 示例条件：值大于 100 表示有目标
    //        });
    //}
    globalcount = temp_ping_.data_count;
    //globalx = 1;
    uint8_t minrange = temp_ping_.minRange;
    uint8_t maxrange = temp_ping_.max_range;



    std::stringstream filenameStream;
    // 生成文件名，使用时间戳作为文件名的一部分

    filenameStream << dataFolder_ << "/all_ping_data1.txt";
    std::string filename = filenameStream.str();

    // 将参数写入文件
    std::ofstream m_outputFile(allPingDataFilename_, std::ios::app);  // 注意这里使用 std::ios::app 进行追加写入
    if (m_outputFile.is_open()) 
    {
        // 记录数据到文件
        m_outputFile << "Processed Ping Data:" << std::endl;
        m_outputFile << "Timestamp: " << std::put_time(std::localtime(&timestamp), "%Y-%m-%d %H:%M:%S") << std::endl;
        m_outputFile << "Angle: " << temp_ping_.angle_du << std::endl;
        m_outputFile << "Step Size: " << temp_ping_.angle_stepSize << std::endl;
        m_outputFile << "Min Range (m): " << temp_ping_.minRange << std::endl;
        m_outputFile << "Max Range (m): " << temp_ping_.max_range << std::endl;
        m_outputFile << "Speed of Sound: " << iss360.settings.setup.speedOfSound << std::endl;
        m_outputFile << "Tx Pulse Width (us): " << iss360.settings.acoustic.txPulseWidthUs << std::endl;
        m_outputFile << "Tx Pulse Length (mm): " << txPulseLengthMm << std::endl;
        m_outputFile << "Data Count: " << ping.data.size() << std::endl;
        m_outputFile << "Beam Data: ";
        for (int i = 0; i < temp_ping_.data_count; ++i) {
            m_outputFile << temp_ping_.beam_data[i] << " ";
        }
        m_outputFile << std::endl;
        m_outputFile.close();
        std::cout << "Data appended to " << allPingDataFilename_ << std::endl;
        

    }
    else 
    {
        std::cerr << "Unable to open the file for writing." << std::endl;
    }

    // 发送格式化数据
    std::string portName = "COM6"; // 根据需要调整
    uint32_t baudrate = 115200; // 根据需要调整
    sendFormattedData(portName, baudrate, year, month, day, hour, minute, second, status, angle, speed ,minrange,maxrange);
 

}


//



void SonarApp::setDataFolder(const std::string& basePath)
{
    dataFolder_ = basePath;

    updatePingDataFilename();
    // writeInitialLog();
}
void SonarApp::updatePingDataFilename()
{
    // 生成文件名，使用时间戳作为文件名的一部分
    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

    std::stringstream filenameStream;
    filenameStream << dataFolder_ << "/all_ping_data_" << std::to_string(timestamp) << ".txt";
    allPingDataFilename_ = filenameStream.str();
    // 将当前时间写入文件
    std::ofstream outputFile(allPingDataFilename_);
    if (outputFile.is_open()) {
        outputFile << "All Ping Data Log:" << std::endl;
        outputFile << "Timestamp: " << std::ctime(&timestamp) << std::endl;
        outputFile.close();
        std::cout << "Initial log created at " << allPingDataFilename_ << std::endl;
    }
    else {
        std::cerr << "Unable to open the file for writing." << std::endl;
    }
}

void SonarApp::writeInitialLog()
{

}
void SonarApp::sendUartData(const std::string& portName, uint32_t baudrate, const std::vector<uint8_t>& data)
{
    // 获取并打印当前可用的串口列表
   /* std::vector<std::string> availablePorts = Uart::getNames();
    std::string portsList = "Available UART ports: ";
    for (const auto& port : availablePorts)
    {
        portsList += port + " ";
    }*/
  //  Debug::log(Debug::Severity::Info, "SonarApp", portsList.c_str());
    Uart uart(portName.c_str());
    if (uart.open())
    {
       // Debug::log(Debug::Severity::Info, "SonarApp", ("UART port opened successfully: " + portName).c_str());
        uart.config(baudrate, 8, Uart::Parity::None, Uart::StopBits::One);
        uart.write(data.data(), data.size(), baudrate);
       // Debug::log(Debug::Severity::Info, "SonarApp", "Data sent via UART");
        uart.close();
      //  Debug::log(Debug::Severity::Info, "SonarApp", ("UART port closed: " + portName).c_str());
    }
    else
    {
        Debug::log(Debug::Severity::Error, "SonarApp", ("Failed to open UART port: " + portName).c_str());
    }
}

void SonarApp::processData(const std::vector<uint16_t>& beam_data)
{
    // 在这里添加数据处理逻辑
    // 例如，计算数据的平均值或检测目标
}
void SonarApp::readUartData(const std::string& portName, uint32_t baudrate)
{
    SerialPort serialPort;
    Uart uart("COM6");
    if (uart.open()) {
        if (!uart.config(baudrate, 8, Uart::Parity::None, Uart::StopBits::One)) {
            Debug::log(Debug::Severity::Error, "SonarApp", "Failed to configure UART port.");
            return;
        }

        while (true) {
            std::vector<uint8_t> buffer(1024);
            {
              std::lock_guard<std::mutex> lock(uartMutex);
                int bytesRead;
                if (!serialPort.read(reinterpret_cast<char*>(buffer.data()), buffer.size(), bytesRead)) {
                    Debug::log(Debug::Severity::Error, "SonarApp", "Failed to read data from UART port.");
                    continue;
                }
                if (bytesRead > 0) {
                    buffer.resize(bytesRead);
                }
                else {
                    continue;
                }
            }
            processReceivedData(buffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    else {
        Debug::log(Debug::Severity::Error, "SonarApp", "Failed to open UART port.");
    }
}

void SonarApp::processReceivedData(const std::vector<uint8_t>& data)
{
    // 在这里添加处理接收到的数据的逻辑
    std::string receivedData(data.begin(), data.end());
    std::cout << "Received Data: " << receivedData << std::endl;
}
void SonarApp::sendFormattedData
(
    const std::string& portName,
    uint32_t baudrate,
    uint16_t year, uint8_t month, uint8_t day,
    uint8_t hour, uint8_t minute, uint8_t second,
    uint8_t status, uint16_t angle, uint8_t speed,
    uint8_t minrange,
    uint8_t maxrange
)
{
    
    memset(&msg, 0, sizeof(msg));
    std::memcpy(msg.header, "$SMSNXX", 7);
    msg.length = 28;
    msg.year = year;
    msg.month = month;
    msg.day = day;
    msg.hour = hour;
    msg.minute = minute;
    msg.second = second;
    msg.status = status;
    msg.angle = angle;
    msg.speed = speed;
    msg.minrange = minrange;
    msg.maxrange = maxrange;
    msg.reserved = 0;
    msg.checksum = addcalculateChecksum(reinterpret_cast<uint8_t*>(&msg), sizeof(msg) - 4);
    msg.end1 = 0x0D;
    msg.end2 = 0x0A;
    std::string messageStr = messageToString(msg);
    memcpy(sendBuffer, messageStr.c_str(), 28);
    memset(&msg, 0, sizeof(msg));
}


