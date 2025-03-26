//------------------------------------------ Includes ----------------------------------------------

#include "sonarApp.h"
#include "maths/maths.h"
#include "platform/debug.h"
#include "files/bmpFile.h"
#include "utils/utils.h"
#include <iostream> // ����ͷ�ļ�
#include <ctime> // For std::time_t, std::time, std::ctime
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <direct.h> 
#include <filesystem> // ʹ�� std::filesystem ������ _mkdir
#include "platform/windows/uart.h" // ȷ����������ȷ��ͷ�ļ�
#include <vector>
#include <string>
#include "devices/device.h"
#include "global.h" // ���� global.h
#include <thread> // �����߳̿�
#include <windows.h> // ���� Windows API ͷ�ļ�
#include <mutex>
#include <opencv2/opencv.hpp>
#include "data_logger.h"
#include "array_data.h"
//#include "global.h"
// ���������һ����־����ʾ�Ƿ�ʼ�� ping data �ļ�¼
bool isRecording = false;
std::string dataFolder_;  // �ļ���·��
using namespace IslSdk;
unsigned char mesbag[28] = { 0 };
std::mutex uartMutex;
// 90��������߽ǽǶȶ���/����1����764�и����޸�
//const float START_ANGLE = 315; // ���ο�ʼ�߽�
//const float END_ANGLE = 45.0;    // ���ν����߽�
//const float ANGLE_TOLERANCE = 0.5; // �ݲΧ ��1��
//const int ROWS = 160; // ����
//const int COLS = 101;  // ����
 
////180��������߽ǽǶȶ���
const float START_ANGLE = 270; // ���ο�ʼ�߽�
const float END_ANGLE = 90;    // ���ν����߽�
const float ANGLE_TOLERANCE = 0.5; // �ݲΧ ��1��
const int ROWS = 160; // ����
const int COLS = 201;  // ����

float shanxing[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float frame1[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float frame2[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float frame3[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float frame4[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float* currentFrame = &frame1[0][0]; // ָ�� frame1 ���׵�ַ
float* nextFrame = &frame2[0][0];   // ָ�� frame2 ���׵�ַ

bool isCollecting = false; // ���ݲɼ���־
// ����ȫ�ֱ���
Message  msg;
extern SerialPort serialPort; // ����ȫ�ֱ���
extern SerialPort serialPort2; // ����ȫ�ֱ���
int bytesWritten12;
int bytesWritten21;

////ʵ��վ
//char writeBufferimage[] = "Image detection ready!\r\n";
//char writeBufferjinggao[] = "$HXXB,WAR,1*CK\r\n";
//char writeBufferxiaoshi[] = "$HXXB,OFF,1*CK\r\n";
char writeBufferjinggao2[] = "$HXXB,WAR,2*CK\r\n";
char writeBufferxiaoshi2[] = "$HXXB,OFF,2*CK\r\n";
//char writeBufferjingzhi[] = "It may be a stationary target\r\n";

//����ʵ��վ
char writeBufferimage[] = "Image detection ready!\r\n";
char writeBufferjinggao[] = "$HXXB,WAR,1*CK\r\n";
char writeBufferxiaoshi[] = "$HXXB,OFF,1*CK\r\n";
char writeBufferjingzhi[] = "It may be a stationary target\r\n";

int first_mubao = 0;
int sec_mubao = 0;
int chazhi_mubiao = 0;
int biaozhi = 0;

std::string IslSdk::messageToString(const Message& msg)
{
    std::ostringstream oss;

    // ��headerת��Ϊ�ַ���
    oss.write(msg.header, sizeof(msg.header));

    // �������ֶΰ�˳����ӵ��ַ�������
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

SerialPort::SerialPort() 
{
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
        std::cout << "�޷��򿪴���!" << std::endl;
        return false;
    }

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hComm, &dcbSerialParams)) {
        std::cout << "��ȡ����״̬ʧ��!" << std::endl;
        close();
        return false;
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hComm, &dcbSerialParams)) {
        std::cout << "���ô���״̬ʧ��!" << std::endl;
        close();
        return false;
    }

    timeouts.ReadIntervalTimeout = 5;
    timeouts.ReadTotalTimeoutConstant = 5;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 5;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    if (!SetCommTimeouts(hComm, &timeouts)) {
        std::cout << "���ô��ڳ�ʱʧ��!" << std::endl;
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

        // �������캯��
        RangeImageBeam();
    };

    // ���幹�캯��
    RangeImageBeam::RangeImageBeam() : angle_du(0.0f), angle_stepSize(0.0f), minRange(0.0f), max_range(0.0f), data_count(0) {
        // ��������������ĳ�ʼ������
    }

} // namespace msis_pkg
//--------------------------------------------------------------------------------------------------
SonarApp::SonarApp(void) : App("SonarApp"), m_pingCount(0), m_scanning(false), sequenceNumber_(1)
{
   // setDataFolder("G:\\508 Project\\20240813\\two");
    setDataFolder("D:\\ceshi"); 
    header = "$SMSNXX";
    length = 28;  // ������Ϣ���ȣ��ų�������
    initializeTime();
    status = 0x0F;  // ʾ��״̬
    angle = 90;     // ʾ���Ƕ�
    speed = 2;      // ʾ���ٶ�
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
    //�Զ�����
    std::thread([this]() {
        while(true){
        std::this_thread::sleep_for(std::chrono::seconds(2)); // �ȴ�1�룬ȷ���豸�ѳ�ʼ��
        this->doTask('r', dataFolder_); // ��ʼɨ��
        //std::this_thread::sleep_for(std::chrono::minutes(4)); // �ȴ�4����
        //this->doTask('R', dataFolder_); // ֹͣɨ��
        //std::this_thread::sleep_for(std::chrono::minutes(1)); // �ȴ�4����
        }
        }).detach();
    //std::this_thread::sleep_for(std::chrono::seconds(1)); // �ȴ�1�룬ȷ���豸�ѳ�ʼ��
    //doTask('r', dataFolder_); // ��ʼɨ��

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
        //SerialPort serialPort;
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
                // �ڼ�¼��ʼʱ����ʱ���
                auto now = std::chrono::system_clock::now();
                std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                // �ڿ�ʼ��¼ʱ����һ���µ��ļ���
                std::stringstream filenameStream;
                filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                std::string filename = filenameStream.str();
                // ���ļ���д��ʱ����������Ϣ
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
                // �ڼ�¼����ʱ����ʱ���
                auto now = std::chrono::system_clock::now();
                std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                // �ڽ�����¼ʱ����һ���µ��ļ���
                std::stringstream filenameStream;
                filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                std::string filename = filenameStream.str();
                // ���ļ���д��ʱ����������Ϣ
                std::ofstream outputFile(filename, std::ios::app);  // ʹ��׷��ģʽ���ļ�
                if (outputFile.is_open()) {
                    // �ڽ�����¼ʱ���������Ϣ
                    outputFile << "Recording Ended:" << std::endl;
                    outputFile << "Timestamp: " << std::ctime(&timestamp);
                    outputFile << "Sequence Number: " << sequenceNumber_ << std::endl;
                    outputFile.close();
                    std::cout << "Recording ended. Data written to " << filename << std::endl;
                }
                else {
                    std::cerr << "Unable to open the file for writing." << std::endl;
                }
                // �������
                ++sequenceNumber_;
            }
            break;

        case 'p':
            renderPalette(path);
            break;

        case 'i':
            m_circular.render(sonarDataStore, m_palette, true);//(��Ⱦ)
            BmpFile::save(path + "texture.bmp", reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), 32, m_circular.width, m_circular.height);
            break;

        case 't':
            m_texture.renderTexture(sonarDataStore, m_palette, false);
            BmpFile::save(path + "sonar.bmp", reinterpret_cast<const uint32_t*>(&m_texture.buf[0]), 32, m_texture.width, m_texture.height);
            break;

        case 'q':
            //std::string portname = "COM4"; // ������Ҫ�����˿ں�
            //uint32_t baudrate = 115200; // ������Ҫ����������

            //sendUartData("COM6", 115200, data1);
            
            //serialPort22.open("COM2", 115200);

           // serialPort2.write(writeBufferjinggao, 16, bytesWritten12);

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
        static int pingDataCount = 0; // ��̬�������ڱ��ּ�������ֵ
        //Debug::log(Debug::Severity::Info, name.c_str(), "Ping data");
        Debug::log(Debug::Severity::Info, name.c_str(), "Ping data, Count: %d", ++pingDataCount); // ��ʾ��������ֵ
        uint_t txPulseLengthMm = static_cast<uint_t>(iss360.settings.setup.speedOfSound * iss360.settings.acoustic.txPulseWidthUs * 0.001 * 0.5);
        txPulseLengthMm = Math::max<uint_t>(txPulseLengthMm, 150);

        sonarDataStore.add(ping, txPulseLengthMm);
        // ��¼���ݵ��ļ�
        recordPingData(iss360, ping, txPulseLengthMm);
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
        
        if (flag_shanxing == 1)
        {
            jishu_shanxing = jishu_shanxing + 1;
            if (jishu_shanxing == 1)
            {
                memcpy(frame4, shanxing, sizeof(shanxing));
            }
            jishi_mubiao++;
            //����������ת��Ϊͼ������
            //frame1 = frame2;
            //std::swap(currentFrame, nextFrame);
            //if (flag_yundong == 0 && flag_jingzhi == 1)//��Ŀ��״̬�µļ��
            //{
            //    memcpy(frame1, frame3, sizeof(frame1));
            //    memcpy(frame2, shanxing, sizeof(shanxing));

            /*}*/
            //if (flag_yundong == 1 && flag_jingzhi == 0)//��Ŀ��״̬�µļ��
            //{
                memcpy(frame1, frame2, sizeof(frame1));
                memcpy(frame2, shanxing, sizeof(shanxing));
    /*        }*/

            if (jishu_shanxing > 1000)
            {
                jishu_shanxing=1;
                jishi_mubiao = 1;
            }
             //frame2 = shanxing;
            flag_shanxing = 0;
            // ����ά����ת��Ϊ cv::Mat
            cv::Mat frame11 = cv::Mat(ROWS, COLS, CV_32FC1, frame1).clone(); // ��¡��֤���ݶ���
            cv::Mat frame22 = cv::Mat(ROWS, COLS, CV_32FC1, frame2).clone();

            // ��һ���� 0~255 ������ʾ
            cv::Mat frame11Display, frame22Display;
            cv::normalize(frame11, frame11Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(frame22, frame22Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            // ���ű���
            double scale = 2.0;
            // ���ͼ���Ƿ�Ϊ��
            if (frame22Display.empty()) 
            {
                std::cerr << "Error: frame22Display is empty!" << std::endl;
                return;
            }

            // �������ͨ����ɫͼ��ת��Ϊ�Ҷ�ͼ��
            if (frame22Display.channels() == 3) {
                cv::cvtColor(frame22Display, frame22Display, cv::COLOR_BGR2GRAY);
            }
            //cv::imshow("Frame22", frame22Display);
            //cv::waitKey(0); // ���ִ��ڴ�
            // ����ͼ���ļ�

            // �Ŵ�ͼ��
            cv::Mat frame11Resized, frame22Resized;
            // cv::resize(frame11Display, frame11Resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
            cv::resize(frame22Display, frame22Resized, cv::Size(), scale, scale, cv::INTER_NEAREST);// ��ʾͼ��
            // cv::imshow("Frame11", frame11Display);
           // std::cout << "frame22 data: " << frame22Display << std::endl;
            bool result = cv::imwrite("D:/ceshi/frame22_image.png", frame22Resized);
            if (result) {
                std::cout << "ͼ���ѱ��浽�ļ���!" << std::endl;
            }
            else {
                std::cerr << "����ͼ��ʧ��!" << std::endl;
            }
            //cv::Mat color_frame11, color_frame22;
            // cv::applyColorMap(frame11Display, color_frame11, cv::COLORMAP_JET); // ʹ��α��ɫ
            //cv::applyColorMap(frame22Display, color_frame22, cv::COLORMAP_JET); // ʹ��α��ɫ
            //cv::imshow("Frame22", color_frame22);
            //saveShanxingToFile(&frame22Display, ROWS, COLS, "frame22Display_data.csv");
            // }
          
            // 
            //ͼ����
            if (jishu_shanxing > 1)
            {
                int no_target = 0;  // ��ʼ����Ŀ���־
                int no_target1 = 0;  // ��ʼ����Ŀ���־
                Centroid centroid;  // ��ʼ�����Ľṹ��

                if(jishu_shanxing==2|| jishu_shanxing == 3||jishu_shanxing == 4)
                {
                serialPort.write(writeBufferimage, 24, bytesWritten21);//���������б�
                saveData("D:/ceshi/Seriallog.txt", writeBufferimage, strlen(writeBufferimage), "COM1 Send", 0);
                }
                // ���� process_frame_difference ����
                int result = process_frame_difference(frame11Display, frame22Display, no_target, centroid);//���ݴ���
                flag_target = no_target;
                if (jishu_mubiao == 1)
                {
                    
                    cv::Mat frame44 = cv::Mat(ROWS, COLS, CV_32FC1, frame4).clone();
                    // ��һ���� 0~255 ������ʾ
                    cv::Mat frame44Display;
                    cv::normalize(frame44, frame44Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                    int result2 = process_frame_difference(frame44Display, frame22Display, no_target1, centroid);//���ݴ���
                    flag_beijing = no_target1;
                    //saveData("D:/ceshi/Seriallog.txt", writeBufferimage, strlen(writeBufferimage), "COM1 1111", 0);
                    if (flag_target == 0 && flag_beijing == 1)
                    {
                        flag_target = 1;
                        //saveData("D:/ceshi/Seriallog.txt", writeBufferimage, strlen(writeBufferimage), "COM1 2222", 0);
                    }//����֮����Կ��ǶԱ�����ֵ���п���
                    //if (flag_target == 1 && flag_beijing == 0)
                    //{
                    //    flag_target = 0;
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferimage, strlen(writeBufferimage), "COM1 2222", 0);
                    //}
                }
                if(flag_zhiling ==0)
                { 
                  
                    if (biaozhi != 1)
                    {      //  memcpy(frame3, frame2, sizeof(frame3));
                        cv::Mat frame44 = cv::Mat(ROWS, COLS, CV_32FC1, frame4).clone();

                        // ��һ���� 0~255 ������ʾ
                        cv::Mat frame44Display;
                        cv::normalize(frame44, frame44Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                        int result2 = process_frame_difference(frame44Display, frame22Display, no_target1, centroid);//���ݴ���
                        flag_beijing = no_target1;
                    }
                    if (flag_target == 0 && flag_beijing == 1)
                    {
                        flag_target = 1;
                        jishu_guding++;
                        if (jishu_guding > 15)
                        {
                            //ʵ��վ
                            //if (globalPn == 2255 && globalSn == 10)
                            //{
                            //    //serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                            //    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send jingzhi2", 0);
                            //    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                            //    //serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi2", 0);
                            //}
                            //else if (globalPn == 2254 && globalSn == 25)
                            //{
                            //    // serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                            //    // serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send jingzhi1", 0);
                            //    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                            //   // serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi1", 0);
                            //}
                            ////����ʵ��վ˫����
                            //if (globalPn == 2255 && globalSn == 10)
                            //{
                            //    serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                            //    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send jingzhi2", 0);
                            //    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                            //    //serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi2", 0);
                            //}
                            //else if (globalPn == 2254 && globalSn == 25)
                            //{
                            //     serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                            //    // serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send jingzhi1", 0);
                            //    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                            //   // serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi1", 0);
                            //}
                            //����ʵ��վ������
                            serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                            // serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                            saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send jingzhi", 0);
                            //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                           // serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                            saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi", 0);


                            memcpy(frame4, frame2, sizeof(frame4));
                            flag_jingzhi = 0;
                            flag_yundong = 1;
                            jishu_guding = 0;
                            flag_zhiling = 1;
                            flag_target = 0;
                            biaozhi = 0;
                        }
                    }
                 }
                
                


           
            }
            if (flag_target == 1)

            {
                biaozhi = 1;
                jishu_mubiao++;
                if (jishu_mubiao == 1)
                {
                    first_mubao = jishi_mubiao;
                }
                if (jishu_mubiao == 2)
                {
                    sec_mubao = jishi_mubiao;
                    chazhi_mubiao = sec_mubao - first_mubao;

                    //��������
                    if (chazhi_mubiao<4)
                    {
                        jishu_mubiao = 0;
                        jishi_mubiao = 0;
                        if (flag_target == 1 && flag_zhiling == 1)
                        {

                            //serialPort2.open("COM2", 115200);

                            // int bytesWritten1;
                            // ʵ��վ
                            //if (globalPn == 2255 && globalSn == 10)
                            //{
                            //    //serialPort.write(writeBufferjinggao2, 16, bytesWritten21);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao2), "COM1 Send jinagao2", 0);
                            //    serialPort2.write(writeBufferjinggao2, 16, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao2), "COM2 Send jinagao2", 0);
                            //}
                            //else if (globalPn == 2254 && globalSn == 25)
                            //{
                            //    //serialPort.write(writeBufferjinggao, 16, bytesWritten21);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM1 Send jinagao1", 0);
                            //    serialPort2.write(writeBufferjinggao, 16, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM2 Send jinagao1", 0);
                            //}
                            //char writeBuffer[] = "$HXXB,WAR,1*CK\r\n";
                            ////����ʵ��վ˫����
                            //if (globalPn == 2255 && globalSn == 10)
                            //{
                            //    serialPort.write(writeBufferjinggao, 16, bytesWritten21);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao2), "COM1 Send jinagao2", 0);
                            //    //serialPort2.write(writeBufferjinggao2, 16, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao2), "COM2 Send jinagao2", 0);
                            //}
                            //else if (globalPn == 2254 && globalSn == 25)
                            //{
                            //    serialPort.write(writeBufferjinggao, 16, bytesWritten21);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM1 Send jinagao1", 0);
                            //    //serialPort2.write(writeBufferjinggao, 16, bytesWritten12);
                            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM2 Send jinagao1", 0);
                            //}
                            //����ʵ��վ
                            serialPort.write(writeBufferjinggao, 16, bytesWritten21);
                            saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao2), "COM1 Send jinagao", 0);
                            //serialPort2.write(writeBufferjinggao2, 16, bytesWritten12);
                            saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao2), "COM2 Send jinagao", 0);

                                memcpy(frame3, frame1, sizeof(frame1));
                          
                            flag_zhiling = 0;
                            flag_jingzhi = 1;
                            flag_yundong = 0;

                        }
                    }
                }
            }
            if (flag_target == 0 && flag_zhiling == 0)
            {
                //SerialPort serialPort;
                //serialPort22.open("COM2", 115200);
                //int bytesWritten1;
               // char writeBuffer[] = "$HXXB,OFF,1*CK\r\n";
                //ʵ��վ
                //if (globalPn == 2255 && globalSn == 10)
                //{
                //    //serialPort.write(writeBufferxiaoshi2, 16, bytesWritten21);
                //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM1 Send xiaoshi2", 0);
                //    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
                //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send xiaoshi2", 0);
                //}
                //else if (globalPn == 2254 && globalSn == 25)
                //{
                //    //serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM1 Send xiaoshi1", 0);
                //    //serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send xiaoshi1", 0);
                //}
                
                //����ʵ��վ
                //if (globalPn == 2255 && globalSn == 10)
                //{
                //    serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM1 Send xiaoshi2", 0);
                //    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
                //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send xiaoshi2", 0);
                //}
                //else if (globalPn == 2254 && globalSn == 25)
                //{
                //    serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM1 Send xiaoshi1", 0);
                //    //serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send xiaoshi1", 0);
                //}
                serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM1 Send xiaoshi", 0);
                //serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send xiaoshi", 0);

                biaozhi = 0;
                flag_zhiling = 1;
                flag_jingzhi=0;
                flag_yundong = 1;
            }




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

//�����




void SonarApp::recordPingData(const Sonar& iss360, const Sonar::Ping& ping,uint_t txPulseLengthMm)
{
    //���ж��Ƿ�ɼ�shanxing����


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
    // ���㵱ǰ�Ƕ�
    float current_angle = float(ping.angle) * 360 / 12800;
    // �ж��Ƿ�ӽ���ʼ�߽� (315��/270��)
    if (!isCollecting && std::abs(current_angle - START_ANGLE) <= ANGLE_TOLERANCE)
    {
        std::cout << "Starting data collection at angle: " << current_angle << " degrees." << std::endl;
        isCollecting = true;
        
    }
   
       //////  �жϵ�ǰ�Ƕ��Ƿ��ںϷ���Χ (315�� ~ 45��)
       // if (current_angle < 315.0 && current_angle>45)
       //{
       //     //isValidAngle = false;
       //     isCollecting = false;
       //     std::memset(shanxing, 0, sizeof(shanxing));
       //     currentCol = 0; // �������� currentCol = 0; ����������
       //}
         /*�жϵ�ǰ�Ƕ��Ƿ��ںϷ���Χ (270�� ~ 90��)*/
        if (current_angle < 270.0 && current_angle>90)
        {
            //isValidAngle = false;
            isCollecting = false;
            std::memset(shanxing, 0, sizeof(shanxing));
            currentCol = 0; // �������� currentCol = 0; ����������
        }

    if (isCollecting)
    { 
    //if (currentCol >= COLS)
    //{
    //    std::cerr << "Array full, cannot store more data." << std::endl;
    //    currentCol = 0; // �������� currentCol = 0; ����������
    //    std::memset(shanxing, 0, sizeof(shanxing));
    //    std::cerr << "Array clear all data." << std::endl;
    //}
    // �� ping.data �洢�� shanxing ����ĵ�ǰ��
    for (int i = 0; i < ping.data.size() && i < ROWS; ++i) {
        shanxing[i][currentCol] = static_cast<float>(ping.data[i]);
    }
    // ���µ�ǰ������
    currentCol++;
    if(currentCol==COLS)
    {
        //jushu_shanxing++;
        flag_shanxing = 1;
        //std::cout << "Displaying shanxing matrix:" << std::endl;

        //for (int i = 0; i < ROWS; ++i) {
        //    for (int j = 0; j < COLS; ++j) {
        //        // ���ù̶���ȱ��ڹ۲����ṹ
        //        std::cout << std::setw(5) << shanxing[i][j] << " ";
        //    }
        //    std::cout << std::endl; // ����
        //}

        //std::cout << "End of matrix display." << std::endl;
//        // �洢���ļ�
//
//
////float shanxing[ROWS][COLS] = {0}; // ��������������
//        saveShanxingToFile(&shanxing[0][0], ROWS, COLS, "shanxing_data.csv");
    }
    }
    // ��ȡ��ǰʱ���
    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&timestamp);
    uint16_t year = now_tm->tm_year + 1900; // ��ȡ���������
    uint8_t month = now_tm->tm_mon + 1;
    uint8_t day = now_tm->tm_mday;
    uint8_t hour = now_tm->tm_hour;
    uint8_t minute = now_tm->tm_min;
    uint8_t second = now_tm->tm_sec;

    uint8_t status=0x00 ; // ʾ��״̬
    // ����״̬λ��Bit0, Bit1, Bit2, Bit3
    // ʹ��ȫ�ֱ��������ж�
    //std::string deviceInfo = std::to_string(globalPn) + "." + std::to_string(globalSn);
    //if (globalPn == 2254 && globalSn == 25)
    //{
    //    status |= 0x01; // ���� Bit0 Ϊ 1
    //}
    //if (globalPn == 2254 && globalSn == 23)
    //{
    //    status |= 0x01; // ���� Bit0 Ϊ 1
    //}
    //if (globalPn == 2255 && globalSn == 10)
    //{
    //    status |= 0x01; // ���� Bit0 Ϊ 1
    //}
    // ���� angle �� speed
    if (biaozhi == 1)
    {
        status = 0x04; //��Ŀ��
        status|=globalstatus;
    }
    else
    {
                status |= globalstatus;//��Ŀ��
    }
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
    // ������ݴ����߼�
    //processData(ping.data);
    //// �������ݴ��������з���
    //bool targetDetected = checkForTarget(temp_ping_.beam_data);
    //if (targetDetected) {
    //    status |= 0x04; // ���� Bit2 Ϊ 1 ��ʾ��Ŀ��
    //}
    // ʾ��Ŀ���⺯��
    //bool SonarApp::checkForTarget(const std::vector<uint16_t>&beam_data)
    //{
    //    // ���������Ŀ�����߼�
    //    // ���� true ��ʾ��⵽Ŀ�꣬���򷵻� false
    //    return std::any_of(beam_data.begin(), beam_data.end(), [](uint16_t value) {
    //        return value > 100; // ʾ��������ֵ���� 100 ��ʾ��Ŀ��
    //        });
    //}
    globalcount = temp_ping_.data_count;
    //globalx = 1;
    uint8_t minrange = temp_ping_.minRange;
    uint8_t maxrange = temp_ping_.max_range;



    std::stringstream filenameStream;
    // �����ļ�����ʹ��ʱ�����Ϊ�ļ�����һ����

    filenameStream << dataFolder_ << "/all_ping_data1.txt";
    std::string filename = filenameStream.str();

    // ������д���ļ�
    std::ofstream m_outputFile(allPingDataFilename_, std::ios::app);  // ע������ʹ�� std::ios::app ����׷��д��
    if (m_outputFile.is_open()) 
    {
        // ��¼���ݵ��ļ�
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

    // ���͸�ʽ������
    std::string portName = "COM6"; // ������Ҫ����
    uint32_t baudrate = 115200; // ������Ҫ����
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
    // �����ļ�����ʹ��ʱ�����Ϊ�ļ�����һ����
    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

    std::stringstream filenameStream;
    filenameStream << dataFolder_ << "/all_ping_data_" << std::to_string(timestamp) << ".txt";
    allPingDataFilename_ = filenameStream.str();
    // ����ǰʱ��д���ļ�
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
    // ��ȡ����ӡ��ǰ���õĴ����б�
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
    // ������������ݴ����߼�
    // ���磬�������ݵ�ƽ��ֵ����Ŀ��
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
    // ��������Ӵ�����յ������ݵ��߼�
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
void SonarApp::saveShanxingToFile(const float* shanxing, int rows, int cols, const std::string& filename)
{
    // ���ļ�
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // ��������д���ļ�
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            file << std::setw(10) << *(shanxing + i * cols + j); // ���ÿ�ȶ��룬���ڹ۲�
            if (j < cols - 1) {
                file << ","; // ��֮��ӿո�ָ�
            }
        }
        file << "\n"; // ÿ�н�������
    }

    file.close(); // �ر��ļ�
    std::cout << "Shanxing data saved to " << filename << std::endl;
}
void SonarApp::saveMatToCSV(const cv::Mat& mat, const std::string& filename)
{
    // ������ļ���
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << " for writing." << std::endl;
        return;
    }

    // ����ÿһ�к�ÿһ�У�������д��CSV
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            file << static_cast<int>(mat.at<uchar>(i, j));  // ������CV_8UC1��ȷ������Ϊuchar����
            if (j < mat.cols - 1) {
                file << ",";  // ��Ӷ��ŷָ���
            }
        }
        file << "\n";  // ����
    }

    file.close();
    std::cout << "Matrix saved to " << filename << std::endl;
}
