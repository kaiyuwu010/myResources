#include <dlfcn.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

#include "../../dependency/gripper/include/ICANCmd.h"

// 全局变量
void* handleLib;

// 连接CAN设备
unsigned int connect()
{
    unsigned int handle = CAN_DeviceOpen(14, 0, "");
    std::cout << "handle of device: " << handle << std::endl;

    CAN_DeviceInformation devInfo;
    std::memset(&devInfo, 0, sizeof(devInfo));

    std::cout << "get device information: " << CAN_GetDeviceInfo(handle, &devInfo) << std::endl;
    std::cout << "device FirmWareVersion: " << devInfo.uFirmWareVersion << std::endl;
    std::cout << "device ChannelNumber: " << (int)devInfo.bChannelNumber << std::endl;

    CAN_InitConfig initConfig;
    std::memset(&initConfig, 0, sizeof(initConfig));
    initConfig.bMode = 0;
    initConfig.nBtrType = 1;
    initConfig.dwBtr[0] = 0x00;
    initConfig.dwBtr[1] = 0x00000014;
    initConfig.dwAccCode = 0xffffffff;
    initConfig.dwAccMask = 0xffffffff;
    initConfig.nFilter = 0;
    initConfig.dwReserved = 0;
    std::cout << "channel start: " << CAN_ChannelStart(handle, 0, &initConfig) << std::endl;
    return handle;
}

// 获取错误信息
unsigned int Get_ErrorInformation(unsigned int handle, int channel)
{
    CAN_ErrorInformation errInfo;
    std::memset(&errInfo, 0, sizeof(errInfo));

    if (CAN_GetErrorInfo(handle, channel, &errInfo) == 1)
    {
        std::cout << "ErrorCode!: " << errInfo.uErrorCode << std::endl;
    }

    return errInfo.uErrorCode;
}

// 发送数据
unsigned int Send_Data(unsigned int handle, int channel, unsigned int ID, unsigned char length, unsigned char List_SendData[])
{
    CAN_DataFrame sendData;
    std::memset(&sendData, 0, sizeof(sendData));
    sendData.uTimeFlag = 0;
    sendData.nSendType = 0;
    sendData.bRemoteFlag = 0;
    sendData.bExternFlag = 0;
    sendData.nDataLen = length;
    sendData.uID = ID;
    std::cout << "test0 " << int(length) << std::endl;

    for (int i = 0; i < int(length); i++)
    {
        sendData.arryData[i] = List_SendData[i];
    }
    std::cout << "test1" << std::endl;

    unsigned int result = CAN_ChannelSend(handle, channel, &sendData, 1);
    std::cout << "Send data: ";
    for (int i = 0; i < length; i++)
    {
        std::cout << (int)List_SendData[i] << " ";
    }
    std::cout << "Result: " << result << std::endl;

    Get_ErrorInformation(handle, channel);
    return result;
}

// 获取缓冲区接收但尚未被读取的帧数
unsigned int GetReceiveCount(unsigned int handle, int channel)
{
    unsigned int num = CAN_GetReceiveCount(handle, channel);
    std::cout << "Number of buffer: " << num << std::endl;
    return num;
}

// 接收数据
std::vector<int> Receive_Data(unsigned int handle, int channel, unsigned int WaitTime)
{
    CAN_DataFrame receiveData;
    std::memset(&receiveData, 0, sizeof(receiveData));

    receiveData.uTimeFlag = 1;
    receiveData.nSendType = 0;
    receiveData.bRemoteFlag = 0;
    receiveData.bExternFlag = 0;
    receiveData.nDataLen = 8;
    receiveData.uID = 0;

    GetReceiveCount(handle, channel);

    unsigned int result = CAN_ChannelReceive(handle, channel, &receiveData, 1, WaitTime);
    std::vector<int> List_ReceiveData;
    List_ReceiveData.push_back(receiveData.uID);
    List_ReceiveData.push_back(receiveData.nDataLen);
    for (int i = 0; i < receiveData.nDataLen; i++)
    {
        List_ReceiveData.push_back(receiveData.arryData[i]);
    }

    std::cout << "RECEIVE DATA: ID: " << List_ReceiveData[0] << " Length: " << List_ReceiveData[1] << " DATA: ";
    for (int i = 2; i < List_ReceiveData.size(); i++)
    {
        std::cout << List_ReceiveData[i] << " ";
    }
    std::cout << "Result: " << result << std::endl;

    Get_ErrorInformation(handle, channel);
    return List_ReceiveData;
}

// 关闭设备
void Close_Device(unsigned int handle, int channel)
{
    std::cout << "stop channel: " << CAN_ChannelStop(handle, channel) << std::endl;
    std::cout << "close device: " << CAN_DeviceClose(handle) << std::endl;
}

// 将数据列表转换为十六进制字符串表示并打印
void print_hex_string(const std::vector<uint8_t>& data)
{
    for (size_t i = 0; i < data.size(); ++i)
    {
        printf("%02x", data[i]);
        if (i < data.size() - 1)
        {
            printf(" ");
        }
    }
    printf("\n");
}

// 置零位
std::vector<uint8_t> Gripper_Sethome()
{
    std::vector<uint8_t> List_Data = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    print_hex_string(List_Data);
    return List_Data;
}

// 位置模式（位置数值范围在0~36000，速度范围0~65535）
uint8_t* Gripper_Pos(uint32_t position, uint16_t maxSpeed)
{
    uint8_t* List_Data = new uint8_t[8];
    List_Data[0] = 0xA4;
    List_Data[1] = 0x00;
    List_Data[2] = static_cast<uint8_t>(maxSpeed & 0xFF);
    List_Data[3] = static_cast<uint8_t>((maxSpeed >> 8) & 0xFF);
    List_Data[4] = static_cast<uint8_t>(position & 0xFF);
    List_Data[5] = static_cast<uint8_t>((position >> 8) & 0xFF);
    List_Data[6] = static_cast<uint8_t>((position >> 16) & 0xFF);
    List_Data[7] = static_cast<uint8_t>((position >> 24) & 0xFF);
    return List_Data;
}

// 力控模式（数值范围在-2000~2000，对应的相电流-32A~32A）
uint8_t* Gripper_Torque(int16_t torque)
{
    uint8_t* List_Data = new uint8_t[8];
    List_Data[0] = 0xA1;
    List_Data[1] = 0x00;
    List_Data[2] = 0x00;
    List_Data[3] = 0x00;
    List_Data[4] = static_cast<uint8_t>(torque & 0xFF);
    List_Data[5] = static_cast<uint8_t>((torque >> 8) & 0xFF);
    List_Data[6] = 0x00;
    List_Data[7] = 0x00;
    return List_Data;
}

// 清除错误
std::vector<uint8_t> Gripper_ClearError()
{
    std::vector<uint8_t> List_Data = {0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    print_hex_string(List_Data);
    return List_Data;
}
