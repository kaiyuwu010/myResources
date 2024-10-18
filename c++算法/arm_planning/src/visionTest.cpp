#include <chrono>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>

#include "../dependency/rm-arm/include/rm_base.h"
#include "../include/arm_planning/getArmUdpData.hpp"
#include "../include/arm_planning/gripperControl.hpp"
#include "../include/arm_planning/hybridPositionForceControl.hpp"
#include "../include/arm_planning/trajectoryPlanning.h"

#define PI 3.141592

int main()
{
    // 连接
    unsigned int handle = connect();
    // CAN默认通道选择
    int channel = 0;
    // ID=0x140 + 电机ID（1~32）
    int ID = 0x141;
    uint8_t* torque_control_data;
    uint8_t* position_control_data;
    torque_control_data = Gripper_Torque(0);       // 力控模式（数值范围在-2000~2000，对应的相电流-32A~32A）正数是打开夹爪，负数是关闭夹爪
    position_control_data = Gripper_Pos(0, 1000);  // 位置模式（位置数值范围在0~36000，速度范围0~65535）  10000对应58mm 20000对应94mm 25000对应最大106mm

    // 位置控制
    // Send_Data(handle, channel, ID, 8, position_control_data);
    sleep(1);
    // 力控
    Send_Data(handle, channel, ID, 8, torque_control_data);

    // 释放内存
    delete[] torque_control_data;
    delete[] position_control_data;
    // 关闭设备
    Close_Device(handle, 0);

    return 0;
}
