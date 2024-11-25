#ifndef _GRIPPER_PROTOCOL_H
#define _GRIPPER_PROTOCOL_H
#include <iostream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <cstdint>

class GripperProtocol {
public:
    // 构造函数，初始化 Modbus 地址
    GripperProtocol(uint8_t device_address) : address(device_address) {}

    // 生成 CRC 校验码
    static uint16_t calculateCRC(const std::vector<uint8_t>& data) {
        uint16_t crc = 0xFFFF;
        for (uint8_t byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; i++) {
                if (crc & 0x0001) {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    // 构造 Modbus 指令
    std::vector<uint8_t> buildCommand(uint16_t register_address, uint16_t value) {
        std::vector<uint8_t> command = {
            address,        // 设备地址
            0x10,           // 功能码 (写单个寄存器)
            uint8_t(register_address >> 8),   // 寄存器地址高位
            uint8_t(register_address & 0xFF), // 寄存器地址低位
            0x00, 0x01,     // 寄存器数量（固定为1）
            0x02,           // 数据字节数
            uint8_t(value >> 8),   // 数据高位
            uint8_t(value & 0xFF)  // 数据低位
        };
        uint16_t crc = calculateCRC(command);
        command.push_back(crc & 0xFF);        // CRC 低位
        command.push_back((crc >> 8) & 0xFF); // CRC 高位
        return command;
    }

    // 解析返回数据
    bool parseResponse(const std::vector<uint8_t>& response) {
        if (response.size() < 6) return false;
        // 校验CRC
        uint16_t received_crc = response[response.size() - 2] | (response[response.size() - 1] << 8);
        std::vector<uint8_t> data_without_crc(response.begin(), response.end() - 2);
        uint16_t calculated_crc = calculateCRC(data_without_crc);
        return received_crc == calculated_crc;
    }

    // 设置夹爪幅度
    std::vector<uint8_t> setGripperPosition(uint16_t position) {
        return buildCommand(0x9C40, position);
    }

    // 设置夹爪力度
    std::vector<uint8_t> setGripperForce(uint16_t force) {
        return buildCommand(0x9C41, force);
    }

    // 读取夹爪当前位置
    std::vector<uint8_t> getGripperPosition() {
        std::vector<uint8_t> command = {
            address, 0x03,  // 功能码 (读寄存器)
            0x9C, 0x45,    // 寄存器地址
            0x00, 0x01     // 寄存器数量
        };
        uint16_t crc = calculateCRC(command);
        command.push_back(crc & 0xFF);
        command.push_back((crc >> 8) & 0xFF);
        return command;
    }

private:
    uint8_t address; // 设备地址
};

#endif
// // 测试代码
// int main() {
//     GripperProtocol gripper(0x01);

//     // 设置夹爪幅度
//     auto position_command = gripper.setGripperPosition(100);
//     std::cout << "Set Position Command: ";
//     for (auto byte : position_command) {
//         std::cout << std::hex << std::setw(2) << std::setfill('0') << int(byte) << " ";
//     }
//     std::cout << std::endl;

//     // 设置夹爪力度
//     auto force_command = gripper.setGripperForce(50);
//     std::cout << "Set Force Command: ";
//     for (auto byte : force_command) {
//         std::cout << std::hex << std::setw(2) << std::setfill('0') << int(byte) << " ";
//     }
//     std::cout << std::endl;

//     // 读取夹爪位置
//     auto read_position_command = gripper.getGripperPosition();
//     std::cout << "Read Position Command: ";
//     for (auto byte : read_position_command) {
//         std::cout << std::hex << std::setw(2) << std::setfill('0') << int(byte) << " ";
//     }
//     std::cout << std::endl;

//     return 0;
// }
