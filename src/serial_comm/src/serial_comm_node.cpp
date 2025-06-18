#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <iostream>

/*
    上位机给下位机下发的数据帧（7字节），格式为0xAA（帧头）+12个电机控制命令
    （L1+L2+L3+L4+R1+R2+R3+R4+F1+B1+F2+B2，12*2bits控制位+6bits空闲位（3bytes））+CRC校验位（2bytes）+0x55（帧尾）
        其中，机械臂和行走轮电机的控制位使用两位来表示控制状态指令，00表示停止，01表示正转，10表示反转，11表示优摩特设定为速度模式
        夹紧轮电机的控制位使用两位来表示控制状态指令，00表示夹紧，01表示张开，10表示压紧后张开微调，11表示忽略
    下位机反馈给上位机的数据帧（44字节），格式为0x55（帧头）+8个电机角度值
    （L1+L2+L3+R1+R2+R3+F1+B1，8*4bytes（float）（32bytes））+两个电机电流值（L4+R4，2*4bytes（float）（8bytes））+CRC校验位（2bytes）+0xAA（帧尾）
*/

// 定义一个继承自rclcpp::Node的串口通信节点类
class SerialCommNode : public rclcpp::Node {
public:
    SerialCommNode() : Node("serial_comm_node") {
        // 打开串口设备 /dev/ttyUSB0
        fd_ = openSerialPort("/dev/ttyUSB0");
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "打开串口失败");
            return;
        }

        // 创建发布器，用于发布电机反馈数据
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/motor_feedback", 10);

        // 创建订阅器，接收来自话题/motor_command的控制指令
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/motor_command", 10,
            std::bind(&SerialCommNode::command_callback, this, std::placeholders::_1)
        );

        // 创建一个定时器，每100ms调用一次timer_callback()，用于周期性通信
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SerialCommNode::timer_callback, this)
        );
    }

private:
    int fd_; // 串口文件描述符
    rclcpp::TimerBase::SharedPtr timer_; // 定时器
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_; // 反馈话题发布器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_; // 控制命令订阅器
    uint16_t current_command_ = 0x0000; // 当前控制指令

    // CRC16-CCITT 校验函数，用于数据完整性验证
    uint16_t CRC16_CCITT(const uint8_t* data, uint16_t length) {
        uint16_t crc = 0xFFFF;
        for(uint8_t i = 0; i < len; i++){
            crc ^= (uint16_t)(data[i] << 8);
            for(uint8_t j = 0; j < 8; j++){
                if(crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
		    }
	    }
        return crc;
    }

    // 打开串口并配置参数
    int openSerialPort(const char* port) {
        int fd = open(port, O_RDWR | O_NOCTTY); // 打开串口设备
        if (fd < 0) return -1;

        termios options{}; // 初始化termios结构体
        tcgetattr(fd, &options); // 获取当前串口配置
        cfsetispeed(&options, B115200); // 设置输入波特率
        cfsetospeed(&options, B115200); // 设置输出波特率
        options.c_cflag |= (CLOCAL | CREAD); // 允许接收
        options.c_cflag &= ~PARENB; // 无校验位
        options.c_cflag &= ~CSTOPB; // 1个停止位
        options.c_cflag &= ~CSIZE; // 清除数据位设置
        options.c_cflag |= CS8; // 8位数据位
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入模式
        options.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
        tcsetattr(fd, TCSANOW, &options); // 应用配置

        return fd;
    }

    // 发送控制帧给下位机，cmd24位控制字
    void sendControlFrame(uint32_t cmd) {
        uint8_t frame[7];
        frame[0] = 0xAA; // 帧头
        frame[1] = (cmd >> 16) & 0xFF;
        frame[2] = (cmd >> 8) & 0xFF;
        frame[3] = cmd & 0xFF;
        uint16_t crc = CRC16_CCITT(&frame[1], 3); // 计算CRC
        frame[4] = (crc >> 8) & 0xFF;
        frame[5] = crc & 0xFF;
        frame[6] = 0x55; // 帧尾
        write(fd_, frame, sizeof(frame)); // 写入串口
    }

    // 接收下位机反馈帧，解析角度和扭矩，并发布到话题
    void receiveFeedbackFrame() {
        uint8_t buffer[44];
        int total = 0;
        // 持续读取直到完整收到44字节
        while (total < 44) {
            int n = read(fd_, buffer + total, 44 - total);
            if (n > 0) total += n;
        }

        // 校验帧头帧尾
        if (buffer[0] != 0x55 || buffer[43] != 0xAA) return;

        // CRC校验
        uint16_t crc = CRC16_CCITT(&buffer[1], 40);
        uint16_t received_crc = (buffer[41] << 8) | buffer[42];
        if (crc != received_crc) return;

        // 提取电机角度（8个float）和电流（2个float）
        float angles[8];
        float currents[2];
        memcpy(angles, &buffer[1], 32);
        memcpy(currents, &buffer[33], 8);

        // 构造ROS2 JointState消息
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        const char* motor_names[8] = {"L1", "L2", "L3", "R1", "R2", "R3", "F1", "B1","L4","R4"};
        for (int i = 0; i < 8; ++i) {
            msg.name.push_back("motor" + std::string(motor_names[i]));
            msg.position.push_back(angles[i]);
        }
        msg.name.push_back("motorL4");
        msg.name.push_back("motorR4");
        msg.effort.push_back(currents[0]);
        msg.effort.push_back(currents[1]);

        joint_pub_->publish(msg); // 发布反馈消息
    }

    // 定时器回调函数，发送控制帧并读取反馈
    void timer_callback() {
        sendControlFrame(current_command_);
        receiveFeedbackFrame();
    }

    // 订阅器回调函数，接收字符串控制指令（24bits十六进制）
    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            uint32_t cmd = std::stoul(msg->data, nullptr, 16); // 转换为32位整数
            current_command_ = cmd; // 更新控制命令
            RCLCPP_INFO(this->get_logger(), "收到控制命令: 0x%06X", cmd); 
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "命令格式错误，应为24位十六进制字符串（如 '0x123456'）");
        }
    }
};

// 主函数入口
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // 初始化ROS2
    rclcpp::spin(std::make_shared<SerialCommNode>()); // 启动节点
    rclcpp::shutdown(); // 清理资源
    return 0;
}