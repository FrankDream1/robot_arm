#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <iostream>

// 定义一个继承自 rclcpp::Node 的串口通信节点类
class SerialCommNode : public rclcpp::Node {
public:
    SerialCommNode() : Node("serial_comm_node") {
        // 打开串口设备 /dev/ttyUSB0
        fd_ = openSerialPort("/dev/ttyUSB0");
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "打开串口失败");
            return;
        }

        // 创建发布器，用于发布电机反馈数据（角度和扭矩）
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/motor_feedback", 10);

        // 创建订阅器，接收来自话题 /motor_command 的控制指令
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/motor_command", 10,
            std::bind(&SerialCommNode::command_callback, this, std::placeholders::_1)
        );

        // 创建一个定时器，每 100ms 调用一次 timer_callback()，用于周期性通信
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
        for (uint16_t i = 0; i < length; ++i) {
            crc ^= (uint16_t)(data[i] << 8);
            for (uint8_t j = 0; j < 8; j++) {
                crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
            }
        }
        return crc;
    }

    // 打开串口并配置参数
    int openSerialPort(const char* port) {
        int fd = open(port, O_RDWR | O_NOCTTY);
        if (fd < 0) return -1;

        termios options{};
        tcgetattr(fd, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD); // 允许接收
        options.c_cflag &= ~PARENB; // 无校验位
        options.c_cflag &= ~CSTOPB; // 1 个停止位
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8; // 8 位数据位
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入模式
        options.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
        tcsetattr(fd, TCSANOW, &options);

        return fd;
    }

    // 发送控制帧给下位机，cmd 为 16 位控制字
    void sendControlFrame(uint16_t cmd) {
        uint8_t frame[6];
        frame[0] = 0xAA; // 帧头
        frame[1] = (cmd >> 8) & 0xFF;
        frame[2] = cmd & 0xFF;
        uint16_t crc = CRC16_CCITT(&frame[1], 2); // 计算 CRC
        frame[3] = (crc >> 8) & 0xFF;
        frame[4] = crc & 0xFF;
        frame[5] = 0x55; // 帧尾
        write(fd_, frame, sizeof(frame)); // 写入串口
    }

    // 接收下位机反馈帧，解析角度和扭矩，并发布到话题
    void receiveFeedbackFrame() {
        uint8_t buffer[44];
        int total = 0;
        // 持续读取直到完整收到 44 字节
        while (total < 44) {
            int n = read(fd_, buffer + total, 44 - total);
            if (n > 0) total += n;
        }

        // 校验帧头帧尾
        if (buffer[0] != 0x55 || buffer[43] != 0xAA) return;

        // CRC 校验
        uint16_t crc = CRC16_CCITT(&buffer[1], 40);
        uint16_t received_crc = (buffer[41] << 8) | buffer[42];
        if (crc != received_crc) return;

        // 提取电机角度（8个 float）和扭矩（2个 float）
        float angles[8];
        float torques[2];
        memcpy(angles, &buffer[1], 32);
        memcpy(torques, &buffer[33], 8);

        // 构造 ROS2 JointState 消息
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        for (int i = 0; i < 8; ++i) {
            msg.name.push_back("motor" + std::to_string(i + 1));
            msg.position.push_back(angles[i]);
        }
        msg.effort.push_back(torques[0]);
        msg.effort.push_back(torques[1]);

        joint_pub_->publish(msg); // 发布反馈消息
    }

    // 定时器回调函数，发送控制帧并读取反馈
    void timer_callback() {
        sendControlFrame(current_command_);
        receiveFeedbackFrame();
    }

    // 订阅器回调函数，接收字符串控制指令（16bit 十六进制）
    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            uint16_t cmd = std::stoul(msg->data, nullptr, 16); // 转换为 16 位整数
            current_command_ = cmd; // 更新控制命令
            RCLCPP_INFO(this->get_logger(), "收到控制命令: 0x%04X", cmd);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "命令格式错误，应为十六进制字符串（如 '0001'）");
        }
    }
};

// 主函数入口
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // 初始化 ROS2
    rclcpp::spin(std::make_shared<SerialCommNode>()); // 启动节点
    rclcpp::shutdown(); // 清理资源
    return 0;
}