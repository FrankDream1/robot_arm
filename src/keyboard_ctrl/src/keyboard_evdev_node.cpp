#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <unordered_map>
#include <string>
#include <sstream>
#include <sys/select.h>

class KeyboardEvdevNode : public rclcpp::Node {
public:
    KeyboardEvdevNode() : Node("keyboard_evdev_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/motor_command", 10);
        device_path_ = declare_parameter<std::string>("device", "/dev/input/by-path/platform-3610000.usb-usb-0:2.2:1.0-event-kbd");

        fd_ = open(device_path_.c_str(), O_RDONLY);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开输入设备: %s", device_path_.c_str());
            throw std::runtime_error("无法打开输入设备");
        }

        init_key_map();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyboardEvdevNode::timer_callback, this));
    }

    ~KeyboardEvdevNode() {
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;
    std::string device_path_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint8_t motor_states_[12] = {0};
    std::unordered_map<int, std::pair<int, uint8_t>> key_map_; // code -> (motor_idx, cmd)

    void init_key_map() {
        key_map_ = {
            {KEY_A, {0, 0x01}}, {KEY_Z, {0, 0x02}},           // L1
            {KEY_S, {1, 0x02}}, {KEY_X, {1, 0x01}},           // L2
            {KEY_D, {2, 0x01}}, {KEY_C, {2, 0x02}}, {KEY_E, {2, 0x03}}, // L3
            {KEY_F, {3, 0x01}}, {KEY_V, {3, 0x02}},           // L4
            {KEY_G, {4, 0x02}}, {KEY_B, {4, 0x01}},           // R1
            {KEY_H, {5, 0x01}}, {KEY_N, {5, 0x02}},           // R2
            {KEY_J, {6, 0x01}}, {KEY_M, {6, 0x02}}, {KEY_U, {6, 0x03}}, // R3
            {KEY_K, {7, 0x01}}, {KEY_COMMA, {7, 0x02}},       // R4
            {KEY_L, {8, 0x01}}, {KEY_DOT, {8, 0x02}},         // F1
            {KEY_SEMICOLON, {9, 0x02}}, {KEY_SLASH, {9, 0x01}}, // B1
            {KEY_1, {10, 0x01}}, {KEY_2, {10, 0x02}}, {KEY_3, {10, 0x03}}, // F2
            {KEY_4, {11, 0x01}}, {KEY_5, {11, 0x02}}, {KEY_6, {11, 0x03}}  // B2
        };
    }
    
    void timer_callback() {
        struct input_event ev;
        fd_set read_fds;
        struct timeval timeout;
        int max_fd = fd_;

        // 设置 100ms 超时
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;  // 100ms

        // 通过 select 检查是否有输入事件
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        int ret = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);

        // 如果有输入事件
        if (ret > 0 && FD_ISSET(fd_, &read_fds)) {
            // 读取输入事件
            while (read(fd_, &ev, sizeof(ev)) == sizeof(ev)) {
                if (ev.type == EV_KEY) {
                    RCLCPP_INFO(this->get_logger(), "Key %d %s", ev.code, 
                                ev.value == 1 ? "press" : (ev.value == 0 ? "release" : "repeat"));

                    auto it = key_map_.find(ev.code);
                    if (it != key_map_.end()) {
                        int idx = it->second.first;
                        uint8_t cmd = it->second.second;

                        if (ev.value == 1) {  // 按下
                            motor_states_[idx] = cmd;
                            RCLCPP_INFO(this->get_logger(), "Motor %d set to %02X", idx, cmd);
                        } else if (ev.value == 0) {  // 松开
                            if (idx <= 9) {
                                motor_states_[idx] = 0x00;  // 停止
                                RCLCPP_INFO(this->get_logger(), "Motor %d stopped", idx);
                            }
                        }
                    }
                }

                // // 打印 motor_states_ 数组的当前状态
                // RCLCPP_INFO(this->get_logger(), "Motor States: ");
                // for (int i = 0; i < 12; ++i) {
                //     RCLCPP_INFO(this->get_logger(), "Motor %d: %d", i, motor_states_[i]);
                // }

                // 拼接 24bit 控制数据
                uint32_t cmd_data = 0;
                for (int i = 0; i < 12; ++i) {
                    cmd_data |= (motor_states_[i] & 0x03) << (22 - i * 2);
                }

                // // 打印拼接的控制命令
                // RCLCPP_INFO(this->get_logger(), "Control Command: %06X", cmd_data);

                // 转换为十六进制字符串
                char buf[9];
                snprintf(buf, sizeof(buf), "%06X", cmd_data);

                // 发布消息
                auto msg = std_msgs::msg::String();
                msg.data = buf;
                publisher_->publish(msg);
            }
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardEvdevNode>());
    rclcpp::shutdown();
    return 0;
}
