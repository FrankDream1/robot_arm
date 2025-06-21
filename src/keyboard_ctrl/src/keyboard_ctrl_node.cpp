#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <unordered_map>

using namespace std::chrono;

struct KeyControl {
    uint8_t motor_idx;  // 电机索引，0-9表示机械臂电机，10-11表示行走轮电机
    uint8_t cmd_value;  // 控制值，2bit表示
    bool is_pressed;    // 是否按下
    steady_clock::time_point last_press_time;   // 上次按下时间，用于处理长按逻辑
};

class KeyboardControlNode : public rclcpp::Node {
public:
    KeyboardControlNode() : Node("keyboard_control_node") {
        // 发布控制指令话题
        publisher_ = this->create_publisher<std_msgs::msg::String>("/motor_command", 10);

        configure_terminal(); // 配置终端为非规范模式，禁用回显
        init_key_map(); // 初始化按键映射

        // 创建一个定时器，每100ms调用一次timer_callback()，用于读取键盘输入和发布控制指令
        timer_ = this->create_wall_timer(100ms, std::bind(&KeyboardControlNode::timer_callback, this));
    }

    ~KeyboardControlNode() {
        restore_terminal();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // 控制指令发布器
    rclcpp::TimerBase::SharedPtr timer_; // 定时器
    uint8_t motor_states_[12] = {0};
    std::unordered_map<char, KeyControl> key_map_; // 键盘按键映射
    static struct termios orig_term_; // 原始终端设置
    bool f2_needs_clear_ = false;
    bool b2_needs_clear_ = false;

    // 配置终端为非规范模式，禁用回显
    void configure_terminal() {
        tcgetattr(STDIN_FILENO, &orig_term_); // 获取当前终端设置
        atexit(restore_terminal); // 确保程序结束时恢复原始终端设置
        struct termios new_term = orig_term_; // 设置新的终端属性
        new_term.c_lflag &= ~(ICANON | ECHO); // 禁用规范模式和回显
        new_term.c_cc[VMIN] = 0; // 设置最小字符数为0，允许非阻塞读取
        new_term.c_cc[VTIME] = 0; // 设置超时为0，立即返回
        tcsetattr(STDIN_FILENO, TCSANOW, &new_term); // 应用新的终端设置
    }

    // 恢复原始终端设置
    static void restore_terminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
    }

    // 初始化按键映射，键值对形式：{按键字符, {电机索引, 控制值}}
    void init_key_map() {
        key_map_ = {
            {'a', {0, 0x01}}, {'z', {0, 0x02}}, // L1
            {'s', {1, 0x02}}, {'x', {1, 0x01}}, // L2
            {'d', {2, 0x01}}, {'c', {2, 0x02}}, {'e', {2, 0x03}}, // L3
            {'f', {3, 0x01}}, {'v', {3, 0x02}}, // L4
            {'g', {4, 0x02}}, {'b', {4, 0x01}}, // R1
            {'h', {5, 0x01}}, {'n', {5, 0x02}}, // R2
            {'j', {6, 0x01}}, {'m', {6, 0x02}}, {'u', {6, 0x03}}, // R3
            {'k', {7, 0x01}}, {',', {7, 0x02}}, // R4
            {'l', {8, 0x01}}, {'.', {8, 0x02}}, // F1
            {';', {9, 0x02}}, {'/', {9, 0x01}}, // B1
            {'1', {10, 0x01}}, {'2', {10, 0x02}}, {'3', {10, 0x03}}, // F2
            {'4', {11, 0x01}}, {'5', {11, 0x02}}, {'6', {11, 0x03}} // B2
        };
    }

    void timer_callback() {
        char c;
        int max_read = 10;  // 每次最多处理 10 个按键事件，防卡死
        int read_count = 0;

        while (read_count < max_read && read(STDIN_FILENO, &c, 1) > 0) {
            read_count++;
            auto it = key_map_.find(c);
            if (it != key_map_.end()) {
                motor_states_[it->second.motor_idx] = it->second.cmd_value;
                it->second.is_pressed = true;
                it->second.last_press_time = steady_clock::now();

                if (it->second.motor_idx == 10) {
                    f2_needs_clear_ = true;
                } else if (it->second.motor_idx == 11) {
                    b2_needs_clear_ = true;
                }
            }
        }

        auto now = steady_clock::now();
        for (auto &pair : key_map_) {
            auto &state = pair.second;
            if (state.motor_idx <= 9) {
                if (state.is_pressed) {
                    auto elapsed = duration_cast<milliseconds>(now - state.last_press_time).count();
                    if (elapsed > 300) {
                        motor_states_[state.motor_idx] = 0x00;
                        state.is_pressed = false;
                    }
                }
            }
        }

        uint32_t cmd = 0;
        for (int i = 0; i < 12; ++i) {
            cmd |= (motor_states_[i] & 0x03) << (22 - i * 2);
        }

        char buf[9];
        snprintf(buf, sizeof(buf), "%06X", cmd);
        auto msg = std_msgs::msg::String();
        msg.data = buf;
        publisher_->publish(msg);

        if (f2_needs_clear_) {
            motor_states_[10] = 0x00;
            f2_needs_clear_ = false;
        }
        if (b2_needs_clear_) {
            motor_states_[11] = 0x00;
            b2_needs_clear_ = false;
        }
    }

};

struct termios KeyboardControlNode::orig_term_; // 静态成员变量，用于保存原始终端设置

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // 初始化ROS2
    rclcpp::spin(std::make_shared<KeyboardControlNode>()); // 启动节点
    rclcpp::shutdown(); // 清理资源
    return 0;
}
