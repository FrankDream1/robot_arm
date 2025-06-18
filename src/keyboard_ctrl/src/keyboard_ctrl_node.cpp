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
    uint8_t motor_idx;
    uint8_t cmd_value;
    bool is_pressed;
    steady_clock::time_point last_press_time;
};

class KeyboardControlNode : public rclcpp::Node {
public:
    KeyboardControlNode() : Node("keyboard_control_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/motor_command", 10);
        configure_terminal();
        init_key_map();
        timer_ = this->create_wall_timer(100ms, std::bind(&KeyboardControlNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Keyboard control node started");
    }

    ~KeyboardControlNode() {
        restore_terminal();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint8_t motor_states_[12] = {0};
    std::unordered_map<char, KeyControl> key_map_;
    static struct termios orig_term_;

    void configure_terminal() {
        tcgetattr(STDIN_FILENO, &orig_term_);
        atexit(restore_terminal);
        struct termios new_term = orig_term_;
        new_term.c_lflag &= ~(ICANON | ECHO);
        new_term.c_cc[VMIN] = 0;
        new_term.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
    }

    static void restore_terminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
    }

    void init_key_map() {
        key_map_ = {
            {'a', {0, 0x01}}, {'z', {0, 0x02}},
            {'s', {1, 0x01}}, {'x', {1, 0x02}},
            {'d', {2, 0x01}}, {'c', {2, 0x02}}, {'e', {2, 0x03}},
            {'f', {3, 0x01}}, {'v', {3, 0x02}},
            {'g', {4, 0x01}}, {'b', {4, 0x02}},
            {'h', {5, 0x01}}, {'n', {5, 0x02}},
            {'j', {6, 0x01}}, {'m', {6, 0x02}}, {'u', {6, 0x03}},
            {'k', {7, 0x01}}, {'l', {7, 0x02}},
            {'q', {8, 0x01}}, {'w', {8, 0x02}},
            {'t', {9, 0x01}}, {'y', {9, 0x02}},
            {'1', {10, 0x01}}, {'2', {10, 0x02}}, {'3', {10, 0x03}},
            {'4', {11, 0x01}}, {'5', {11, 0x02}}, {'6', {11, 0x03}}
        };
    }

    void timer_callback() {
        char c;
        while (read(STDIN_FILENO, &c, 1) > 0) {
            auto it = key_map_.find(c);
            if (it != key_map_.end()) {
                motor_states_[it->second.motor_idx] = it->second.cmd_value;
                it->second.is_pressed = true;
                it->second.last_press_time = steady_clock::now();
            }
        }

        auto now = steady_clock::now();
        for (auto &pair : key_map_) {
            auto &state = pair.second;
            if (state.motor_idx <= 9 || state.motor_idx == 2 || state.motor_idx == 6) {
                if (state.is_pressed) {
                    auto elapsed = duration_cast<milliseconds>(now - state.last_press_time).count();
                    if (elapsed > 300) {
                        motor_states_[state.motor_idx] = 0x00;
                        state.is_pressed = false;
                    }
                }
            }
        }

        // F2 B2 无操作时发 00 忽略
        if (motor_states_[10] != 0x01 && motor_states_[10] != 0x02 && motor_states_[10] != 0x03) {
            motor_states_[10] = 0x00;
        }
        if (motor_states_[11] != 0x01 && motor_states_[11] != 0x02 && motor_states_[11] != 0x03) {
            motor_states_[11] = 0x00;
        }

        // 拼成 24bit 数据（按顺序组合12个2bit控制位）
        uint32_t cmd = 0;
        for (int i = 0; i < 12; ++i) {
            cmd |= (motor_states_[i] & 0x03) << (22 - i * 2);
        }

        char buf[9];
        snprintf(buf, sizeof(buf), "%06X", cmd);
        auto msg = std_msgs::msg::String();
        msg.data = buf;
        publisher_->publish(msg);
    }
};

struct termios KeyboardControlNode::orig_term_;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardControlNode>());
    rclcpp::shutdown();
    return 0;
}

