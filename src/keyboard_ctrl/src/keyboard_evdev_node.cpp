#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

        feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motor_feedback", 10,
            std::bind(&KeyboardEvdevNode::feedback_callback, this, std::placeholders::_1)
        );

        device_path_ = declare_parameter<std::string>("device", "/dev/input/by-path/platform-3610000.usb-usb-0:2.2:1.0-event-kbd");

        fd_ = open(device_path_.c_str(), O_RDONLY);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开输入设备: %s", device_path_.c_str());
            throw std::runtime_error("无法打开输入设备");
        }

        init_key_map();
        // 每100ms调用回调函数
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyboardEvdevNode::timer_callback, this));
    }

    ~KeyboardEvdevNode() {
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;
    std::string device_path_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;  // 新增：反馈订阅器
    rclcpp::TimerBase::SharedPtr timer_;

    uint8_t motor_states_[12] = {0};    // 记录每个电机的内容
    uint8_t rem1_idx = 0x00;    // 记录10的内容
    uint8_t rem2_idx = 0x00;    // 记录11的内容
    std::unordered_map<int, std::pair<int, uint8_t>> key_map_; // code -> (motor_idx, cmd)

    bool rem1_latch_ = false;   // 记录状态10
    bool rem2_latch_ = false;   // 记录状态11

    void init_key_map() {
        key_map_ = {
            // 好像Q、R用不了，松开时000000
            // A->L1，向上，400000；Z->L1，向下，800000；Q->双机械臂工作位(速度环) 
            {KEY_A, {0, 0x01}}, {KEY_Z, {0, 0x02}}, {KEY_Q, {0, 0x03}},
            // S->L2，向内，100000；X->L2，向外，200000；W->F2，向里，300000
            {KEY_S, {1, 0x01}}, {KEY_X, {1, 0x02}}, {KEY_W, {1, 0x03}},
            // D->L3，向前，040000；C->L3，向后，080000；E->L3、R3速度使能，0C0000
            {KEY_D, {2, 0x01}}, {KEY_C, {2, 0x02}}, {KEY_E, {2, 0x03}}, 
            // F->L4，逆时针，010000；V->L4，顺时针，020000       
            {KEY_F, {3, 0x01}}, {KEY_V, {3, 0x02}}, {KEY_R, {3, 0x03}},
            // G->R1，向上，004000；B->R1，向下，008000，T->F2，向外00C000
            {KEY_G, {4, 0x02}}, {KEY_B, {4, 0x01}}, {KEY_T, {4, 0x03}},
            // H->R2，向内，001000；N->R2，向外，002000
            {KEY_H, {5, 0x01}}, {KEY_N, {5, 0x02}}, 
            // J->R3，向前，000400；M->R3，向后，000800；U->L3、R3位置使能，000C00
            {KEY_J, {6, 0x01}}, {KEY_M, {6, 0x02}}, {KEY_U, {6, 0x03}}, 
            // K->R4，逆时针，000100；,->R4，顺时针，000200
            {KEY_K, {7, 0x01}}, {KEY_COMMA, {7, 0x02}},
            // L->F1，向前，000040；.->F1，向后，000080
            {KEY_L, {8, 0x01}}, {KEY_DOT, {8, 0x02}},
            // ;->F2，向前，000010；/->F2，向后，000020
            {KEY_SEMICOLON, {9, 0x01}}, {KEY_SLASH, {9, 0x02}},
            // 1->双机械臂初始化，000004；2->双机械臂工作位，000008；3->压紧轮初始化，00000C
            {KEY_1, {10, 0x01}}, {KEY_2, {10, 0x02}}, {KEY_3, {10, 0x03}},
            // 4->压紧轮工作位，000001；5->步进电机初始化，00000E
            {KEY_4, {11, 0x01}}, {KEY_5, {11, 0x02}}, {KEY_6, {11, 0x03}}
        };
    }
    
    void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到电机反馈数据:");
        
        // 打印所有关节名称和位置
        /*
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "关节 %s: 位置 = %.2f 弧度", 
                        msg->name[i].c_str(), msg->position[i]);
        }
        
        // 打印力矩数据（如果有）
        for (size_t i = 0; i < msg->effort.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "力矩 %zu: %.2f Nm", i, msg->effort[i]);
        }
        */
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
                        int idx = it->second.first;         // 获取电机编号
                        uint8_t cmd = it->second.second;    // 获取电机状态

                        if (ev.value == 1) {  // 按下
                            motor_states_[idx] = cmd;
                            RCLCPP_INFO(this->get_logger(), "Motor %d set to %02X", idx, cmd);
                            if(idx == 10){      // 锁存状态10
                                rem1_latch_ = true;
                                rem1_idx = cmd;
                                rem2_latch_ = false;
                                rem2_latch_ = 0x00;
                            }
                            else if(idx == 11){ // 锁存状态11
                                rem1_latch_ = false;
                                rem1_idx = 0x00;
                                rem2_latch_ = true;
                                rem2_idx = cmd;
                            }
                            else{   // 清除锁存状态
                                rem1_latch_ = false;
                                rem1_idx = 0x00;
                                rem2_latch_ = false;
                                rem2_idx = 0x00;
                                motor_states_[10] = 0x00;
                                motor_states_[11] = 0x00;
                            }
                        } 
                        else if (ev.value == 0) {  // 松开
                            if (rem1_latch_ == true || rem2_latch_ == true){    // 置入锁存状态
                                motor_states_[10] = rem1_idx;
                                motor_states_[11] = rem2_idx;
                            }
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
                // 每个电机占位2bit，motor0占最高的两位，motor1...类推
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
