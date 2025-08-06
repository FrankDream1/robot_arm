#include "controller.hpp"
#include <sstream>
#include <iomanip>

AutoController::AutoController() 
: Node("auto_controller"),
pid_x_(0.5, 0.0, 0.05),
pid_y_(0.5, 0.0, 0.05),
pid_z_(0.5, 0.0, 0.05)
{
    image_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/image_transfer", 10,
        std::bind(&AutoController::image_cb, this, std::placeholders::_1));

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/motor_feedback", 10,
        std::bind(&AutoController::joint_cb, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<std_msgs::msg::String>("/motor_command", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AutoController::control_loop, this));
}

void AutoController::image_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
	if (msg->data.size() >= 3) {
		target_ = { msg->data[0], msg->data[1], msg->data[2] };
		have_target_ = true;
		RCLCPP_INFO(get_logger(),
		"Target [cm]: x=%.2f, y=%.2f, z=%.2f",
		target_[0], target_[1], target_[2]);
	}
}

void AutoController::joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  	joints_ = *msg;
 	have_joints_ = true;
}

void AutoController::control_loop() {
	if (!have_target_ || !have_joints_) return;

	// 1. 正运动学
	auto tcp = ForwardKinematics::compute(joints_.position);  // cm

	// 2. PID 计算
	double dt = 0.1;  // 100 ms
	double ux = pid_x_.compute(target_[0], tcp[0], dt);
	double uy = pid_y_.compute(target_[1], tcp[1], dt);
	double uz = pid_z_.compute(target_[2], tcp[2], dt);

	// 3. 映射到关节增量 (此处示例：前3 关节)
	std::vector<double> cmd = joints_.position;
	for (int i = 0; i < 3 && i < (int)cmd.size(); ++i) {
		cmd[i] += (i==0?ux:(i==1?uy:uz));
	}

	// 4. 拼帧：12*2bit=24bit 控制，剩余补 0
	uint32_t frame = 0;
	for (int i = 0; i < 12; ++i) {
		uint8_t v = 0;  // 默认停止
		if (i < 3) {    // 用高 2 bits 表示关节增量方向
		v = (cmd[i] > joints_.position[i]) ? 0x01 : 0x02;
		}
		frame |= (v & 0x03) << (22 - 2*i);
	}

	// 5. 变成 6 字节：0xAA + 3 bytes 数据 + CRC16 + 0x55
	//    此处仅发 3 bytes 数据示例:
	char buf[9];
	snprintf(buf, sizeof(buf), "%06X", frame & 0xFFFFFF);

	auto out = std_msgs::msg::String();
	out.data = buf;
	cmd_pub_->publish(out);

	RCLCPP_INFO(get_logger(), "Cmd frame: %s", buf);
}
