#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <vector>

/**
 * 简单示例：6-DOF 机械臂正运动学
 * 读取关节角度（rad），返回末端位置 (x,y,z) 单位 cm
 * 实际请用 DH 参数或 MoveIt2 API
 */
class ForwardKinematics {
public:
    static std::vector<double> compute(
        const std::vector<double> &joint_angles) {
        // TODO: 根据你机械臂 DH 参数实现
        // 这里仅返回一个示例位置
        double x = 0.1, y = 0.1, z = 0.1;
        return {x, y, z};
    }
};

#endif  // KINEMATICS_HPP_
