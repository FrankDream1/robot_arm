#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <vector>

class ForwardKinematics {
public:
    static std::vector<double> compute(
        const std::vector<double> &joint_angles) {
        // TODO: 读取关节角度（rad），返回末端位置 (x,y,z) 单位 cm，使用 MoveIt2 API
        double x = 0.1, y = 0.1, z = 0.1;
        return {x, y, z};
    }
};

#endif  // KINEMATICS_HPP_
