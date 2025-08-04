#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@说明: 订阅螺栓三维坐标
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from learning_interface.msg import ObjectPosition  # 自定义坐标消息[4](@ref)

class BoltPositionSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2节点父类初始化
        
        # 创建订阅者（消息类型、话题名、回调函数、队列长度）
        self.sub = self.create_subscription(
            ObjectPosition,                       # 使用自定义消息类型[4](@ref)
            "bolt_positions",                     # 话题名称（与发布者一致）
            self.position_callback,               # 回调函数
            10
        )
        self.get_logger().info(f"已创建订阅者：话题 bolt_positions")

    def position_callback(self, msg):
        """处理接收到的坐标消息"""
        # 提取并格式化坐标值（保留三位小数）
        x = round(msg.x, 3)
        y = round(msg.y, 3)
        z = round(msg.z, 3)
        
        # 结构化日志输出
        self.get_logger().info(
            f"螺栓位置: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m",
            throttle_duration_sec=1  # 限流：每秒最多打印1次[5](@ref)
        )

def main(args=None):
    rclpy.init(args=args)
    node = BoltPositionSubscriber("bolt_position_subscriber")
    try:
        rclpy.spin(node)  # 阻塞式处理消息
    except KeyboardInterrupt:
        node.get_logger().info("用户终止操作")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
