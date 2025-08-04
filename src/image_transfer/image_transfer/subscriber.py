import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import os
import ament_index_python
import sys
import message_filters
from message_filters import ApproximateTimeSynchronizer
import time
from std_msgs.msg import String
import json
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo
from learning_interface.msg import ObjectPosition  # 自定义的目标位置消息

class BoltDetectionNode(Node):
    def __init__(self):
        super().__init__('bolt_detection_node')
        
        # 设置话题名称
        self.color_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'

        self.camera_info_topic = '/camera/camera/color/camera_info'  # 确保先定义

        self.pub = self.create_publisher(ObjectPosition, "bolt_positions", 10)
        
        self.get_logger().info(f"订阅彩色图: {self.color_topic}")
        self.get_logger().info(f"订阅深度图: {self.depth_topic}")
        
        # 订阅彩色图像和深度图像（时间同步）
        color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        
        # 设置时间同步器（10ms 时间差容限）
        self.ts = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.01)
        self.ts.registerCallback(self.image_callback)

        # 订阅相机内参
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        self.depth_intrinsics = None  # 存储深度图像的内参
        
        # 初始化YOLO模型
        self.model = self.initialize_yolo_model()
        self.br = CvBridge()
        self.last_depth_map = None
        self.get_logger().info("螺栓检测节点已启动，等待图像数据...")
    
    # 相机内参回调函数
    def camera_info_callback(self, msg):
        """处理相机内参消息"""
        if self.depth_intrinsics is None:
            # 创建RealSense内参对象
            self.depth_intrinsics = rs.intrinsics()
            self.depth_intrinsics.width = msg.width
            self.depth_intrinsics.height = msg.height
            self.depth_intrinsics.ppx = msg.k[2]
            self.depth_intrinsics.ppy = msg.k[5]
            self.depth_intrinsics.fx = msg.k[0]
            self.depth_intrinsics.fy = msg.k[4]
            self.depth_intrinsics.model = rs.distortion.none  # 假设无畸变
            # 将畸变系数设置为零数组
            self.depth_intrinsics.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            self.get_logger().info("已接收相机内参，深度处理已启用")

    def initialize_yolo_model(self):
        """加载YOLO模型"""
        try:
            # 获取包共享目录的完整路径
            pkg_share_dir = ament_index_python.get_package_share_directory('image_transfer')
            
            # 构建模型文件完整路径
            model_path = os.path.join(pkg_share_dir, 'yolo', 'best.pt')
            
            self.get_logger().info(f"加载YOLO模型: {model_path}")
            
            # 检查文件是否存在
            if not os.path.exists(model_path):
                self.get_logger().error(f"模型文件不存在: {model_path}")
                # 获取包安装的lib路径
                lib_path = os.path.join(os.path.dirname(pkg_share_dir), 'lib', 
                                       f'python{sys.version_info.major}.{sys.version_info.minor}', 
                                       'site-packages', 'image_transfer')
                self.get_logger().info(f"检查备选路径: {lib_path}/yolo/best.pt")
                if os.path.exists(os.path.join(lib_path, 'yolo', 'best.pt')):
                    model_path = os.path.join(lib_path, 'yolo', 'best.pt')
                    self.get_logger().warn(f"使用备选路径: {model_path}")
                else:
                    self.get_logger().error("所有备选路径均未找到模型文件！")
                    return None
            
            # 加载模型
            model = YOLO(model_path)
            
            # 设置模型参数
            model.conf = 0.6  # 置信度阈值
            
            self.get_logger().info("YOLO模型加载成功")
            return model
        except Exception as e:
            self.get_logger().error(f"YOLO模型加载失败: {str(e)}")
            return None

    def image_callback(self, color_msg, depth_msg):
        """处理同步的彩色和深度图像"""
        try:
            # 转换彩色图像
            color_frame = self.br.imgmsg_to_cv2(color_msg, "bgr8")
            
            # 转换深度图像（16位单通道）
            depth_map = self.br.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth_map = np.array(depth_map, dtype=np.float32) / 1000.0  # 转换为米
            self.last_depth_map = depth_map  # 存储深度图用于后续处理
            
            # 创建深度图可视化（伪彩色）
            depth_viz = self.visualize_depth(depth_map)
            
            # 处理彩色图像并检测螺栓
            if self.model:
                # 使用YOLO模型检测
                yolo_results = self.model(color_frame)
                
                # 处理检测结果并在图像上绘制
                detected_frame, bolt_count = self.process_detections(color_frame, yolo_results, depth_map)
                
                # 显示结果（分别显示三张图）
                self.display_separate_images(detected_frame, color_frame, depth_viz, bolt_count)
                
        except Exception as e:
            self.get_logger().error(f"图像处理错误: {str(e)}")
            self.get_logger().error(f"错误详情: {str(e)}", throttle_duration_sec=1)
    
    def process_detections(self, frame, results, depth_map):
        """处理检测结果并返回带有标注的图像"""
        detections = results[0]
        bolt_count = 0

        detected_positions = []  # 存储所有检测到的位置
        
        # 检查是否有检测结果
        if detections.boxes is not None and len(detections) > 0:
            # 解析检测结果
            boxes = detections.boxes.xyxy.cpu().numpy()  # 边界框坐标
            confs = detections.boxes.conf.cpu().numpy()  # 置信度
            classes = detections.boxes.cls.cpu().numpy()  # 类别
            
            # 在图像上绘制结果
            for box, conf, cls_id in zip(boxes, confs, classes):
                # 解析边界框坐标
                x1, y1, x2, y2 = map(int, box)
                
                # 计算中心点
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                # 获取类别名称
                class_name = detections.names.get(int(cls_id), "object")
                
                # 获取深度信息 - 添加有效性检查
                if depth_map is not None and depth_map.size > 0:
                    if 0 <= center_y < depth_map.shape[0] and 0 <= center_x < depth_map.shape[1]:
                        distance = depth_map[center_y, center_x]
                    else:
                        distance = 0.0
                else:
                    distance = 0.0
                
                # ===== 新增: 计算真实三维坐标 =====
                real_x, real_y, real_z = 0.0, 0.0, 0.0
                if self.depth_intrinsics is not None and distance > 0:
                    # 使用RealSense函数获取真实三维坐标
                    camera_coordinate = rs.rs2_deproject_pixel_to_point(
                        self.depth_intrinsics, 
                        [center_x, center_y],
                        distance
                    )
                    real_x, real_y, real_z = camera_coordinate
                # ===== 结束新增部分 =====

                # 记录检测信息
                bolt_info = {
                    "pixel_x": center_x,
                    "pixel_y": center_y,
                    "distance": float(distance),  # 转换为原生 float
                    "camera_x": float(real_x),    # 新增
                    "camera_y": float(real_y),    # 新增
                    "camera_z": float(real_z),    # 新增
                    "confidence": float(conf),
                    "class": class_name
                }
                detected_positions.append(bolt_info)

                # 记录检测到螺栓
                if distance > 0:
                    self.get_logger().info(f"检测到螺栓, "
                                           f"相机坐标: ({real_x:.3f}, {real_y:.3f}, {real_z:.3f})m, "
                                           f"置信度: {conf:.2f}")
                else:
                    self.get_logger().info(f"检测到螺栓: ({center_x}, {center_y}), 距离: 无效, 置信度: {conf:.2f}")
                
                bolt_count += 1
                
                # 绘制边界框
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # 创建标签文本
                label = f"{class_name} {conf:.2f}"
                if distance > 0:
                    label += f" | {distance:.2f}m"
                
                # 绘制标签
                cv2.putText(frame, label, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # 绘制中心点
                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
                
                # 绘制坐标文本
                coord_text = f"({real_x:.3f}, {real_y:.3f})"
                cv2.putText(frame, coord_text, (center_x + 10, center_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        # 发布所有检测到的位置
        if detected_positions:
            for bolt in detected_positions:
                position = ObjectPosition()

                position.x = round(bolt['camera_x'], 3)
                position.y = round(bolt['camera_y'], 3)
                position.z = round(bolt['camera_z'], 3)

                self.pub.publish(position)
                self.get_logger().info(
                    f"发布螺栓位置: ({position.x:.3f}, {position.y:.3f}, {position.z:.3f})m",
                    throttle_duration_sec=1  # 限流避免日志刷屏
                )

        """
        # 发布所有检测到的位置
        if detected_positions:
            msg = String()
            msg.data = json.dumps({
                "timestamp": time.time(),
                "positions": detected_positions
            })
            self.pub.publish(msg)
            self.get_logger().info(f"发布 {len(detected_positions)} 个螺栓位置")
        """

        # 显示统计信息
        cv2.putText(frame, f"Bolts: {bolt_count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return frame, bolt_count

    def display_separate_images(self, detected_frame, color_frame, depth_viz, bolt_count):
        """分别显示三张图像：检测结果图、原始彩色图、深度图"""
        # 显示检测结果图
        cv2.imshow("Detected Bolts", detected_frame)
        
        # 显示原始彩色图
        cv2.imshow("Color Image", color_frame)
        
        # 显示深度图（伪彩色）
        cv2.imshow("Depth Map", depth_viz)
        
        # 添加简单的键盘控制
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 按q键退出
            self.get_logger().info("用户请求退出...")
            rclpy.shutdown()
        elif key == ord('s'):  # 按s键保存当前帧
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            cv2.imwrite(f"detected_{timestamp}.png", detected_frame)
            cv2.imwrite(f"color_{timestamp}.png", color_frame)
            cv2.imwrite(f"depth_{timestamp}.png", depth_viz)
            self.get_logger().info(f"保存图像: detected_{timestamp}.png, color_{timestamp}.png, depth_{timestamp}.png")

    def visualize_depth(self, depth_map):
        """将深度图转换为伪彩色可视化"""
        if depth_map is None or depth_map.size == 0 or np.all(depth_map == 0):
            return np.zeros((360, 640, 3), dtype=np.uint8)
        
        # 应用归一化处理（跳过无效值）
        valid_mask = depth_map > 0
        if np.any(valid_mask):
            min_depth = np.min(depth_map[valid_mask])
            max_depth = np.max(depth_map[valid_mask])
        else:
            min_depth, max_depth = 0, 1.0
        
        # 避免除以零
        if max_depth - min_depth < 1e-5:
            normalized = np.zeros_like(depth_map)
        else:
            normalized = (depth_map - min_depth) / (max_depth - min_depth)
        
        # 转换为8位伪彩色
        normalized[~valid_mask] = 0  # 将无效值设为0
        normalized_8u = (normalized * 255).astype(np.uint8)
        pseudo_color = cv2.applyColorMap(normalized_8u, cv2.COLORMAP_JET)
        
        # 在无效区域（如深度为0）设为黑色
        pseudo_color[~valid_mask] = 0
        
        # 添加标题和信息
        cv2.putText(pseudo_color, f"Min: {min_depth:.1f}m", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(pseudo_color, f"Max: {max_depth:.1f}m", (pseudo_color.shape[1]-120, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(pseudo_color, "Depth Map", (10, pseudo_color.shape[0]-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return pseudo_color

    def destroy_node(self):
        """节点销毁时关闭所有窗口"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BoltDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('键盘中断 (CTRL-C) 检测到，退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
