import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(  #create_subscription()：创建话题订阅器的方法（消息类型，话题名称，回调函数，缓冲区）
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.br = CvBridge()  #创建CvBridge实例，后续用于 ROS Image → OpenCV Mat 的转换
        self.get_logger().info("图片订阅器准备就绪...")
    
    # 回调函数
    def image_callback(self, msg):
        self.get_logger().info("收到图像消息")
        
        # 转换ROS消息为OpenCV格式
        current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        
        # 显示图片
        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

# 节点生命周期管理
def main(args=None):
    rclpy.init(args=args)
    subscriber = ImageSubscriber()
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()