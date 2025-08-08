import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 FPS
        self.bridge = CvBridge()

        # Открой видеопоток с камеры
        self.cap = cv2.VideoCapture(2)  # Поменяй на нужный индекс камеры

        if not self.cap.isOpened():
            self.get_logger().error('Не удалось открыть камеру')
        else:
            self.get_logger().info('Камера успешно запущена')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Не удалось прочитать кадр с камеры')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Прерывание по Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()
