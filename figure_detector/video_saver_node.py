import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from pathlib import Path


class VideoSaverNode(Node):
    def __init__(self):
        super().__init__('video_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        env_dir = os.getenv('VIDEO_OUTPUT_DIR')

        # 2. Если задана — используем её, иначе выбираем путь в зависимости от среды
        if env_dir:
            output_base_dir = Path(env_dir)
        else:
        # Проверяем, есть ли признак работы в Docker
            in_docker = Path('/.dockerenv').exists()
            if in_docker:
                output_base_dir = Path('/workspace/video_data')
            else:
                output_base_dir = Path.home() / 'video_data'


        # Инициализация видеозаписи
        time_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.video_path = self.output_dir / f'{time_str}.avi'
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = None
        self.frame_width = None
        self.frame_height = None

        self.get_logger().info(f'Видео будет сохранено в: {self.video_path}')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.out is None:
            self.frame_height, self.frame_width = frame.shape[:2]
            self.out = cv2.VideoWriter(
                str(self.video_path), self.fourcc, 30.0, (self.frame_width, self.frame_height))

        self.out.write(frame)
        #Если нужно показывать видео
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.out:
            self.out.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Прерывание по Ctrl+C')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
