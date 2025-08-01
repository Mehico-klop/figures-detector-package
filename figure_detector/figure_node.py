import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class FigureDetector(Node):
    def __init__(self):
        super().__init__('figure_detector')
        self.bridge = CvBridge()

        # Путь к модели
        pkg_path = Path(get_package_share_directory('figure_detector'))
        model_path = Path(__file__).parent.parent / 'models' / 'best.pt'

        # Загрузка YOLO
        self.model = YOLO(str(model_path))
        self.get_logger().info(f"YOLOv11 модель загружена: {model_path}")

        # Подписка на камеру
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, msg):
        # Конвертация ROS-изображения в OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Детекция
        results = self.model(frame)[0]

        found_objects = []

        for box in results.boxes:
            cls = int(box.cls)
            conf = float(box.conf)
            name = self.model.names[cls]
            found_objects.append(f"{name} ({conf:.2f})")

            # Для отладки рисуем прямоугольники на видеоряде
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f'{name} {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # Вывод в терминал
        if found_objects:
            self.get_logger().info(f"Вижу: {', '.join(found_objects)}")
        else:
            self.get_logger().info("Ничего не вижу")

        # Отображение окна (опционально)
        cv2.imshow("YOLOv11", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FigureDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
