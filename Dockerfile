FROM ros:jazzy

# Установим зависимости ROS 2 и Python
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-usb-cam \
    ros-jazzy-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# Установим нужные pip-зависимости
RUN pip3 install --default-timeout=300 --no-cache-dir --break-system-packages --ignore-installed \
    ultralytics \
    opencv-python
    
# Копируем рабочее пространство внутрь контейнера
WORKDIR /workspace
COPY . /workspace

# Собираем ROS 2 workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select figure_detector

# Устанавливаем путь до весов модели
ENV YOLO_MODEL_PATH=/workspace/runs/detect/train/weights/best.pt

# Source ROS 2 и запускаем launch файл
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.sh && source install/setup.sh && ros2 launch figure_detector figure_launch.py"]
