# Базовый образ ROS 2 Jazzy
FROM ros:jazzy

# Установка зависимостей
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-usb-cam \
    ros-jazzy-image-transport \
    libgl1 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

RUN apt remove -y python3-numpy

ENV PIP_BREAK_SYSTEM_PACKAGES=1
# Установка Python-зависимостей
RUN pip3 install --no-cache-dir \
    "numpy<2.0" \
    ultralytics \
    opencv-python

# Создание рабочего каталога
WORKDIR /workspace
RUN mkdir -p video_data/video
COPY . /workspace/ros2_ws/src/figure_detector
# Сборка пакетов
RUN . /opt/ros/jazzy/setup.sh && \
    cd ros2_ws && \
    colcon build

# Исходная среда
SHELL ["/bin/bash", "-c"]

# Загрузка окружения ROS и установка рабочей директории
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

# Запуск по умолчанию
CMD ["bash"]
