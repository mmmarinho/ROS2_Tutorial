FROM ubuntu:24.04
LABEL authors="Murilo Marinho"

ENTRYPOINT ["/bin/bash", "-c"]

RUN apt-get update && apt-get install sudo -y && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*
RUN sudo apt-get update && sudo apt-get install wget -y && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*
RUN wget https://raw.githubusercontent.com/mmmarinho/ROS2_Tutorial/refs/heads/main/docker/ros2/install_ros2.sh && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*
RUN chmod +x install_ros2.sh && bash -c "./install_ros2.sh"
