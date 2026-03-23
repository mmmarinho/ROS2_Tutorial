FROM ghcr.io/uommscrobotics/sfr_ros2:latest
LABEL authors="Murilo Marinho"

ENTRYPOINT ["/bin/bash", "-c"]

RUN wget https://raw.githubusercontent.com/mmmarinho/ROS2_Tutorial/refs/heads/main/docker/gazebo_nav2/install_gazebo.sh
RUN chmod +x install_gazebo.sh && bash -c "./install_gazebo.sh" && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*
RUN wget https://raw.githubusercontent.com/mmmarinho/ROS2_Tutorial/refs/heads/main/docker/gazebo_nav2/install_nav2.sh
RUN chmod +x install_nav2.sh && bash -c "./install_nav2.sh" && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*
RUN wget https://raw.githubusercontent.com/mmmarinho/ROS2_Tutorial/refs/heads/main/docker/gazebo_nav2/install_age.sh
RUN chmod +x install_age.sh && bash -c "./install_age.sh" && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*
