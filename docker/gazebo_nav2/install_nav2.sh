#!/bin/bash
set -e

# Update and upgrade apt-get
sudo apt-get update
sudo apt-get upgrade -y

# Install nav2 (https://docs.nav2.org/getting_started/index.html)
sudo apt-get install -y ros-jazzy-navigation2
sudo apt-get install -y ros-jazzy-nav2-bringup
sudo apt-get install -y ros-jazzy-nav2-minimal-tb*

# (https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
sudo apt-get install -y ros-jazzy-slam-toolbox
