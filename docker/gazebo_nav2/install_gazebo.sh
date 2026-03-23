#!/bin/bash
set -e

# Update and upgrade apt-get
sudo apt-get update
sudo apt-get upgrade -y

# https://gazebosim.org/docs/harmonic/install_ubuntu/
sudo apt-get install -y curl lsb-release gnupg

# Install gazebo harmonic (https://gazebosim.org/docs/harmonic/install/)
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic

## Install pairing ROS pairing libraries
sudo apt-get install -y ros-jazzy-ros-gz