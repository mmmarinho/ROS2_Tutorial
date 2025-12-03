#!/bin/bash
set -e

# This is expected to be run at the root of the repository
pwd
export PIP_BREAK_SYSTEM_PACKAGES=1
pip install -r docs/source/requirements.txt
pip install sphinx-autobuild
chmod +x .devcontainer/serve_sphinx.sh

# Making mermaid graphs work
sudo apt-get update
sudo apt-get install -y npm
npm install -g @mermaid-js/mermaid-cli
npm install -g @iconify-json/logos@1
npm install -g @iconify-json/material-symbols

# Adding other useful packages
sudo apt-get install -y tree
pip install nottf2

# Install gazebo
curl -OL https://raw.githubusercontent.com/UoMMScRobotics/sfr_gazebo_nav2/refs/heads/main/install_gazebo.sh
chmod +x install_gazebo.sh
. install_gazebo.sh

# Install nav2
curl -OL https://raw.githubusercontent.com/UoMMScRobotics/sfr_gazebo_nav2/refs/heads/main/install_nav2.sh
chmod +x install_nav2.sh
. install_nav2.sh
