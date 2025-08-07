#!/bin/bash
set -e

# This is expected to be run at the root of the repositoryhttps://raw.githubusercontent.com/UoMMScRobotics/SFR_Gazebo/refs/heads/main/install_gazebo.sh
pwd
export PIP_BREAK_SYSTEM_PACKAGES=1
pip install -r docs/source/requirements.txt
pip install sphinx-autobuild
chmod +x serve_sphinx.sh