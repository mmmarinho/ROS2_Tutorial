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

# Adding other useful packages
sudo apt-get install -y tree