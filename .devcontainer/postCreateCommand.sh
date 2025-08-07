#!/bin/bash
set -e

# This is expected to be run at the root of the repository
pwd
export PIP_BREAK_SYSTEM_PACKAGES=1
pip install -r docs/source/requirements.txt
pip install sphinx-autobuild
chmod +x .devcontainer/serve_sphinx.sh