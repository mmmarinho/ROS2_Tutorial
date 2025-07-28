#!/bin/bash
set -e

export PIP_BREAK_SYSTEM_PACKAGES=1
pip install -r ../docs/source/requirements.txt
pip install sphinx-autobuild
chmod +x serve_sphinx.sh