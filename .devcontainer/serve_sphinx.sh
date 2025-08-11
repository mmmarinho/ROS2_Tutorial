#!/bin/bash
set -e

# This script is expected to be run at the root of the repository.
cd docs/source
# sphinx-autobuild in general should be run in this directory or the mermaid configuration will not be found.
sphinx-autobuild . build/html