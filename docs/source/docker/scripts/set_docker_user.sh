#!/bin/bash
set -e
# https://docs.docker.com/engine/install/linux-postinstall/
# If there are errors with docker.json
# sudo chown "$USER":"$USER" /home/"$USER"/.docker -R
# sudo chmod g+rwx "$HOME/.docker" -R

echo "Working with user $1."
if id "$1" >/dev/null 2>&1; then
    echo "User $1 found."
else
    echo "User $1 NOT found, exiting..."
    exit 1
fi

USER_TO_ADD="$1"

# Added the -f so that it does not fail if the group already exists
sudo groupadd -f docker
# Add the user in the argument
sudo usermod -aG docker "${USER_TO_ADD}"
# Activate changes in the group
newgrp docker
# Check if it works without sudo
docker run hello-world