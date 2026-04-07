#!/bin/bash
set -e

pip3 install -r requirements.txt
pip3 install -r requirements_dev.txt

apt-get update
rosdep update
rosdep install --from-paths . --ignore-src -r -y