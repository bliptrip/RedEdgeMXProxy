#!/bin/sh

sudo pip3 install .
sudo cp systemd/rededgemxproxy.service /etc/systemd/system/
sudo systemctl enable rededgemxproxy.service #Enable on RedEdgeMX proxy service on system boot
