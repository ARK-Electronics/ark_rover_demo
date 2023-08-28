#!/bin/bash

sudo systemctl stop rover.service
sudo systemctl disable rover.service
sudo cp rover.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rover
sudo systemctl start rover
