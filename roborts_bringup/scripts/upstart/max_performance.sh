#!/bin/bash
sleep 10
sudo jetson_clocks
echo "Max performance"
sudo iw dev wlan0 set power_save off
echo "Set power_save off"
