#!/bin/sh
sudo ip link set can0 down
sudo ip link set can0 type can restart-ms 100 bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
echo "CAN port 0 has been reset"

