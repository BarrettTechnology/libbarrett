#!/bin/bash
sudo ip link set can0 up type can restart-ms 100 bitrate 1000000
echo "CAN port 0 has been reset"
