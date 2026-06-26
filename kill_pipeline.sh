#!/bin/bash
pkill -9 -x gzserver 2>/dev/null
pkill -9 -x python3 2>/dev/null
pkill -9 -x ros2 2>/dev/null
killall -9 gzserver 2>/dev/null
echo "done"
