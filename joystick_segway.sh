#!/bin/bash
cd ~/joy_segway_rosless
# ./sora_arm64.run
#/usr/bin/chromium-browser http://127.0.0.1:8080/html/test.html &

if [ $(arch) = 'aarch64' ]; then
    ./momo.run
    ./without_SegwayRMPNode &
    ~/ojima-librealsense/build/examples/pointcloud/ojima-pointcloud
elif [ $(arch) = 'armv7l' ]; then
    ./momo_arm64.run
    ./without_SegwayRMPNode
fi
# python3 ./src/mywork/obstacleDetection.py &
