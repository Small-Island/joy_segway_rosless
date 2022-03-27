#!/bin/bash
cd ~/joy_segway_rosless
# ./sora_arm64.run
#/usr/bin/chromium-browser http://127.0.0.1:8080/html/test.html &

if [ $(arch) = 'aarch64' ]; then # jetson xavier NX
    ./momo.run
    ./main &
    ~/ojima-librealsense/build/examples/pointcloud/ojima-pointcloud
elif [ $(arch) = 'armv7l' ]; then # Raspberry pi
    ./momo.run
    ./main
fi
# python3 ./src/mywork/obstacleDetection.py &
