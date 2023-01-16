#!/bin/bash

cd ~/tactile
./momo.run

sleep 2s

./main &

cd ~/joy_segway_rosless
# ./sora_arm64.run
#/usr/bin/chromium-browser http://127.0.0.1:8080/html/test.html &

if [ $(arch) = 'aarch64' ]; then # jetson xavier NX
    ./momo.run
    ~/ojima-librealsense/build/examples/pointcloud/ojima-pointcloud &
    ./main
elif [ $(arch) = 'armv7l' ]; then # Raspberry pi
    ./momo.run
    ./main_ras
fi
# python3 ./src/mywork/obstacleDetection.py &
