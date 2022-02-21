#!/bin/bash
cd ~/joy_segway_rosless
# ./sora_arm64.run
#/usr/bin/chromium-browser http://127.0.0.1:8080/html/test.html &
./momo_arm64.run --no-video-device
cd segway_rmp/build
./segway_rmp_node
# python3 ./src/mywork/obstacleDetection.py &
