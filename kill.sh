#!/bin/bash

if [[ $(ps | grep momo_x64) ]]; then
    killall momo_x64
fi

if [[ $(ps | grep momo_raspberry) ]]; then
    killall momo_raspberry
fi

if [[ $(ps | grep momo_xavier) ]]; then
    killall momo_xavier
fi

if [[ $(ps | grep socat) ]]; then
    killall socat
fi

if [[ $(ps | grep segway_rmp_node) ]]; then
    killall segway_rmp_node
fi

if [[ $(ps | grep sora_x64.run) ]]; then
    killall sora_x64.run
fi

if [[ $(ps | grep sora_arm64.run) ]]; then
    killall sora_arm64.run
fi
