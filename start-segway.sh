#!/bin/bash
gnome-terminal -- bash -c "systemctl --user restart joystick_segway; systemctl --user status joystick_segway"
