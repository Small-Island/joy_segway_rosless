#!/bin/bash
gnome-terminal -- bash -c "systemctl --user start joystick_segway; systemctl --user status joystick_segway; cd ${HOME}/segway; bash; source ./devel/setup.bash"
