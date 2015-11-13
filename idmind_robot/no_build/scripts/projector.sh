#!/bin/bash

dmode="$(cat /sys/class/drm/card0-HDMI-A-2/status)"

export DISPLAY=:0
export XAUTHORITY=/home/viva/.Xauthority

if [ "${dmode}" = disconnected ]; then
	xrandr --output HDMI2 --off
	xrandr --output HDMI1 --mode 1280x720 --set audio force-dvi
	xrandr -o left

elif [ "${dmode}" = connected ]; then
	xrandr -o normal
	xrandr --output HDMI1 --off
	xrandr --output HDMI2 --auto
fi

exit 0
