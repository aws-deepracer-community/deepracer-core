#!/usr/bin/env bash
source /root/venv/bin/activate; 
export XAUTHORITY=/root/.Xauthority
export DISPLAY=:0 # Select screen 0 by default.
xvfb-run -f $XAUTHORITY -l -n 0 -s ":0 -screen 0 1400x900x24" jwm &
x11vnc -bg -forever -nopw -rfbport 5700 -display WAIT$DISPLAY &
dashboard -d /root/coach_files -p 8000 &
sleep 999d
