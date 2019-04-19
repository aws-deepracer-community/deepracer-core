export DISPLAY=${DISPLAY:-:0} # Select screen 0 by default.
export XAUTHORITY=/root/.Xauthority
source /opt/ros/kinetic/setup.bash
source install/setup.sh
xvfb-run -f $XAUTHORITY -l -n 0 -s "-screen 0 1400x900x24" roslaunch deepracer_simulation distributed_training.launch &
sleep 1
if which x11vnc &>/dev/null; then
  ! pgrep -a x11vnc && x11vnc -bg -forever -nopw -quiet -display WAIT$DISPLAY &
fi
#! pgrep -a Xvfb && Xvfb $DISPLAY -screen 0 1024x768x16 &
sleep 1
#if which fluxbox &>/dev/null; then
#  ! pgrep -a fluxbox && fluxbox &
#fi
echo "IP: $(hostname -I) ($(hostname))"
wait
