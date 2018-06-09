#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in
# adapted for A4WD2 usage by Lukas Solanka

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# ensure to not use different shell type which was set before
CATKIN_SHELL=sh

# Determine IP address of wlan0. This is the default interface to currently run the ROS
# on.
IFC=wlan0
IP_OUTPUT=$(ip -4 addr show dev $IFC)
if [ $? -ne 0 ]; then
    echo "error: cannot get information about $IFC."
    exit 1
fi
THIS_IP=$(echo "$IP_OUTPUT" | grep scope | grep -o --regexp 'inet [0-9\.]*' | cut -d' ' -f 2)
if [ -z "$THIS_IP" ]; then
    echo "error: IP of $IFC is empty. Looks like interface is not configured"
    exit 1
fi

export ROS_IP=$THIS_IP
export ROS_MASTER_URI=http://$THIS_IP:11311

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(cd "`dirname "$0"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"
exec "$@"
