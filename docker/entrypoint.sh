#!/bin/bash
# Container main process; interactive and dev.sh shells get the ROS env from /etc/fxteso.bashrc.
set +u
source /etc/fxteso.bashrc
exec "$@"
