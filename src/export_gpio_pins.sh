#!/usr/bin/env bash

### BEGIN INIT INFO
# Provides:          gpio_setup
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: exports gpio pins for 3 sonars.
# Description:       exports gpio pins for 3 sonars.
### END INIT INFO

case "$1" in
    start)
	# to run as non run using wiringPiSetupSys we
	# need to export the gpio pins.
	# see http://wiringpi.com/reference/setup/
	gpio export 24 out
	gpio export 25 in
	gpio export 22 out
	gpio export 23 in
	gpio export 18 out
	gpio export 27 in
	;;
    stop)
	gpio unexportall
	;;
    *)
	echo "unknown command"
	exit 1
	;;
esac

