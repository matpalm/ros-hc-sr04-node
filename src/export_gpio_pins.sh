#!/usr/bin/env bash
# to run as non run using wiringPiSetupSys we 
# need to export the gpio pins. 
# see http://wiringpi.com/reference/setup/
gpio export 24 out
gpio export 25 in
gpio export 22 out
gpio export 23 in
gpio export 18 out
gpio export 27 in
gpio exports
