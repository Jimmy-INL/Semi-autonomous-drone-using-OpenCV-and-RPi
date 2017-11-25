from dronekit import connect, VehicleMode, time, LocationGlobal, LocationGlobalRelative
import numpy as np
import cv2
from pymavlink import mavutil
import argparse  
from imutils.video import VideoStream
import datetime
import imutils

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
# parser.add_argument('-p','--picamera', type=int, default=-1)
args = parser.parse_args()


print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)

def arm_and_takeoff(target_altitude):
	while not vehicle.is_armable:
		print "Waiting for the vehicle to initialize...."
		time.sleep(1)

	print "Arming motors"
	# Copter should be in GUIDED mode for the 
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:
		print "Waiting for the vehicle to arm...."
		time.sleep(1)

	print "Taking off!"
	vehicle.simple_takeoff(target_altitude)

	while True:
		print "Altitude: ", vehicle.location.global_relative_frame.alt
		if vehicle.location.global_relative_frame.alt >= target_altitude*0.95:
			print "Reached target location"
			break
		time.sleep(1)

arm_and_takeoff(2)
