#  BOLO JAIIIIIIIIIIIIIIIIIIIIII SHRI RAAAAAAM!!!


from dronekit import connect, VehicleMode, time as timed, LocationGlobal, LocationGlobalRelative
import numpy as np
import cv2
from pymavlink import mavutil
import argparse
import time
from imutils.video import VideoStream
import datetime
import imutils

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
# parser.add_argument('-p','--picamera', type=int, default=-1)
args = parser.parse_args()


print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)

vs = VideoStream(usePiCamera=True)
image_number = 0

def arm_and_takeoff(target_altitude):
	while not vehicle.is_armable:
		print "Waiting for the vehicle to initialize...."
		timed.sleep(1)

	print "Arming motors"
	# Copter should be in GUIDED mode for the
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:
		print "Waiting for the vehicle to arm...."
		timed.sleep(1)

	print "Taking off!"
	vehicle.simple_takeoff(target_altitude)

	while True:
		print "Altitude: ", vehicle.location.global_relative_frame.alt
		if vehicle.location.global_relative_frame.alt >= target_altitude*0.95:
			print "Reached target location"
			break
		timed.sleep(1)

def detectBarricadeHelper(image):     # Add the functionality for detecting whether move right or left
    #resizing the image
    img=cv2.resize(image,(500,300))
    #cv2 BGR2RGB
    img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    #cv2 RGB2HSV
    hsv_img=cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    #Gaussian_smoothing
    hsv_img_blur=cv2.GaussianBlur(hsv_img,(3,3),0)
    hsv_s=hsv_img_blur[:,:,1]
    #thresholding image on saturation
    ret,binary_hsv_global = cv2.threshold(hsv_s,200,255,cv2.THRESH_BINARY)
    kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    #dilating the image
    dilate_hsv = cv2.dilate(binary_hsv_global,kernel,iterations = 1)
    dilate_hsv_Gaussian=cv2.GaussianBlur(dilate_hsv,(3,3),0)
    #contouring to detect the rectangle of the barricade
    _,contours, hierarchy = cv2.findContours(dilate_hsv_Gaussian,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #max_area of the barricade
    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    cnt=contours[max_index]
    #bounding rectangle
    x,y,w,h = cv2.boundingRect(cnt)
    if(w>300):
		distance_parameter = 1177.5
        dist=distance_parameter/w
        cv2.imwrite('bbimage'+str(image_number)+'.jpeg',img)
        return 1,dist,0
    return 0,0,0

def detectBarricade(image):
	# caliberate cv2.VideoCapture with Picamera
	# camera = cv2.VideoCapture("./sololink.sdp")
	# _,capture = camera.read()

	det,dist,direc = detectBarricadeHelper(image)
	return (det,dist,direc)

def get_location_metres(orignal_location, dNorth, dEast):
	earth_radius = 6378137.0
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*orignal_location.lat/180))
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	if type(original_location) is LocationGlobal:
	    targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
	elif type(original_location) is LocationGlobalRelative:
	    targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
	else:
	    raise Exception("Invalid Location object passed")
	return targetlocation;

def get_distance_meters(aLocation1, aLocation2):
	dLat = aLocation2.lat - aLocation1.lat
	dLon = aLocation2.lon - aLocation1.lon
	return math.sqrt(dLat**2 + dLon**2)*1.113195e5


def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
	currentLocation = vehicle.location.global_relative_frame
	targetLocation = get_location_metres(currentLocation, dNorth, dEast)
	targetDistance = get_distance_meters(currentLocation, targetLocation)
	gotoFunction(targetLocation)

	while vehicle.mode == "GUIDED":
		frame = vs.read()
		cv2.imwrite('detected_image'+str(image_number)+'.jpeg',frame)
		det,dist,direc = detectBarricade(frame)
		remainingDistance = get_distance_meters(vehicle.location.global_relative_frame, targetlocation)
		image_number = image_number+1
		print "Distance to the target : ", remainingDistance
		if remainingDistance<=targetlocation*0.01 :
			print "Reached the target"
			break
		time.sleep(0.5)
	time.sleep(2)


vehicle.groundspeed = 0.5
arm_and_takeoff(1.5)
goto(2,0)
goto(0,2)
