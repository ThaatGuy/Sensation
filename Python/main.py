#!/usr/bin/python3

import jetson.inference
import jetson.utils

import cv2 #image processing
import argparse
import sys
import Motor #control motors
import time
import GPS
import openrouteservice as ors #navigation
import pyttsx3 # text to speech output
import geopy.distance
import csv #logging

import Jetson.GPIO as gpio # interface jetson nanos gpio
import subprocess, os
import signal

from segnet_utils import * #utilities for segmentation


# dictionary of instructions provided by ORS
instrDict = {
    0 : 'left',
    1 : 'right',
    2 : 'sharp left',
    3 : 'sharp right',
    4 : 'straight left',
    5 : 'straight right',
    6 : 'straight',
    7 : 'enter roundabout',
    8 : 'exit roundabout',
    9 : 'u-turn',
    10 : 'goal',
    11 : 'depart',
    12 : 'KL',
    13 : 'KR'
}

# dictionary for different way types provided by ORS
wayTypeDict = {
    0 : 'unknown',
    1 : 'state road',
    2 : 'road',
    3 : 'street',
    4 : 'path',
    5 : 'track',
    6 : 'cycleway',
    7 : 'footway', 
    8 : 'steps',
    9 : 'ferry',
    10 : 'construction'
}

# set up the vibration motors
motor = Motor.Motor()

# turn all motors off
motor.disengageAllMotors()

# initialize the text to speech
speech = pyttsx3.init()
# initialize the gps
gps = GPS.GPS()

# assign inpit pin for startup button
pinIn = 'DAP4_SCLK'

# indicate if detection/navigation system is running
start = 0

# parse the command line
parser = argparse.ArgumentParser(description="Segment a live camera stream using an semantic segmentation DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.segNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="/dev/video0 ", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="savedVideo.mp4", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="fcn-resnet18-cityscapes", help="pre-trained model to load, see below for options")
parser.add_argument("--filter-mode", type=str, default="linear", choices=["point", "linear"], help="filtering mode used during visualization, options are:\n  'point' or 'linear' (default: 'linear')")
parser.add_argument("--visualize", type=str, default="overlay,mask", help="Visualization options (can be 'overlay' 'mask' 'overlay,mask'")
parser.add_argument("--ignore-class", type=str, default="void", help="optional name of class to ignore in the visualization results (default: 'void')")
parser.add_argument("--alpha", type=float, default=150.0, help="alpha blending value to use during overlay, between 0.0 and 255.0 (default: 150.0)")
parser.add_argument("--stats", action="store_true", help="compute statistics about segmentation mask class output")

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# define target longitude and latitude info
targetLongi = 11.0086529
targetLati = 49.596501

# load the segmentation network
net = jetson.inference.segNet(opt.network, sys.argv)
net1 = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

# set the alpha blending value
net.SetOverlayAlpha(opt.alpha)

# create video output
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)

# create buffer manager
buffers = segmentationBuffers(net, opt)

# create video source
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)

# work with the raw classification grid dimensions
grid_width, grid_height = net.GetGridSize()	
num_classes = net.GetNumClasses()

# allocate a single-channel uint8 image for the class mask
class_mask = jetson.utils.cudaAllocMapped(width=grid_width, height=grid_height, format="gray8")

# setup gpio
try:
	# Set pin as input pin
	gpio.setup(pinIn, gpio.IN)

	# wait for button press event
	while True:
		if not 'event' in locals():
			event = gpio.add_event_detect(pinIn, gpio.RISING, bouncetime=1000)
			print(start)

		# if button press has been detected...
		else:
			# if butten has been pressed and programme is not running start program
			if gpio.event_detected(pinIn) and start == 0:
				start = 1
				# initialize
				prevTime = time.monotonic()
				prevInstruction = ''
				# indicate device is running at 100% duty
				motor.engageMotor('left', 100)
				time.sleep(2)
				motor.disengageAllMotors()
				motor.engageMotor('straight', 100)
				time.sleep(2)
				motor.disengageAllMotors()
				motor.engageMotor('right', 100)
				time.sleep(2)
				motor.disengageAllMotors()

				# initialize
				prevTime = time.monotonic()
				prevInstruction = ''
				motor.disengageAllMotors()

				longi, lati = 0.0, 0.0
				# open file for location logging
				with open("data.csv","w") as f:
					gpsLogger = csv.writer(f,delimiter=",")
					# determing full route info
					longi, lati = gps.getLocation(speech)
					print(longi)
					if longi != 0:
						coordinates = [[longi, lati], [targetLongi, targetLati]]
						gpsLogger.writerow([lati,longi])

						try:
					        # get directions
							client = ors.Client(key='5b3ce3597851110001cf6248c2ecac6a05ad444bb520fd5f33245545') # Specify your personal API key
							route = client.directions(coordinates=coordinates,
							                          profile='foot-walking',
							                          extra_info = ['steepness', 'waytype'],
							                          format='geojson')
							completeWaytype= list(route['features'][0]['properties']['extras']['waytypes']['values'])
							completeRoute = list(route['features'][0]['properties']['segments'][0]['steps'])
							completeGeometry = enumerate(route['features'][0]['geometry']['coordinates'])

							print(len(completeRoute))
							print(completeRoute)

							for instruction in completeRoute:
								print(instruction)
								print(instruction['instruction'],instruction['distance'], instruction['type'])
								speech.say(instruction['instruction']) 
								speech.runAndWait()
								speech.say(' walk ')
								speech.runAndWait()
								speech.say(instruction['distance'])
								speech.runAndWait()
								speech.say(' meters ')
								speech.runAndWait()

								# determine the way type
								for wayType in completeWaytype:
									if wayType[0] <= instruction['way_points'][0] and wayType[1]>= instruction['way_points'][1]:
										print(wayTypeDict[wayType[2]])
										speech.say(' on the ')
										speech.runAndWait()
										speech.say(wayTypeDict[wayType[2]])
										speech.runAndWait()

								motor.engageMotor(instrDict[instruction['type']])

								motor.engageMotor(instrDict[instruction['type']])
								time.sleep(2)
								motor.disengageMotor(instrDict[instruction['type']])

						except:
							print('can not request for directions')
							speech.say(' can not request for directions ')
							speech.runAndWait()
							motor.engageMotor('error')


						speech.say(' starting now ')
						speech.runAndWait()
						prevLongi, prevLati = longi, lati
						prevTime = time.monotonic()
						prevInstrCount  = 0

						prevReqTime = time.monotonic()
					while True:

						# direct user to avoid obsticles

						# capture the next image
						img_input = input.Capture()
						detections = net1.Detect(img_input)

						jetson.utils.cudaDeviceSynchronize ()

						# convert the frame to numpy
						frame = jetson.utils.cudaToNumpy(img_input)

						frameWidth = frame.shape[1]
						frameHeight = frame.shape[0]

						# key points for rectangle
						rectBottomHeight = frameHeight - frameHeight/9
						rectTopHeight = frameHeight- frameHeight/7
						topLeftWidth = int((frameWidth/2) - frameWidth/20 -(frameWidth/30))
						topRightWidth = int((frameWidth/2) + frameWidth/20 +(frameWidth/30))
						rectHeight = int(frameHeight - frameHeight/4)

						centerWidthRight = int(frameWidth/2 + (frameWidth/30))
						centerWidthLeft = int(frameWidth/2 - (frameWidth/30))

						# color (RGB)
						red = (255, 0, 0)
						green = (0, 255, 0)
						blue = (0, 0, 255)
						black = (0, 0, 0)

						# draw the rectangles on the frame indicating where we are sensing for obstructions
						cv2.rectangle(frame, (topLeftWidth, rectHeight), (centerWidthLeft, frameHeight), red)
						cv2.rectangle(frame, (centerWidthRight, rectHeight), (topRightWidth, frameHeight), blue)

						# if the objects detected by mobilnet overlap the rectangular region initiate the correct motor so the user can avoid the object
						motorCommandList = []
						for detection in detections:
							print(detection)
							left, top, right, bottom = detection.Left, detection.Top, detection.Right, detection.Bottom
							print(left, top, right, bottom)
							# check if the bounding box overlaps with the right rectangle
							if (left >= topRightWidth) or (right <= centerWidthRight) or (bottom <= rectHeight) or (top >= frameHeight):
								motor.disengageAllMotors()
							else:
								print('**************************************overlap*************************************************')
								motorCommandList.append('left straight')
								#Motor.Motor.engageMotor(motor, 'left straight', 100)
								#time.sleep(1)

						# check if the bounding box overlaps with the left rectangle
							if (left >= centerWidthLeft) or (right <= topLeftWidth) or (bottom <= rectHeight) or (top >= frameHeight):
								motor.disengageAllMotors()
							else:
								print('**************************************overlap*************************************************')
								#Motor.Motor.engageMotor(motor, 'right straight', 100)
								motorCommandList.append('right straight')
								#time.sleep(1)

						# engage the motors needing to be engaged
						motor.engageMotor(' '.join(motorCommandList), 100)

						# convert image back to cuda
						img_input = jetson.utils.cudaFromNumpy(frame)
						# allocate buffers for this size image
						buffers.Alloc(img_input.shape, img_input.format)

						# process the segmentation network
						net.Process(img_input, ignore_class=opt.ignore_class)

						# generate the overlay
						if buffers.overlay:
							net.Overlay(buffers.overlay, filter_mode=opt.filter_mode)

						# generate the mask
						if buffers.mask:
							net.Mask(buffers.mask, filter_mode=opt.filter_mode)

						# composite the images
						if buffers.composite:
							jetson.utils.cudaOverlay(buffers.overlay, buffers.composite, 0, 0)
							#jetson.utils.cudaOverlay(buffers.mask, buffers.composite, buffers.overlay.width, 0)

						# render the output image
						output.Render(buffers.output)

						# update the title bar
						output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

						# print out performance info
						jetson.utils.cudaDeviceSynchronize()
						net.PrintProfilerTimes()

					    # compute segmentation class stats
						if opt.stats:
							buffers.ComputeStats()

						# direct the user through a route
						# get current time
						currTime = time.monotonic()

						# get current longitude and latitude info
						longi, lati = gps.getLocation(speech)

						motor.disengageAllMotors()
						if longi !=0:
							coordinates = [[longi, lati], [targetLongi, targetLati]]
							gpsLogger.writerow([lati,longi])
							# find the change in distance between 2 readings in meters
							distChange = geopy.distance.geodesic((prevLongi, prevLati),(longi, lati)).m
							print(distChange)
							# if distance change is less than 20m within 3 seconds consider it as a valid reading else take another reading
							if (6<currTime - prevTime >= 3) and (distChange < 20):
								try:
									# get directions
									route = client.directions(coordinates=coordinates,
									                          profile='foot-walking',
									                          extra_info = ['steepness', 'waytype'],
									                          format='geojson')
									newActions = list(route['features'][0]['properties']['segments'][0]['steps'])

									# get a set of key coordinates
									newGeometry = list(route['features'][0]['geometry']['coordinates'])



						            # if user is 10m away from next point of interest indicate next action
						            # distToNextAction = newActions[0]['distance']
						            # get user to perform the required action 10m beforehand
									distToNextAction = newActions[0]['distance']

									print('prev count = ' + str(prevInstrCount) + ', current action count = ' + str(len(newActions)))
									# if we have less instructions than before then consider the action complete and turn motors off
									if len(newActions) < prevInstrCount:
										motor.disengageAllMotors()
										print('disengaging motors')
										prevInstrCount = len(newActions)

									print('distToNextAction' + str(distToNextAction))
									# if the distance to the next action is less than 10 indicate to the user using motors and speech
									if distToNextAction < 10:
										nextInstruction = newActions[1]['type']
										speech.say(instrDict[nextInstruction])
										speech.runAndWait()
										motor.engageMotor(instrDict[nextInstruction])
										prevInstrCount = len(newActions)

									prevTime = currTime
									prevLongi, prevLati = longi, lati

									if (currTime - prevTime > 6) and (distChange > 20):
										motor.engageMotor('right left')
										time.sleep(1)
										motor.disengageMotor('right left')
										prevTime = currTime
										prevLongi, prevLati = longi, lati

								except:
									print('can not request for directions')
									#speech.say(' can not request for directions ')
									#speech.runAndWait()
									#Motor.Motor.engageMotor(motor, 'error')
					
						if gpio.event_detected(pinIn) and start == 1:
							start = 0
							time.sleep(1)
							motor.disengageAllMotors()
							
							break

						# exit on input/output EOS
						if not input.IsStreaming() or not output.IsStreaming():
							motor.disengageAllMotors()
							break

finally:  
	gpio.cleanup()