import time
import board
import adafruit_gps
import serial

class GPS():

	def __init__(self):
		uart = serial.Serial("/dev/GPS", baudrate=9600, timeout=10)

		# Create a GPS module instance.
		self.gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

		# Turn on everything (not all of it is parsed!)
		self.gps.send_command(b"PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0")

		# Set update rate to 5 times a second (5hz).
		self.gps.send_command(b"PMTK220,5000")

		#self.getLocation = None

	def getLocation(self, speech):
		
		noSignalmsgPause = time.monotonic()
		while True:

			current = time.monotonic()
			self.gps.update()

			if not self.gps.update() or not self.gps.has_fix:
				time.sleep(0.1)
				print(current - noSignalmsgPause)
				if current - noSignalmsgPause >=.5:
					speech.say('n')
					speech.runAndWait()
					noSignalmsgPause = current
					return 0.0, 0.0 
				continue
					
			if self.gps.nmea_sentence[3:6] == "GSA":
				longi = self.gps.longitude
				lati = self.gps.latitude

				noSignalmsgPause = current

				print(f"{self.gps.latitude:.6f}, {self.gps.longitude:.6f} {self.gps.altitude_m}m")

				return longi, lati
	