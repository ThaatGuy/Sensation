from board import SCL, SDA
import busio
import time
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685


class Motor():

	def __init__(self):
		# Create the I2C bus interface.
		self.i2c_bus = busio.I2C(SCL, SDA)

		# Create a simple PCA9685 class instance.
		self.pca = PCA9685(self.i2c_bus)

		# Set the PWM frequency to 60hz.
		self.pca.frequency = 60
		self.timer = 0

		#define which channel belongs to which motor
		self.leftMotor = self.pca.channels[7]
		self.rightMotor = self.pca.channels[5]
		self.centerMotor = self.pca.channels[6]
		print('motor')

	def engageMotor(self, command, duty=100):

		# conveet duty to be a number between 0 and 15
		dutyLevel = int(15*duty/100)

		dutyCommand = dutyLevel*4095
		print(dutyCommand)

		off = 0x0FFF
		
		if 'right' in command:
			self.rightMotor.duty_cycle = dutyCommand
			print('right')
		if 'left' in command:
			self.leftMotor.duty_cycle = dutyCommand
			print('left')
		if 'straight' in command:
			self.centerMotor.duty_cycle = dutyCommand

		if 'u-turn' in command:
			self.rightMotor.duty_cycle = dutyCommand
			print('right')
			time.sleep(0.5)
			self.rightMotor.duty_cycle = off
			self.centerMotor.duty_cycle = dutyCommand
			print('center')
			time.sleep(0.5)
			self.centerMotor.duty_cycle = off
			self.leftMotor.duty_cycle = dutyCommand
			print('left')
			time.sleep(0.5)
			self.leftMotor.duty_cycle = off

		if 'goal' in command:
			for i in range (3):
				self.rightMotor.duty_cycle = dutyCommand
				self.centerMotor.duty_cycle = dutyCommand
				self.leftMotor.duty_cycle = dutyCommand
				time.sleep(1)
				self.rightMotor.duty_cycle = off
				self.centerMotor.duty_cycle = off
				self.leftMotor.duty_cycle = off
				time.sleep(1)

		if 'error' in command:
			for i in range(5):
				self.rightMotor.duty_cycle = dutyCommand
				time.sleep(1)
				self.rightMotor.duty_cycle = off
				self.leftMotor.duty_cycle = dutyCommand
				time.sleep(1)
				self.leftMotor.duty_cycle = off

		if 'start' in command:
			for i in range(2):
				self.rightMotor.duty_cycle = dutyCommand
				time.sleep(1)
				self.rightMotor.duty_cycle = off
				self.leftMotor.duty_cycle = dutyCommand
				time.sleep(1)
				self.leftMotor.duty_cycle = off
		return


	def disengageMotor(self, command):
		off = 0x0FFF
		if 'right' in command:
			self.rightMotor.duty_cycle = off
		if 'left' in command:
			self.leftMotor.duty_cycle = off
		else:
			self.centerMotor.duty_cycle = off
		return

	def disengageAllMotors(self):
		off = 0x0FFF
		self.rightMotor.duty_cycle = off
		self.leftMotor.duty_cycle = off
		self.centerMotor.duty_cycle = off
		return