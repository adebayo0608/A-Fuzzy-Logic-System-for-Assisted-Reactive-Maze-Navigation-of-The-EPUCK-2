import serial
import time
import struct
import pygame
from os import system, name

from OA_Control import ObjectAvoidance

## SET COM NUMBER BELOW: UART
ser = serial.Serial('COM9', 115200, timeout=0)

COMMAND_PACKET_SIZE = 22
SENSORS_PACKET_SIZE = 103

# STARTING VALUES
left_speed = 0 								
right_speed = 0								
command = bytearray([0] * COMMAND_PACKET_SIZE)
sensors = bytearray([0] * SENSORS_PACKET_SIZE)
acc = [0 for x in range(6)]
gyro = [0 for x in range(6)]
magnetic_field = [0 for x in range(12)]
proximity = [0 for x in range(16)]
distance_cm = 0
mic_volume = [0 for x in range(8)]
left_steps = [0 for x in range(4)]
right_steps = [0 for x in range(4)]
battery_raw = 0
tv_remote_data = 0
selector = 0
button_state = 0
demo_state = 0
# Initializing Lists for the readings
proximity_readings = [[] for _ in range(8)]

def clear():
	# for windows 
	if name == 'nt': 
		_ = system('cls') 
	# for mac and linux(here, os.name is 'posix') 
	else: 
		_ = system('clear')   
  

# INITIALISATION
# Set initial values for actuators
command[0] = 0xF8;	# -0x08: get all sensors 
command[1] = 0xF7;	# -0x09: set all actuators
command[2] = 0;		# Settings: do not calibrate IR, disable onboard OA, set motors speed
command[3] = struct.unpack('<BB', struct.pack('<h', left_speed))[0]		# left motor LSB
command[4] = struct.unpack('<BB', struct.pack('<h', left_speed))[1]		# left motor MSB
command[5] = struct.unpack('<BB', struct.pack('<h', right_speed))[0]	# right motor LSB
command[6] = struct.unpack('<BB', struct.pack('<h', right_speed))[1]	# right motor MSB
command[7] = 0x00;	# lEDs
command[8] = 0;		# LED2 red
command[9] = 0;		# LED2 green
command[10] = 0;	# LED2 blue
command[11] = 0;	# LED4 red	
command[12] = 0;	# LED4 green
command[13] = 0;	# LED4 blue
command[14] = 0;	# LED6 red
command[15] = 0;	# LED6 green
command[16] = 0;	# LED6 blue
command[17] = 0;	# LED8 red
command[18] = 0;	# LED8 green
command[19] = 0;	# LED8 blue
command[20] = 0;	# speaker
command[21] = 0;	# End delimiter

start = time.time()

# Setting the initial states of the OA algorithm
curr_state = None
next_state = "USER_CONTROLLING"

while(1):	
	ser.write(command)									    # Execute set commands
	sensors = ser.read()									# Collect data from first sensor
	while len(sensors) < SENSORS_PACKET_SIZE:
		sensors += ser.read()								# Collect data from all sensors
	
	# Accelerometer
	acc[0] = struct.unpack("<h", struct.pack("<BB", sensors[0], sensors[1]))[0]
	acc[1] = struct.unpack("<h", struct.pack("<BB", sensors[2], sensors[3]))[0]
	acc[2] = struct.unpack("<h", struct.pack("<BB", sensors[4], sensors[5]))[0]
	
	# Gyro
	gyro[0] = struct.unpack("<h", struct.pack("<BB", sensors[18], sensors[19]))[0] #sensors[18] + sensors[19]*256
	gyro[1] = struct.unpack("<h", struct.pack("<BB", sensors[20], sensors[21]))[0] #sensors[20] + sensors[21]*256
	gyro[2] = struct.unpack("<h", struct.pack("<BB", sensors[22], sensors[23]))[0] #sensors[22] + sensors[23]*256	
	
	# Magnetometer
	magnetic_field[0] = struct.unpack("<f", struct.pack("<BBBB", sensors[24], sensors[25], sensors[26], sensors[27]))[0]
	magnetic_field[1] = struct.unpack("<f", struct.pack("<BBBB", sensors[28], sensors[29], sensors[30], sensors[31]))[0]
	magnetic_field[2] = struct.unpack("<f", struct.pack("<BBBB", sensors[32], sensors[33], sensors[34], sensors[35]))[0]
	
	# Proximity sensors
	proximity[0] = sensors[37] + sensors[38]*256
	proximity[1] = sensors[39] + sensors[40]*256
	proximity[2] = sensors[41] + sensors[42]*256
	proximity[3] = sensors[43] + sensors[44]*256
	proximity[4] = sensors[45] + sensors[46]*256
	proximity[5] = sensors[47] + sensors[48]*256
	proximity[6] = sensors[49] + sensors[50]*256
	proximity[7] = sensors[51] + sensors[52]*256
	
	# Microphone
	mic_volume[0] = sensors[71] + sensors[72]*256
	mic_volume[1] = sensors[73] + sensors[74]*256
	mic_volume[2] = sensors[75] + sensors[76]*256
	mic_volume[3] = sensors[77] + sensors[78]*256
	
	# Motors steps
	left_steps = struct.unpack("<h", struct.pack("<BB", sensors[79], sensors[80]))[0]
	right_steps = struct.unpack("<h", struct.pack("<BB", sensors[81], sensors[82]))[0]
	 
	# Battery
	battery_raw = sensors[83] + sensors[84]*256
	
	# TV remote
	tv_remote_data = sensors[88]
	
	# Selector
	selector = sensors[89]
	
	# Button
	button_state = sensors[102]
 
	
	# Extracting the proximity sensor readings as a list 
	proximity_sensors = [proximity[0], proximity[1], proximity[2], proximity[3], proximity[4], proximity[5], proximity[6], proximity[7]]
 
	# Extracting reading from the TOF sensor
	forward_distance_cm = (sensors[69] + sensors[70]*256)/10.0
	
	## Initialize the Pygame library for user input handling
	pygame.init()
 
	# Create an instance of the ObjectAvoidance class for controlling the robot's behavior
	obj_avoidance = ObjectAvoidance()
	
 
	# Specified number of sample readings for each sensor ()
	desired_readings = 1

	# Collection of multiple data ony in obstacle avoidance mode
	if next_state == "AVOIDING_OBSTACLE":
	 
		for i in range(8):
			# Clear the readings for this sensor
			proximity_readings[i] = []
			
			# Collect desired no. of readings 
			for _ in range(desired_readings):
		
				# Getting new sensor readings
				ser.write(command)
				sensors = ser.read()
				while len(sensors) < SENSORS_PACKET_SIZE:
					sensors += ser.read()
		
				# Storing new sensor readings
				proximity[0] = sensors[37] + sensors[38]*256
				proximity[1] = sensors[39] + sensors[40]*256
				proximity[2] = sensors[41] + sensors[42]*256
				proximity[3] = sensors[43] + sensors[44]*256
				proximity[4] = sensors[45] + sensors[46]*256
				proximity[5] = sensors[47] + sensors[48]*256
				proximity[6] = sensors[49] + sensors[50]*256
				proximity[7] = sensors[51] + sensors[52]*256
	
				# Appending sensor readings to the list of porximity readings   
				proximity_readings[i].append(proximity[i])		
	else:
		# Use the single proximity readings
		proximity_readings = proximity_sensors
	
	# Uncomment mode to run individualy
	#obj_avoidance.run_object_avoidance()
	#obj_avoidance.run_user_controlling(proximity_readings,forward_distance_cm)
	
	# Uncomment to run FSM control: combination of all states
	obj_avoidance.fsm_OA(proximity_readings,forward_distance_cm,next_state)	
	
	# Current state of the EPUCK
	curr_state = obj_avoidance.get_curr_state()
 
	# Next state of the EPUCK
	next_state = obj_avoidance.get_next_state()	
 
	#obj_avoidance.print_object_avoidance_variables()
	
	# Get the set speed from the program
	speed = obj_avoidance.get_velocity()
 
	# Get the set colour from the program
	led_array = obj_avoidance.get_led_colour()	
	
	# Set the speed 
	left_speed = int(speed[0])	
	right_speed = int(speed[1])
 
	# Set the commands using the set speed and colour
	command[3] = struct.unpack('<BB', struct.pack('<h', left_speed))[0]		# left motor LSB
	command[4] = struct.unpack('<BB', struct.pack('<h', left_speed))[1]		# left motor MSB
	command[5] = struct.unpack('<BB', struct.pack('<h', right_speed))[0]	# r, [2,5,2] ight motor LSB
	command[6] = struct.unpack('<BB', struct.pack('<h', right_speed))[1]	# right motor MSB
	command[7] = 0x00;			# lEDs
	command[8] = led_array[0]	# LED2 red
	command[9] = led_array[1]	# LED2 green
	command[10] = led_array[2]	# LED2 blue
	command[11] = led_array[3]	# LED4 red	
	command[12] = led_array[4]	# LED4 green
	command[13] = led_array[5]	# LED4 blue
	command[14] = led_array[6]	# LED6 red
	command[15] = led_array[7]	# LED6 green
	command[16] = led_array[8]	# LED6 blue
	command[17] = led_array[9]	# LED8 red
	command[18] = led_array[10]	# LED8 green
	command[19] = led_array[11]	# LED8 blue
 
	print("Next State: ", next_state)
	print("Current State: ", curr_state)
	print("LM Speed: " + str(left_speed))
	print("RM Speed: " + str(right_speed))	

	# Getting the next state of the algorithm	
	next_state = obj_avoidance.get_next_state()
 
 	# Clearing proximity readings
	if curr_state == "AVOIDING_OBSTACLE": 
		for i in range(8):
			proximity_readings[i].clear()
	
ser.close()

