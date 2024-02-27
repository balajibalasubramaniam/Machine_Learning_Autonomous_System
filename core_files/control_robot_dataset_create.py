# Machine Learning Autonomous System
# Author: Balaji Balasubramaniam
# Date: February 26, 2024

import sys
import time
import sim
from time import sleep as delay
from sshkeyboard import listen_keyboard

print("Program Started")

# Connect to CoppeliaSim remote API. 
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

# Prints out a successful/unsuccessful connection message
if (clientID != -1):
    print('Connected Successfully.')
else:
    sys.exit('Failed To connect.')

delay(1)

# Get object handles
error_code, pioneer_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_oneshot_wait)
error_code, left_motor_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_oneshot_wait)
error_code, right_motor_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_oneshot_wait)
error_code, left_proximity_sensor = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/left_sensor', sim.simx_opmode_oneshot_wait)
error_code, right_proximity_sensor = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/right_sensor', sim.simx_opmode_oneshot_wait)

# The below variable will be used to determine the location of the robot
position=0
explored_location = {"X : 0 - Y : 0","X : 100 - Y : 100"}

#Initial motor speed
lSpeed = 0
rSpeed = 0

# Analyse keyboard input and set the velocity
def press(user_key_press): 

	#Read sensor data
	data_left_sensor = sim.simxReadProximitySensor(clientID, left_proximity_sensor, sim.simx_opmode_streaming)
	data_right_sensor = sim.simxReadProximitySensor(clientID, right_proximity_sensor, sim.simx_opmode_streaming)
	
	data_pioneer = sim.simxGetObjectPosition(clientID, pioneer_handle, position, sim.simx_opmode_streaming)
	current_position = "X : "+ str(round(data_pioneer[1][0],1))+" - Y : "+ str(round(data_pioneer[1][1],1))
	
	# Robot location in X, Y position
	if current_position not in explored_location:
		explored_location.add(current_position)
		print("current_position: ",current_position)
	
	# If the wall is too far from the proximity sensor's detection range then assume there is no obstacle
	if(data_left_sensor[1]):
		left_obstacle_distance  = data_left_sensor[2][2]
	else:
		left_obstacle_distance = 10; # No obstacle for 10m
	
	if(data_right_sensor[1]):
		right_obstacle_distance = data_right_sensor[2][2]
	else:
		right_obstacle_distance = 10; # No obstacle for 10m
			
	# Analyse keyboard input and set the velocity
	if (user_key_press == 'w'): # Forward
		print('loop: w')
		lSpeed = 0.5
		rSpeed = 0.5
		message_data = str(round(left_obstacle_distance,2)) + "," + str(round(right_obstacle_distance,2)) + "," +  str(0) + "\n"
		f.write(message_data)
	elif (user_key_press =='a'): # Left
		print('loop: a')
		lSpeed = 0.25
		rSpeed = 0.5
		message_data = str(round(left_obstacle_distance,2)) + "," + str(round(right_obstacle_distance,2)) + "," +  str(1) + "\n"
		f.write(message_data)
	elif (user_key_press == 'd'): # Right
		print('loop: d')
		lSpeed = 0.5
		rSpeed = 0.25
		message_data = str(round(left_obstacle_distance,2)) + "," + str(round(right_obstacle_distance,2)) + "," +  str(2) + "\n"
		f.write(message_data)
	elif (user_key_press == 's'): # Back
		print('loop: s')
		lSpeed = -0.25
		rSpeed = -0.25
		message_data = str(round(left_obstacle_distance,2)) + "," + str(round(right_obstacle_distance,2)) + "," +  str(3) + "\n"
		f.write(message_data)
			
	error_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, lSpeed, sim.simx_opmode_oneshot_wait)
	error_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, rSpeed, sim.simx_opmode_oneshot_wait)
	
	# Stop the robot after 2ms
	delay(0.2)
	lSpeed = 0
	rSpeed = 0
			
	error_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, lSpeed, sim.simx_opmode_oneshot_wait)
	error_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, rSpeed, sim.simx_opmode_oneshot_wait)

# Write data into CSV file
with open('data_sensor_test.csv', 'a') as f:
	    
	while (1):
		listen_keyboard(on_press=press)
	
f.write('exit')


