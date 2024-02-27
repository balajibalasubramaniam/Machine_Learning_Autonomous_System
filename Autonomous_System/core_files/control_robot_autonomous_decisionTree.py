# Machine Learning Autonomous System
# Author: Balaji Balasubramaniam
# Date: February 26, 2024

import sys
import time
import sim
from time import sleep as delay
import pandas
from sklearn import tree
from sklearn.tree import DecisionTreeClassifier
import matplotlib
import matplotlib.pyplot as plt

print("Program Started")

# Read the dataset file
df = pandas.read_csv("data_unique_sensor.csv")

# The feature columns are the columns that we try to predict from, and the target column is the column with the values we try to predict. Here, all_senesor_data is the feature columns, robot_direction is the target column. 
features = ['LeftProximityData', 'RightProximityData']
all_senesor_data = df[features]
robot_direction = df['Direction']

# Create the actual decision tree, fit it with our details.
dtree = DecisionTreeClassifier()
dtree = dtree.fit(all_senesor_data.values, robot_direction)

# Visualize the decision tree in a graph and save it in a file
fig = plt.figure(figsize=(25,20))
_ = tree.plot_tree(dtree, feature_names=features, filled=True)
fig.savefig("decistion_tree_visualization.png")

# Connect to CoppeliaSim remote API
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

# Initialize motor speed
left_motor_speed = 0
right_motor_speed = 0

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
explored_location = {"X : 100 - Y : 100","X : -100 - Y : -100"}
destination_position = "X : 2.7 - Y : 2.0" # destination to stop the robot

while (1):
	# Read the proximity sensor data
	data_left_sensor = sim.simxReadProximitySensor(clientID, left_proximity_sensor, sim.simx_opmode_streaming)
	data_right_sensor = sim.simxReadProximitySensor(clientID, right_proximity_sensor, sim.simx_opmode_streaming)
	
	# Robot location in X, Y position
	data_pioneer = sim.simxGetObjectPosition(clientID, pioneer_handle, position, sim.simx_opmode_streaming)
	current_position = "X : "+ str(round(data_pioneer[1][0],1))+" - Y : "+ str(round(data_pioneer[1][1],1))
	if current_position not in explored_location:
		explored_location.add(current_position)
		print("current_position: ",current_position)
		
		# Check if the robot reached destination
		if current_position in destination_position:
			error_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_oneshot_wait)
			error_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_oneshot_wait)
			print("Stopping the robot. Reached destination position-> ",destination_position)
			sys.exit(0)
	
	# If the wall is too far from the proximity sensor's detection range then assume there is no obstacle
	if(data_left_sensor[1]):
		left_obstacle_distance  = data_left_sensor[2][2]
	else:
		left_obstacle_distance = 10;
	
	if(data_right_sensor[1]):
		right_obstacle_distance = data_right_sensor[2][2]
	else:
		right_obstacle_distance = 10;
	
	# Predict the robot direction to move using decision tree model	
	predict_Direction_DecisionTree = dtree.predict([[round(left_obstacle_distance,2),round(right_obstacle_distance,2)]])
	
	if( predict_Direction_DecisionTree == 0): # Forward
		left_motor_speed = 0.5
		right_motor_speed = 0.5
		
	if( predict_Direction_DecisionTree == 1): # Left
		left_motor_speed = 0.25
		right_motor_speed = 0.5
		
	if( predict_Direction_DecisionTree == 2): # Right
		left_motor_speed = 0.5
		right_motor_speed = 0.25
        
	error_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, left_motor_speed, sim.simx_opmode_streaming)
	error_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, right_motor_speed, sim.simx_opmode_streaming)

