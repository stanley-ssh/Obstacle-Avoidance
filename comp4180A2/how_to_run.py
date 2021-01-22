# coding=utf-8
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import radians
import cv2
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from robot import Robot
from goal import Goal
from obstacleDetection import ObstacleDetection
import threading 
from enum import Enum
import operator

goal_threshold = 500


"""
class RObState;
	The class is to store the differnet states of the robots afetrr a movement in the system 
	The states are considered to be enums 
"""

class RobState(Enum):
	STARTING = "STARTING"
	OBSTACLEINRANGE ="OBSTACLEINRANGE"
	NOOBSTACLE="NOOBSTACLE"
	GOALAHEAD ="GOALAHEAD"



global image_counter
global state 


"""
Function:move_left 
purpose: movethe robot to the left for a specified amount of time.For debugging poses the function save an image after a move 
retrun NULL 

"""
def move_left(robot,image_counter,seconds):
	new_rate = rospy.Rate(10)
	twist = Twist()
	twist.angular.z=0.5
	twist.linear.x = 1
	time = seconds  
	for i in range(time):
		robot.publish_twist(twist)
		new_rate.sleep()
	rospy.sleep(1)

	image=robot.get_image()
	cv2.imwrite("{}_ML.jpg".format(image_counter), image )


"""
Function : turn_left 
Purpose : The function turns the robot to the the left without moving. 

"""

def turn_left(robot,image_counter,seconds):
	new_rate = rospy.Rate(10)
	twist = Twist()
	twist.angular.z=0.5
	#twist.linear.x = 1
	time = seconds  
	for i in range(time):
		robot.publish_twist(twist)
		new_rate.sleep()
	rospy.sleep(1)

"""
Function : turn_right 
Purpose : The function turns the robot to the the right without moving for a specified number of sconds . 

"""
def turn_right(robot,image_counter,seconds):
	new_rate = rospy.Rate(10)
	twist = Twist()
	twist.angular.z=-0.5
	#twist.linear.x = 1
	time = seconds  
	for i in range(time):
		robot.publish_twist(twist)
		new_rate.sleep()
	rospy.sleep(1)


"""
purpose : The function determines of the closest obstacle in the image is at the left of the image 

"""
def isleft (image ):
	# check if an obsacle is in the left of the robot
	obstacle_x,obstacle_y,obstacle_h = getObstacleCord(image= image) 
	if (( obstacle_y!=0   ) and  (obstacle_x <150 and obstacle_x !=0)  ):
		return True 
	else :
		return False

# The fuction deteermines if an obstacle is in the right of an image 
def isright (image ):
	obstacle_x,obstacle_y,obstacle_h = getObstacleCord(image= image)
	if (( obstacle_y !=0 ) and   obstacle_x > 150):
		return True 
	else :
		return False

#GoalWidth : gets the width of he goal in the image 
def goalWidth(image):
	gol = Goal()
	tol = gol.find_goal_single_image(image)
	goal_width = tol[2]

	return goal_width

# go_forward:Move the robot to forward for a specified nuber of seconds and save the image to a file. The saved image is used for debugging purposes 

def go_forward(seconds,image_counter, robot ):
        """
        :param seconds:int
        """
        rate = rospy.Rate(10)
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.angular.y = 0
        move_cmd.angular.x= 0 
        move_cmd.linear.y =0 
        move_cmd.linear.z =0 
        move_cmd.linear.x = 1
        times = seconds
        for x in range(0, times):
            robot.publish_twist(move_cmd)
            rate.sleep()
        rospy.sleep(1)

        image=robot.get_image()
        cv2.imwrite("{}_MF.jpg".format(image_counter), image )


"""
Function:Move the robo tto the right of the system for a specific amount of time afer which the image is stpred for debugging purposes 

"""
def move_right(robot,image_counter,seconds):
	new_rate = rospy.Rate(10)
	twist = Twist()
	twist.angular.z=-0.5
	twist.linear.x = 1
	 
	time =seconds
	for i in range(time):
		robot.publish_twist(twist)
		new_rate.sleep()
	rospy.sleep(1)

	image=robot.get_image()
	cv2.imwrite("{}_MR.jpg".format(image_counter), image )


"""
Function:getGoalArea 
Purpose: The function gets the area of the goal in the current image. The area is calcuated by geting teh width of the rectangular box by the heighht of the rectangular box

"""
def getGoalArea(image):
	gol = Goal()
	tol = gol.find_goal_single_image(image)
	print("goal is at x = {}".format(tol))
	area = tol[2] * tol[3]
	print ("The area is {}".format(area))

	return area


"""
Function: getGoalCord 
Purpose:  Helper method to get  coordinate of the rectangular box on the goal

"""
def getGoalCord(image):
	gol = Goal() 
	x,y,w,h = gol.find_goal_single_image(image )
	return x,y,w,h


def goalPos(image):
	"""
	This function checks the position opf the goal in the imahe if the goal is in the
	Returs left is the goal is in teh left and returns right if the goal is in te roght 

	"""

		
	goal_x,goal_y, goal_w, goal_h = getGoalCord(image)
	print "goal cordinate {},{},{},{}".format(goal_x,goal_y,goal_w,goal_h)
	if goal_y !=0 :
		h,w = image .shape[:2]
		value = goal_x 
		if value < 100  :
			return "left_goal"
		elif value in range(100,w//2):
			return "middle_goal" 
		elif value > 100:
			return "right_goal"
	else :
		return "GoalInvisble"




def getObstacleCord(image ):
	obs = ObstacleDetection()
	obstacle_cord  = obs.getObstacleSingleImage(image )
	obstacle_area  = obstacle_cord[2] * obstacle_cord[3]
	print ( "The area of obstacle is {}".format(obstacle_area))
	obstacle_y = obstacle_cord[1]
	obstacle_x = obstacle_cord[0]
	obstacle_h = obstacle_cord[3]
	return obstacle_x,obstacle_y,obstacle_h


"""
Function : getObstaclePos
Purpose : Get the position of the obstacle. If the position is to the right of the image, the left of the image or in the middle of the image 
"""
def getObstaclePos(robot):
	ret_value = " "
	global state 
	image = robot.get_image()
	
	if (isleft(image)):
		print("The obstacle is on the left ")
		ret_value = "left"
		

	elif( isright(image)):
		ret_value = "right"
	else:
		ret_value = "none"
		
	#TODO check if we have obstacle on both sides 
	return ret_value



def proceessMovement(position, robot):
	print("The obstacle position is {}".format(position))
	image_counter =1
	image = robot.get_image()
	_,obstacle_y,obstacle_h = getObstacleCord(image= image)
	if position =="left":
		# if it is at the left we want to move to the right 
		move_right(robot,image_counter)
		state = RobState.GOALAHEAD
		move_left(robot,image_counter)
		processStates (state, robot)

	elif position =="right":
		#if obstacle_y < 180:

		move_left(robot,image_counter)
		
		state  = RobState.STARTING
		#move_right(robot,image_counter)
		processStates(state,robot)
	elif position == "none":
		print("going forward ")

		#go_forward(5,image_counter,robot)
		#state =RobState.GOALAHEAD
		#processStates(state,robot)


def getMin1(image):
	obs = ObstacleDetection()
	obstacle_cord  = obs.getObstacleSingleImage(image )
	#print(obstacle_cord)
	allMins = []
	if len(obstacle_cord) >0 :

		min_value =obstacle_cord [0][1]
		
		for i in range(1,len(obstacle_cord)):
			print obstacle_cord[i][1]
			if obstacle_cord[i][1] < min_value:
				min_tuple = obstacle_cord[i]
				min_value =obstacle_cord[i][1]
			elif obstacle_cord[i][1] ==min_value:
				allMins.append(obstacle_cord[i])
				allMins.append(min_tuple)
	#print("all min is{}".format(allMins))

	return allMins



def getSmallestX(list1):
	data = min(list1, key = operator.itemgetter(0))
	return data


def leftObs (image ):
	# check if an obsacle is in the left of the robot
	obstacle_x,_,_,_=  getObstacleCord(image= image) 
	#print ( obstacle_x)
	if ( obstacle_x in range(1,200) ):
		return True 
	else :
		return False


def right_Obs (image ):
	obstacle_x,_,_,_= getObstacleCord(image= image)
	if ( obstacle_x >380):
		return True 
	else :
		return False


def middle_Obs(image):
	obstacle_x,_,obstacle_w,_ = getObstacleCord(image= image)
	h,w = image.shape[:2]
	value = obstacle_x + obstacle_w
	#@print "value is {}".format(value)
	if ( value  in range(w//2)  ):
		return True
	else:
		return False


"""
FUnction: getObstacleCord
Purose : Get the coorodinate of the closest obstacle in the image 
returun value : rectange box coordinate 
"""
def getObstacleCord(image ):

	data = getMin(image)
	#print "data is {}".format(data)
	return data



"""
Function :getMin 
Purpose : helper method to get the closest obstacle to the robot


"""
def getMin(image):
	obs = ObstacleDetection()
	obstacle_cord  = obs.getObstacleSingleImage(image )
	#print(obstacle_cord)
	#print obstacle_cord
	ret_value = (0,0,0,0)
	#print obstacle_cord
	if (len(obstacle_cord) > 0 ):

		data = min (obstacle_cord, key = lambda t: t[1])
		#print data
		
		alldata= [item for item in obstacle_cord if item[1] == data[1]]
		#print alldata
		if (len(alldata) > 1 ):
			ret_value = getSmallestX(alldata)
		elif (len(alldata) ==1 ):
			ret_value = alldata[0]
	return ret_value


def getSmallestX(list1):
	data = min(list1, key = operator.itemgetter(0))
	return data



#Helper method to check if the obstacle is atthe right 
def isleft (image ):
	# check if an obsacle is in the left of the robot
	obstacle_x,_,_,_=  getObstacleCord(image= image) 
	print ( obstacle_x)
	if ( obstacle_x in range(1,200) ):
		return True 
	else :
		return False

def isright (image ):
	obstacle_x,_,_,_= getObstacleCord(image= image)
	if ( obstacle_x >200):
		return True 
	else :
		return False

def middle(image):
	obstacle_x,_,obstacle_w,_ = getObstacleCord(image= image)
	
	h,w = image .shape[:2]

	value = obstacle_x + obstacle_w
	print "value is {}".format(value)
	if ( value  in range(1,w//2)  ):
		return True
	else:
		return False


# Given a dictionary and a value the function will get the key relating to the value in the dictionary 
def get_key(val,my_dict): 
    for key, value in my_dict.items(): 
         if val == value: 
             return key 
  
    return "key doesn't exist"


# find the tuple with th minimumy-value in the dictionary 
def minDicts(dict1):
	min_value = dict1["forward"][1]
	out_tuple = dict1["forward"]
	for key,value in dict1.items():
		if value[1] < min_value:
			min_value = value[1]
			out_tuple= value

	return out_tuple


"""
The scan function moves the robot from left to righ and get tge images at those movenet.
	The coordinate of the closest obstacle at the differnet point is stored ina dictionary with key as the direction (e.g right) and the 
	value od the coordinate of the closest obstacle 

"""
def scan(robot):
	# loomk forward
	image_counter =1 
	forwardImage = robot.get_image()
	cv2.imwrite("forward.jpg", forwardImage )
	# look left 
	turn_left(robot,image_counter,5)
	leftImage = robot.get_image()
	cv2.imwrite("leftView.jpg", leftImage )
	# look right 
	turn_right(robot,image_counter,10)
	right_image = robot.get_image()
	cv2.imwrite("right_view.jpg", right_image )
	dict1 = {"left":getObstacleCord(leftImage),"forward":getObstacleCord(forwardImage),"right":getObstacleCord(right_image)}
	
	path = getclearestPath(dict1)

	#go back to facing forward 
	turn_left(robot,image_counter,1)

	return path
	 

# helper fnction to get the path with the list numberof obstacles 
def getclearestPath(dict1):
	avalue = minDicts(dict1)
	key_value = get_key(avalue,dict1)

	if key_value == "left":
		return "left"
	elif key_value =="right":
		return "right"
	elif key_value =="forward":
		return "forward"


"""
The function process the scan and gets the path which has the smallest obstacle. Based on the path we publish a movement (Left or right or go_forward)
"""

def newProcess2(robot):
	image_counter =1 
	image = robot.get_image()
	path = scan(robot)
	state = RobState.GOALAHEAD
	print "The path is {}".format(path)
	if path == "left":
		turn_left(robot,image_counter,10)
		go_forward(10,image_counter,robot)
		turn_right(robot,image_counter,7)
		print "done moving left"
		#turn_right(robot,image_counter,5)
	elif path =="right":
		move_right(robot,image_counter,10)
		turn_left(robot,image_counter,10)
	elif path == "forward":
		robot.go_forward(1)
		state = RobState.GOALAHEAD
	goTowardsGoal(robot,state)



def newProcess(robot):
	image_counter = 2
	image = robot.get_image()
	pos = getObstacleCord(image)
	print ("pos is {}".format(pos))
	state = RobState.STARTING
	if pos[1] !=0 :

		if pos[1] < 180:
			if (middle_Obs(image)):
				"There is an obstacle in my face"
				if leftObs(image ):
					print "left obstacle"
					move_right(robot,image_counter,5)
					move_left(robot,image_counter,5)
					robot.go_forward(2)
					state = RobState.GOALAHEAD
				elif right_Obs(image):
					print "right obstacle"
					#move_left(robot,image_counter,5)
					#move_right(robot,image_counter,5)
					image_counter=+ 1 
					robot.go_forward(1)
					state = RobState.GOALAHEAD
				
			else:
				#moveToGoal(robot)
				print "no obstacle in front"
				state = RobState.GOALAHEAD
			
		elif pos[1] == 0:
			print "no obstacle "
		else:
			print "obstacle is far"
			# geth where the obstacle is
			#moveToGoal(robot)
			state = RobState.GOALAHEAD 
	else:
		print "no obstacle in sight "
		state = moveToGoal(robot)


	return state


def goTowardsGoal(robot,currState):
	image_counter=1
	image = robot.get_image()
	goal_width= goalWidth(image)
	cv2.imwrite("{}_GG.jpg".format(image_counter), image )
	image_counter +=1
	#pos = getClosestCord(image)
	while ( goal_width < 650 and goal_width !=0  ):
		#if pos[1] >180:
			#print "can go forward "

		if (currState == RobState.STARTING):

			newProcess2(robot)
		elif currState == RobState.GOALAHEAD:
			currState= newProcess(robot)
		robot.go_forward(1)
		image = robot.get_image()
		cv2.imwrite("{}_GF.jpg".format(image_counter), image )
		image_counter+=1
		#moveToGoal(robot)
		goal_width= goalWidth(image)

	print "done!"


"""
Function: goTowarsd goal The function moves the robot to the direction of the goal post when there is no obstacke in its way 
"""
def moveToGoal(robot):
	image_counter = 1 
	image = robot.get_image()
	position_goal = goalPos(image)
	goal_width= goalWidth(image)
	print("Goal post is {}".format(position_goal))
	state = RobState.GOALAHEAD
	if position_goal == "left_goal":

		move_left(robot,image_counter,5)
		# image_counter =+ 1 
		# while goal_width <500 and goal_width !=0:
		# 	go_forward(5,image_counter,robot)
		# 	image = robot.get_image()
		# 	goal_width=goalWidth(image)
		state = RobState.GOALAHEAD
		#processStates(state,robot)
	elif position_goal =="right_goal":
		move_right(robot,image_counter,5)
		state = RobState.GOALAHEAD
		#processStates(state,robot)
	elif position_goal == "middle_goal":
		robot.go_forward(1)
		state = RobState.GOALAHEAD
		#processStates(state,robot)

	return state



def processStates(currState, robot):
	image = robot.get_image()
	goal_width= goalWidth(image)
	_,obstacle_y,obstacle_h = getObstacleCord(image= image)


	if currState == RobState.STARTING and goal_width < 500:
		#check the location of the obstacle 
		#moveToGoal(robot)
		if obstacle_y <180:

			pos = getObstaclePos(robot)
			proceessMovement(pos,robot)
	elif currState == RobState.GOALAHEAD:
		if goal_width!=0 and (obstacle_y <180 or obstacle_y ==0 ):
			print "Goal ahead and no obstacle "
			#move_forward until hoal is reached 
			if goal_width < 500:
				image_counter =1 
				go_forward(5,image_counter, robot)
				currState = RobState.GOALAHEAD
				processStates(currState,robot)

		elif goal_width!=0 and (obstacle_y> 180  ):
			print "Goal ahead but obstacle is visble"
			currState = RobState.STARTING
			processStates(currState,robot)



if __name__ == '__main__':
    robot=Robot()
    count = 1

    start = RobState.GOALAHEAD
    newProcess2(robot) 
