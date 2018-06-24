#!/usr/bin/env python
# license removed for brevity

import thread
import time
import rospy
import actionlib
#import robot
#import region
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class robot:
#robot state can be: free or busy
    state = 'free'
    number = -1
    x = 0
    y = 0
    region = None

    def __init__(self, state, number, x, y, region):
        self.state = state
        self.number = number
        self.x = x
        self.y = y
        self.region = region

class region:
#region state can be: dirty, clean or busy
    state = 'dirty'
    number = 0.0
    start_x = 0.0
    start_y = 0.0
    end_x = 0.0
    end_y = 0.0

    def __init__(self, state, number, start_x, start_y, end_x, end_y):
        self.state = state
        self.number = number
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y

obstacles_x = [1.80, 1.80, 5.70, 5.70]	
obstacles_y = [2.50, 5.80, 2.50, 5.80]

regions = []

def movebase_robot(x, y,w,z,robot_no):

    robot_1 = actionlib.SimpleActionClient('robot_'+robot_no+'/move_base',MoveBaseAction)
    robot_1.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y	
    goal.target_pose.pose.orientation.w = w
    goal.target_pose.pose.orientation.z = z

    robot_1.send_goal(goal)
    wait = robot_1.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return robot_1.get_result()

#to move multi robot to specific points in same time 
def move_robots(robot_no, start_x, start_y, end_x, end_y):
        x = start_x
        y = start_y

        for i in range(12):
            for j in range(25):
		isObstacle = False
		for k in range(4):
			if x - obstacles_x[k] >= 0 and x - obstacles_x[k] <= 2.5 and y - obstacles_y[k] >= 0 and y - obstacles_y[k] <= 2.5 :
				isObstacle = True
				break
                if x >= start_x and x <= end_x and y >= start_y and y <= end_y and (not isObstacle):
                    result = movebase_robot(x, y, 1.0, 0.0, str(robot_no))
		    print (x,y)
                    if result:
                        rospy.loginfo("Goal execution done!")
                x = x+0.4
            y = y+0.4
            for j in range(25):
		isObstacle = False
		for k in range(4):
			if x - obstacles_x[k] >= 0 and x - obstacles_x[k] <= 2.5 and y - obstacles_y[k] >= 0 and y - obstacles_y[k] <= 2.5 :
				isObstacle = True
				break
                if x >= start_x and x <= end_x and y >= start_y and y <= end_y and (not isObstacle):
                    result =movebase_robot(x, y, 0.0, 1.0, str(robot_no))
                    print (x,y)
                    if result:
                        rospy.loginfo("Goal execution done!")
                x = x - 0.4
            y = y + 0.4

def split_map():
    new = region('dirty', 0, 0.9, 0.9, 9.34, obstacles_y[0])
    regions.append(new)
    new = region('dirty', 1, 0.9, obstacles_y[0]+2.5, 9.34, obstacles_y[1])
    regions.append(new)
    new = region('dirty', 2, 0.9, obstacles_y[1]+2.5, 9.34, 9.34)
    regions.append(new)
    new = region('dirty', 3, 0.9, obstacles_y[0], obstacles_x[0], obstacles_y[0]+2.5)
    regions.append(new)
    new = region('dirty', 4, obstacles_x[0]+2.5, obstacles_y[0], obstacles_x[2], obstacles_y[0]+2.5)
    regions.append(new)
    new = region('dirty', 5, obstacles_x[2]+2.5, obstacles_y[0], 9.34, obstacles_y[0]+2.5)
    regions.append(new)
    new = region('dirty', 6, 0.9, obstacles_y[1], obstacles_x[0], obstacles_y[1]+2.5)
    regions.append(new)
    new = region('dirty', 7, obstacles_x[0]+2.5, obstacles_y[1], obstacles_x[2], obstacles_y[1]+2.5)
    regions.append(new)
    new = region('dirty', 8, obstacles_x[2]+2.5, obstacles_y[1], 9.34, obstacles_y[1]+2.5)
    regions.append(new)

if __name__ == '__main__':
    split_map()
    try:
	   try:
        	# Initializes a rospy node to let the SimpleActionClient publish and subscribe
        	rospy.init_node('movebase_client_py')
        	print('hello world')
        	print(regions)
        	print(regions[1].start_x)
        	print(regions[1].start_y)
        	print(regions[1].end_x)
        	print(regions[1].end_y)
        	thread.start_new_thread( move_robots, (1, regions[0].start_x, regions[0].start_y, regions[0].end_x, regions[0].end_y, ) )
        	thread.start_new_thread( move_robots, (0, regions[1].start_x, regions[1].start_y, regions[1].end_x, regions[1].end_y, ) )
        	thread.start_new_thread( move_robots, (2, regions[2].start_x, regions[2].start_y, regions[2].end_x, regions[2].end_y, ) )

	   except rospy.ROSInterruptException:
        	rospy.loginfo("Navigation test finished.")

    except:
	   print "Error: unable to start thread"

    while 1:
	   pass
