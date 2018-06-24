#!/usr/bin/env python
# license removed for brevity

import thread
import time
import rospy
import actionlib
#import robot
#import region
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class robot:
#robot state can be: free or busy
    state = 'free'
    number = -1
    x = 0
    y = 0

    def __init__(self, state, number, x, y):
        self.state = state
        self.number = number
        self.x = x
        self.y = y

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
obstacles_y = [2.50, 5.81, 2.50, 5.81]

regions = []
robots = []

def movebase_robot(x, y, w, z, robot_no):

    robot = actionlib.SimpleActionClient('robot_'+str(robots[robot_no].number)+'/move_base',MoveBaseAction)
    robot.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y	
    goal.target_pose.pose.orientation.w = w
    goal.target_pose.pose.orientation.z = z

    robots[robot_no].x = x
    robots[robot_no].y = y

    robot.send_goal(goal)
    wait = robot.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return robot.get_result()

#to move multi robot to specific points in same time 
def move_robots(robot_no, region_no):
        x = regions[region_no].start_x
        y = regions[region_no].start_y

        while(y <= regions[region_no].end_y):
            while(x <= regions[region_no].end_x):
		isObstacle = False
		for k in range(4):
			if x - obstacles_x[k] >= 0 and x - obstacles_x[k] <= 2.5 and y - obstacles_y[k] >= 0 and y - obstacles_y[k] <= 2.5 :
				isObstacle = True
				break
                if x >= regions[region_no].start_x and x <= regions[region_no].end_x and y >= regions[region_no].start_y and y <= regions[region_no].end_y and (not isObstacle):
                    result = movebase_robot(x, y, 1.0, 0.0, robot_no)
		    print (x,y)
                    if result:
                        rospy.loginfo("Goal execution done!")
                x = x+0.4
            y = y+0.4
            while(x >= regions[region_no].start_x):
		isObstacle = False
		for k in range(4):
			if x - obstacles_x[k] >= 0 and x - obstacles_x[k] <= 2.5 and y - obstacles_y[k] >= 0 and y - obstacles_y[k] <= 2.5 :
				isObstacle = True
				break
                if x >= regions[region_no].start_x and x <= regions[region_no].end_x and y >= regions[region_no].start_y and y <= regions[region_no].end_y and (not isObstacle):
                    result =movebase_robot(x, y, 0.0, 1.0, robot_no)
                    print (x,y)
                    if result:
                        rospy.loginfo("Goal execution done!")
                x = x - 0.4
            y = y + 0.4
        robots[robot_no].state = 'free'
        regions[region_no].state = 'clean'
        print('robot_no', robot_no, ' execution done! at ( ', x, ', ', y, ' )')

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

def directDistance(robot, region):
    return math.sqrt( (robot.x - region.start_x)**2 + (robot.y - region.start_y)**2 )


if __name__ == '__main__':
    split_map()
    new = robot('free', 0, 2.0, 1.0)
    robots.append(new)
    new = robot('free', 1, 2.0, 2.0)
    robots.append(new)
    new = robot('free', 2, 3.0, 2.0)
    robots.append(new)
    try:
	   try:
        	# Initializes a rospy node to let the SimpleActionClient publish and subscribe
        	rospy.init_node('movebase_client_py')
        	allRegionsAreClean = False
        	while(not allRegionsAreClean):
        		allRegionsAreClean = True
        		for i in range(len(regions)):
        			if(regions[i].state == 'dirty'):
        				allRegionsAreClean = False
        		for j in range(len(robots)):
        			if(robots[j].state == 'free'):
        				nearestRegion = -1
        				minDis = 10000.0
        				for i in range(len(regions)):
        					if(regions[i].state == 'dirty'):
        						dis = directDistance(robots[j], regions[i])
        						if(dis < minDis ):
        							minDis = dis
        							nearestRegion = i
        				if(nearestRegion != -1):
        					robots[j].state = 'busy'
        					regions[nearestRegion].state = 'busy'
        					thread.start_new_thread( move_robots, (robots[j].number, regions[nearestRegion].number, ) )
        				else:
        					movebase_robot(1.0*robot_no, 1.0, 1.0, 0.0, robot_no)
	   	for i in range(len(regions)):
	   		print(regions[i].state)
	   except rospy.ROSInterruptException:
        	rospy.loginfo("Navigation test finished.")

    except:
	   print "Error: unable to start thread"

    while 1:
	   pass
