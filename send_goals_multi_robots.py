#!/usr/bin/env python
# license removed for brevity

import thread
import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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
        x = 0.9
        y = 0.9

	obstacles_x = [2.00, 2.00, 6.00, 6.00]	
	obstacles_y = [3.00, 6.26, 3.00, 6.26]

        for i in range(12):
            for j in range(25):
		isObstacle = False
		for k in range(4):
			if x - obstacles_x[k] > 0 and x - obstacles_x[k] < 1.9 and y - obstacles_y[k] > 0 and y - obstacles_y[k] < 1.9 :
				isObstacle = True
				break
                if x > start_x and x < end_x and y > start_y and y < end_y and (not isObstacle):
                    result = movebase_robot(x, y, 1.0, 0.0, str(robot_no))
		    print (x,y)
                    if result:
                        rospy.loginfo("Goal execution done!")
                x = x+0.4
            y = y+0.4
            for j in range(25):
		isObstacle = False
		for k in range(4):
			if x - obstacles_x[k] > 0 and x - obstacles_x[k] < 1.9 and y - obstacles_y[k] > 0 and y - obstacles_y[k] < 1.9 :
				isObstacle = True
				break
                if x > start_x and x < end_x and y > start_y and y < end_y and (not isObstacle):
                    result =movebase_robot(x, y, 0.0, 1.0, str(robot_no))
                    print (x,y)
                    if result:
                        rospy.loginfo("Goal execution done!")
                x = x - 0.4
            y = y + 0.4
#amr da tneyn 
if __name__ == '__main__':
    try:
	   try:
        	# Initializes a rospy node to let the SimpleActionClient publish and subscribe
        	rospy.init_node('movebase_client_py')

        	thread.start_new_thread( move_robots, (0,0.7,0.7,9.34,5.0,) )
        	thread.start_new_thread( move_robots, (1,0.7,5.0,9.34,9.34,) )

	   except rospy.ROSInterruptException:
        	rospy.loginfo("Navigation test finished.")

    except:
	   print "Error: unable to start thread"

    while 1:
	   pass
