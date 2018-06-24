#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x, y, w, z):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y	
    goal.target_pose.pose.orientation.w = w
    goal.target_pose.pose.orientation.z = z

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def movebase_client_in_region(x1, y1, x2, y2, x_sign, y_sign, step):
	x = x1
	y = y1
	for i in range(abs(int((y2-y1)/0.8)+1)):
		for j in range(abs(int((x2-x1)/0.4)+1)):
			if x_sign*x < x_sign*x2 and y_sign*y < y_sign*y2:
				result = movebase_client(x, y, 1.0, 0.0)
				print (x, y)
				if result:
					rospy.loginfo("Goal execution done!")
			x = x + x_sign*step
		y = y + y_sign*step
		for j in range(abs(int((x2-x1)/0.4)+1)):
			if x_sign*x < x_sign*x2 and y_sign*y < y_sign*y2:
				result = movebase_client(x, y,0.0,1.0)
				print (x,y)
				if result:
					rospy.loginfo("Goal execution done!")
			x = x + x_sign*step
		y = y + y_sign*step

if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        #x = 0.9
        #y = 0.9

	map_start = 0.9
	map_end = 9.34

	obstacle_len = 1.8

	obstacles_x = [2.00, 2.00, 5.90, 5.90]	
	obstacles_y = [3.00, 6.26, 3.00, 6.26]

	
	movebase_client_in_region(map_start, map_start, map_end, obstacles_y[0], 1.0, 1.0, 0.4)
	movebase_client_in_region(map_start, obstacles_y[0], obstacles_x[0], obstacles_y[0]+obstacle_len, 1.0, 1.0, 0.2)
	movebase_client_in_region(obstacles_x[0]+obstacle_len, obstacles_y[0]+obstacle_len, obstacles_x[2], obstacles_y[0], 1.0, -1.0, 0.2)
	movebase_client_in_region(obstacles_x[2]+obstacle_len, obstacles_y[0], map_end, obstacles_y[0]+obstacle_len, 1.0, 1.0, 0.2)
	movebase_client_in_region(map_end, obstacles_y[0]+obstacle_len, map_start, obstacles_y[1], -1.0, 1.0, 0.4)
	movebase_client_in_region(map_start, obstacles_y[1], obstacles_x[0], obstacles_y[0]+obstacle_len, 1.0, 1.0, 0.2)
	movebase_client_in_region(obstacles_x[0]+obstacle_len, obstacles_y[1]+obstacle_len, obstacles_x[2], obstacles_y[1], 1.0, -1.0, 0.2)
	movebase_client_in_region(obstacles_x[2]+obstacle_len, obstacles_y[1], map_end, obstacles_y[1]+obstacle_len, 1.0, 1.0, 0.2)
	movebase_client_in_region(map_end, obstacles_y[1]+obstacle_len, map_start, map_end, -1.0, 1.0, 0.4)
	

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
