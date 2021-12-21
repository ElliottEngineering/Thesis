#!/usr/bin/env python
# license removed for brevity
#
# https://answers.ros.org/question/107326/publishersubscriber-in-one-python-script/
import rospy
import csv
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import LinkStates
import os

model_name = [["robot43::wheel_1_1_1","robot43::wheel_1_1_2"],
				["robot43::wheel_1_2_1","robot43::wheel_1_2_2"],
            	["robot43::wheel_2_1_1","robot43::wheel_2_1_2"],
            	["robot43::wheel_3_1_1","robot43::wheel_3_1_2"]];
wheel_position_x  = [[0,0],
					[0,0],
					[0,0],
					[0,0]];


wheel_position_y  = [[0,0],
					[0,0],
					[0,0],
					[0,0]];


wheel_Xt  = [1,1,1,1,1,1,1,1]






def link_states_Callback(data):
    #print len(data.name)
    for h in range(len(data.name)):
        #print data.name[k]
    	for j in range(4):
			for k in range(2):
				if data.name[h] == model_name[j][k]:
					#print "k " + repr(k) + " i " + repr(i) + " j " + repr(j)
					#print model_name[i][j]
					wheel_position_x[j][k] = data.pose[h].position.x
					wheel_position_y[j][k] = data.pose[h].position.y
    #rospy.loginfo(rospy.get_caller_id() + " I heard %s", wheel_position_x[1][0])
    # print "#########################################################"

def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Xt_handler_node', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_Callback)
    pub = rospy.Publisher('/terrain_coefficient', Float64MultiArray, queue_size=10)
    msg = Float64MultiArray()


    datafile = open('/home/joshua/catkin_ws/src/ros_robotics/scripts/sastrugi_1000_0.25-2.csv', 'r')
    Xt_map = list(csv.reader(datafile))
    print "CSV loaded in map variable"



    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        for j in range(4):
            for k in range(2):
                #print "i is " + repr(i) + ", j is " + repr(j) + ", k is " + repr(k)
                #wheel_Xt[4*i + 2*j + k] = float(Xt_map[int(wheel_position_x[j][k]*10) % 1000][int(wheel_position_y[j][k]*10) % 1000])
	    		wheel_Xt[2*j + k] = 2
        msg.data = wheel_Xt
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
