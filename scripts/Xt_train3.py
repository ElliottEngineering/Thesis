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

model_name = [[["train3::wheel_1_1_1","train3::wheel_1_1_2"],["train3::wheel_1_2_1","train3::wheel_1_2_2"]],
            [["train3::wheel_2_1_1","train3::wheel_2_1_2"],["train3::wheel_2_2_1","train3::wheel_2_2_2"]],
            [["train3::wheel_3_1_1","train3::wheel_3_1_2"],["train3::wheel_3_2_1","train3::wheel_3_2_2"]]];
wheel_position_x  = [[[ 0 for y in range(2) ]
                        for x in range(2) ]
                        for z in range(3) ]
wheel_position_y  = [[[ 0 for y in range(2) ]
                        for x in range(2) ]
                        for z in range(3) ]

wheel_Xt  = [1,1,1,1,1,1,1,1,1,1,1,1]






def link_states_Callback(data):
    #print len(data.name)
    for h in range(len(data.name)):
        #print data.name[k]
        for i in range(3):
            for j in range(2):
                for k in range(2):

                    if data.name[h] == model_name[i][j][k]:
                        #print "k " + repr(k) + " i " + repr(i) + " j " + repr(j)
                        #print model_name[i][j]
                        wheel_position_x[i][j][k] = data.pose[h].position.x
                        wheel_position_y[i][j][k] = data.pose[h].position.y
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
        for i in range(0,3):
            for j in range(0,2):
                for k in range(0,2):
                    #print "i is " + repr(i) + ", j is " + repr(j) + ", k is " + repr(k)
                    wheel_Xt[4*i + 2*j + k] = float(Xt_map[int(wheel_position_x[i][j][k]*10) % 1000][int(wheel_position_y[i][j][k]*10) % 1000])
		    #wheel_Xt[4*i + 2*j + k] = 1.25
        msg.data = wheel_Xt
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
