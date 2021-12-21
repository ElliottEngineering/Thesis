#!/usr/bin/env python
# license removed for brevity
#
# https://answers.ros.org/question/107326/publishersubscriber-in-one-python-script/
import rospy
import csv
import math
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import LinkStates
import os

N = 10;


model_name = [[["train5::wheel_1_1_1","train5::wheel_1_1_2"],["train5::wheel_1_2_1","train5::wheel_1_2_2"]],
            [["train5::wheel_2_1_1","train5::wheel_2_1_2"],["train5::wheel_2_2_1","train5::wheel_2_2_2"]],
            [["train5::wheel_3_1_1","train5::wheel_3_1_2"],["train5::wheel_3_2_1","train5::wheel_3_2_2"]],
	        [["train5::wheel_4_1_1","train5::wheel_4_1_2"],["train5::wheel_4_2_1","train5::wheel_4_2_2"]],
            [["train5::wheel_5_1_1","train5::wheel_5_1_2"],["train5::wheel_5_2_1","train5::wheel_5_2_2"]],
            [["train5::wheel_6_1_1","train5::wheel_6_1_2"],["train5::wheel_6_2_1","train5::wheel_6_2_2"]],
            [["train5::wheel_7_1_1","train5::wheel_7_1_2"],["train5::wheel_7_2_1","train5::wheel_7_2_2"]],
            [["train5::wheel_8_1_1","train5::wheel_8_1_2"],["train5::wheel_8_2_1","train5::wheel_8_2_2"]],
            [["train5::wheel_9_1_1","train5::wheel_9_1_2"],["train5::wheel_9_2_1","train5::wheel_9_2_2"]],
            [["train5::wheel_10_1_1","train5::wheel_10_1_2"],["train5::wheel_10_2_1","train5::wheel_10_2_2"]]];
wheel_position_x  = [[[ 0 for y in range(2) ]
                        for x in range(2) ]
                        for z in range(N) ]
wheel_position_y  = [[[ 0 for y in range(2) ]
                        for x in range(2) ]
                        for z in range(N) ]

wheel_Xt  = [1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1]






def link_states_Callback(data):
    #print len(data.name)
    for h in range(len(data.name)):
        #print data.name[k]
        for i in range(N):
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
    pub = rospy.Publisher('/terrain_coefficient', Float32MultiArray, queue_size=1)
    msg = Float32MultiArray()


    datafile = open('/home/joshua/catkin_ws/src/ros_robotics/scripts/sastrugi_1000_0.25-2.csv', 'r')
    Xt_map = list(csv.reader(datafile))
    print "CSV loaded in map variable"

    # bad xt = 1.5
    # one segment
    # 0.43 yes
    # 0.44 no
    # two segment
    # 0.65 yes
    # 0.66 no
    # three segment
    # 1.09 yes
    # 1.1 no
    # four segment
    # 1.45 yes
    # 1.5 no
    # five segment
    # 1.85 yes
    # 1.9 no
    
    
    #bad = 0.58
    #two segment
    #1.41 yes
    #1.43 no    
    #three segment
    # 2.25 yes
    # 2.3 no
    #four segment
    # 3.05 yes
    # 3.1 no
    #five segment
    #3.75 yes
    #3.9 no
    #six segment
    #4.5 yes
    #4.6 no
    
   
    
   #bad_xt = 0.45
   # 1 segment 
   # 0.5 yes
   # 2 segment
   # 0.85 yes
   # 0.9 no
   # 3 segment

  #xt = 0.53 no
  #xt = 0.535 yes
  
  #bad xt = 0.53
  # 1 segment 
  # 0.75 no
  # 3 segment
  # 2.0 yes
  # 2.1 no
  # 5 segment
  # 3.25 yes
  # 3.5 no
  # 10 segment
  # 6.2 yes
  # 6.4 no
  
  
    

    distance = 6.2
    bad_xt = 0.53
    good_xt = 0.9

    rate = rospy.Rate(2000)
    while not rospy.is_shutdown():
        for i in range(N):
            for j in range(2):
                for k in range(2):
                    #print "i is " + repr(i) + ", j is " + repr(j) + ", k is " + repr(k)
                    
                    if wheel_position_x[i][j][k] < 0:
                        wheel_Xt[4*i + 2*j + k] = good_xt

                    elif wheel_position_x[i][j][k] < distance:
                        wheel_Xt[4*i + 2*j + k] = good_xt - (good_xt-bad_xt)*(wheel_position_x[i][j][k]/distance)

                    elif wheel_position_x[i][j][k] < 4*distance:
                        wheel_Xt[4*i + 2*j + k] = bad_xt

                    elif wheel_position_x[i][j][k] < 5*distance:
                        wheel_Xt[4*i + 2*j + k] = good_xt - (bad_xt-good_xt)*(wheel_position_x[i][j][k] - 5*distance) / distance

                    else:
                        wheel_Xt[4*i + 2*j + k] = good_xt
                    wheel_Xt[4*i + 2*j + k] = 0.9
                    #wheel_Xt[4*i + 2*j + k] = (float(Xt_map[int(wheel_position_x[i][j][k]*10) % 1000][int(wheel_position_y[i][j][k]*10) % 1000]) -0.25) / 7 + 0.75
        msg.data = wheel_Xt
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
