#!/usr/bin/env python

#from pid_control2 import PID_control
from pid_control3 import *
import time
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import threading

test=PID_control(0,0)
########################### cmd_vel_subscriber ####################
def converter(linear_x,angular_z):
    if linear_x>0.1:
        test.speedl=linear_x   -  0.05*angular_z
        test.speedr=linear_x  + 0.05*angular_z
        rospy.loginfo("exit from PID_control()_move_forward")
    elif angular_z > 0:
        test.speedl= 0
        test.speedr= 0.3*angular_z
        rospy.loginfo("exit from PID_control()_left_turn")
    else:
        test.speedl= -0.3*angular_z
        test.speedr= 0
        rospy.loginfo("exit from PID_control()_right_turn")
    #time.sleep(2)
    #if angular_z >0.2:
       # PID_control(0,7)
    #else:
    #    PID_control(7,0)
    #time.sleep(3)

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    converter(msg.linear.x,msg.angular.z)



def listener():
   rospy.init_node('cmd_vel_listener')
   rospy.Subscriber("/cmd_vel", Twist, callback)
   test.start()
   rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        pass
    GPIO.cleanup()
   

