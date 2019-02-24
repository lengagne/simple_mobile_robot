#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

vel_msg = Twist()

# distance US de devant
d_front = 0

# fonction appelee quand un message arrive sur le topic du capteur US frontal
def callback_front(msg): 
    global d_front # peut etre utile pour le mode
    d_front = msg.range


rospy.init_node('SolutionLabyrinthe')
# Definition des publisher
pub = rospy.Publisher("TOPIC_VITESSE_ROBOT_A_CHANGER", Twist, queue_size=10)

# Definition des subscriber
rospy.Subscriber("TOPIC_SONAR_FRONT_A_CHANGER", Range, callback_front, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    
    if d_front > 0.5:
        vel_msg.linear.x = 0.1;
        vel_msg.angular.y = 0.0;
    if d_front < 0.4:
        vel_msg.linear.x = -0.1;
        vel_msg.angular.y = 0.0;
        
    pub.publish(vel_msg);
    rate.sleep();
