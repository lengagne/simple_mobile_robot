#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

vel_msg = Twist()

# mode est une variable qui peut contenir le mode de fonctionnement (a vous de definir)
mode = 0

# distance US de devant
d_front = 0
# distance US de gauche
d_left  = 0
# distance US de droite
d_right = 0

# fonction appelee quand un message arrive sur le topic du capteur US frontal
def callback_front(msg): 
    global d_front # peut etre utile pour le mode
    d_front = msg.range

# fonction appelee quand un message arrive sur le topic du capteur US de gauche
def callback_left(msg): 
    global d_left
    d_left = msg.range

# fonction appelee quand un message arrive sur le topic du capteur US de droite
def callback_right(msg):     
    global  d_right
    d_right = msg.range

rospy.init_node('SolutionLabyrinthe')
# Definition des publisher
pub = rospy.Publisher("TOPIC_VITESSE_ROBOT_A_CHANGER", Twist, queue_size=10)

# Definition des subscriber
rospy.Subscriber("TOPIC_SONAR_FRONT_A_CHANGER", Range, callback_front, queue_size=10)
rospy.Subscriber("TOPIC_SONAR_LEFT_A_CHANGER",  Range, callback_left , queue_size=10)
rospy.Subscriber("TOPIC_SONAR_RIGHT_A_CHANGER", Range, callback_right, queue_size=10)
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
