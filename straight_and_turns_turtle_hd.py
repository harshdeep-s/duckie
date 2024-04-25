#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist, Point  # Added Point message type
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math
import time

class TurtlesimStraightsAndTurns:
    def _init_(self):  
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True
        self.last_angle = 0
        self.goal_angle = 0
        self.angle_goal_active = False
        self.rotation_completed = False  # New variable to track rotation completion

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self, msg):
        self.last_angle = msg.theta

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_position_callback(self, msg):
        self.goal_angle = math.atan2(msg.y, msg.x)  
        self.goal_distance = math.sqrt(msg.x*2 + msg.y*2)  

        if self.goal_angle == 0:
            self.angle_goal_active = False
        else:
            self.angle_goal_active = True

        if self.goal_distance == 0:
            self.dist_goal_active = False
        else:
            self.dist_goal_active = True

    def timer_callback(self, msg):
        if self.angle_goal_active:  # Check if rotation is needed and not completed
            rotate = Twist()
            rotate.angular.z = self.goal_angle
            self.velocity_publisher.publish(rotate)
            self.angle_goal_active = False
            time.sleep(1)

        elif self.dist_goal_active:  # Check if distance movement is needed and rotation completed
            move = Twist()
            move.linear.x = self.goal_distance
            self.velocity_publisher.publish(move)
            self.dist_goal_active = False
            
        

if _name_ == '_main_': 
    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
