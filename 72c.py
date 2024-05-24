#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class TargetFollower:
    def _init_(self):
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "duckie" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin()  # Spin forever but listen to message callbacks

    # AprilTag Detection Callback
    def tag_callback(self, msg):
        self.move_object(msg.detections)

    # Stop Robot before node has shut down. This ensures the robot does not keep moving
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        rospy.loginfo("Stopping Robot")
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_object(self, detections):
        
        if len(detections) == 0:
            self.stop_robot()
        else:
            x = detections[0].transform.translation.x
            y = detections[0].transform.translation.y
            z = detections[0].transform.translation.z

            rospy.loginfo("x,y,z: %f %f %f", x, y, z)

            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()

            if z > 0.5:  
                cmd_msg.v = 0.3  
            if z < 0.5:  
                cmd_msg.v = -0.3  
            else:
                cmd_msg.v = 0.0  
        
            if x > 0.1:  
                cmd_msg.omega = -3.0  
            if x < -0.1:  
                cmd_msg.omega = 3.0  
            else:
                cmd_msg.omega = 0.0  

            self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '_main_':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass