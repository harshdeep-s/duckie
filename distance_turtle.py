#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist 
import time 

def move_turtle_square(): 
    rospy.init_node('turtlesim_square_node', anonymous=True)
    
    # Init publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
    rospy.loginfo("Turtles are great at drawing squares!")

    ########## YOUR CODE GOES HERE ##########

    move_forward = Twist()
    move_forward.linear.x = 1.0  # Move forward at 1 m/s

    # Create a Twist message to turn
    turn = Twist()
    turn.linear.y = 1.0  # Rotate at 1 rad/s

    move_backwards = Twist()
    move_backwards.linear.x = -1.0  # Move forward at 1 m/s

    # Create a Twist message to turn
    turn_down = Twist()
    turn_down.linear.y = -1.0  # Rotate at 1 rad/s

    # Move in a square pattern
    while True:  # Perform 4 sides of the square
        # Move forward for 4 seconds
        velocity_publisher.publish(move_forward)
        time.sleep(1)

        # Stop moving
        velocity_publisher.publish(Twist())

        # Turn for 2 seconds
        velocity_publisher.publish(turn)
        time.sleep(1)

        velocity_publisher.publish(Twist())

        velocity_publisher.publish(move_backwards)
        time.sleep(1)

        velocity_publisher.publish(Twist())

        velocity_publisher.publish(turn_down)
        time.sleep(1)


        # Stop turning
        velocity_publisher.publish(Twist())

    # Stop the turtle when done
    

    ###########################################

if _name_ == '_main_': 

    try: 
        move_turtle_square() 
    except rospy.ROSInterruptException: 
        pass
