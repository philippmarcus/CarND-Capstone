#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # Create Publishers for controll commands
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create `Controller` object
        self.controller = Controller()

        # Subscribe to all the topics you need to
        self.sub_twist_cmd        = rospy.Subscriber('/twist_cmd', TwistStamped, \
                                                     self.callback_twist_cmd)    
        self.sub_current_velocity = rospy.Subscriber('/current_velocity', TwistStamped, \
                                                     self.callback_current_velocity)
        self.sub_dbw_enabled      = rospy.Subscriber('/vehicle/dbw_enabled', Bool, \
                                                     self.callback_dbw_enabled)
        
        # Set state variables
        self.last_dbw_enabled = None
        self.last_current_velocity = None
        self.last_twist_cmd = None
        
        self.was_dbw_enabled = False

        # Start the loop with 50 Hz
        self.loop()
    
    """
    Callback for the /twist_cmd subscription
    """
    def callback_twist_cmd(self, data):
        self.last_twist_cmd = data
        pass
    
    """
    Callback for the /current_velocity subscription
    """
    def callback_current_velocity(self, data):
        self.last_current_velocity = data
        pass
    
    """
    Callback for the /vehicle/dbw_enabled subscription
    """
    def callback_dbw_enabled(self, data):
        self.last_dbw_enabled = data
        pass

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            sample_time = 1./50. # according to rate
            
            # Check if DBW is active
            if self.last_dbw_enabled is not None and self.last_dbw_enabled.data is True and \
                self.last_current_velocity and self.last_twist_cmd:
                
                if not self.was_dbw_enabled:
                    # Reset PID controller first
                    self.controller.reset()
                   
                # Get predicted throttle, brake, and steering using `twist_controller`
                throttle, brake, steering = self.controller.control(self.last_twist_cmd.twist.linear, \
                                                                    self.last_twist_cmd.twist.angular, \
                                                                    self.last_current_velocity.twist.linear, \
                                                                    self.last_current_velocity.twist.angular, \
                                                                    self.last_dbw_enabled.data, \
                                                                    sample_time)
                
                # Publish the steering commands
                print("COMMAND throttle= {}, brake= {}, steering={}".format(throttle, brake, steering))
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
