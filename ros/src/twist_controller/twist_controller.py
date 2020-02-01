
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

import math

class Controller(object):
    def __init__(self, *args, **kwargs):
        min_speed = 0.02
 
        # Define controllers @TODO Tune parameters
        self.controller_steer = YawController(rospy.get_param('~wheel_base'), \
                                              rospy.get_param('~steer_ratio'), \
                                              min_speed, \
                                              rospy.get_param('~max_lat_accel'),\
                                              rospy.get_param('~max_steer_angle'))  
        self.controller_acceleration = PID(kp=1.0, ki=1.0, kd=5.0, mn=rospy.get_param('~decel_limit'), mx=rospy.get_param('~accel_limit'))

        # Define low pass filters @TODO Tune parameters
        self.lowpass_steer = LowPassFilter(tau=5.0, ts=1.0)
        self.lowpass_acceleration = LowPassFilter(tau=10.0, ts=1.0)        
        return
    
    # Resets all controllers
    def reset(self):
        self.controller_acceleration.reset()
    
    """
    Computes x,y magnitude for each object that brings
    an x, y value.
    """
    def _get_magn(self, data):
        return math.sqrt(data.x **2 + data.y**2)
    
    def control(self, proposed_linear_v, proposed_angular_v,\
                      current_linear_v, current_angular_v, \
                      is_dbw_enabled, sample_time,):
        
       
        # Values from current run
        desired_velocity = self._get_magn(proposed_linear_v)
        desired_angular_velocity = proposed_angular_v.z
        current_velocity = self._get_magn(current_linear_v)
        
        # Velocity error - steered by acceleration
        error_vel = desired_velocity - current_velocity 
        #print("desired_velocity= {} \t current_velocity= {} \t error_vel={}".format(desired_velocity, current_velocity, error_vel))

        # Steering Command
        cmd_steer = self.controller_steer.get_steering(desired_velocity, desired_angular_velocity, current_velocity)
        cmd_steer = self.lowpass_steer.filt(cmd_steer)

        # Acceleration Command
        cmd_accel = self.controller_acceleration.step(error_vel, sample_time)
        cmd_accel = self.lowpass_acceleration.filt(cmd_accel)

        # Convert acceleration to throttle and brake
        cmd_throttle = 0.0
        cmd_brake = 0.0
        if cmd_accel > 0.:
            # Accelerate proportional to accel cmd
            cmd_throttle = cmd_accel / rospy.get_param('~accel_limit')
        else:
            # Brake - compute needed torque
            brake_torque =  rospy.get_param('~wheel_radius') * (rospy.get_param('~vehicle_mass') + rospy.get_param('~fuel_capacity')*GAS_DENSITY)  * -1. * cmd_accel
            
            cmd_brake = min(brake_torque, 3250.)

        return cmd_throttle, cmd_brake, cmd_steer
