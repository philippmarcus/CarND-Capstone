
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

import math

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        
        # read out params
        vehicle_mass = rospy.get_param('~vehicle_mass')
        fuel_capacity = rospy.get_param('~fuel_capacity')
        brake_deadband = rospy.get_param('~brake_deadband')
        decel_limit = rospy.get_param('~decel_limit')
        accel_limit = rospy.get_param('~accel_limit')
        wheel_radius = rospy.get_param('~wheel_radius')
        wheel_base = rospy.get_param('~wheel_base')
        steer_ratio = rospy.get_param('~steer_ratio')
        max_lat_accel = rospy.get_param('~max_lat_accel')
        max_steer_angle = rospy.get_param('~max_steer_angle')
        min_speed = 0.02
        
        """
        Idea:
        - Create a PID controller for throttle, break, steer
        - The PID controllers use a low-pass to filter the output
        """
                
        # Define controllers @TODO Tune parameters
        self.controller_throttle = PID(kp=1.0, ki=1.0, kd=1.0, mn=0., mx=1.)
        self.controller_break = PID(kp=1.0, ki=1.0, kd=1.0, mn=0., mx=3250.)
        self.controller_steer = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        #PID(kp=1.0, ki=1.0, kd=1.0, mn=-8.2, mx=8.2)
        
        # Define low pass filters @TODO Tune parameters
        self.lowpass_throttle = LowPassFilter(tau=1.0, ts=1.0)
        self.lowpass_break = LowPassFilter(tau=1.0, ts=1.0)
        self.lowpass_steer = LowPassFilter(tau=3.0, ts=1.0)
        pass
    
    """
    Computes x,y magnitude for each object that brings
    an x, y value.
    """
    def _get_magn(self, data):
        return math.sqrt(data.x **2 + data.y**2)
    
    def control(self, proposed_linear_v, proposed_angular_v,\
                      current_linear_v, current_angular_v, \
                      is_dbw_enabled, sample_time):

        linear_velocity = self._get_magn(proposed_linear_v)
        angular_velocity = proposed_angular_v.z
        current_velocity = self._get_magn(current_linear_v)
        
        # Derive error in linear velocity for throttle and break
        error_v = current_velocity - linear_velocity
        error_throttle = -1. *error_v # negative value if too fast
        error_break = error_v 
        
        print("error_throttle: ", error_throttle)
        
        
        # Throttle Command
        cmd_throttle = self.controller_throttle.step(error_throttle, sample_time)
        cmd_throttle = self.lowpass_throttle.filt(cmd_throttle)
        print("cmd_throttle:", cmd_throttle)
        
        # Break Command
        cmd_break = self.controller_break.step(error_break, sample_time)
        cmd_break = self.lowpass_break.filt(cmd_break)

        # Steering Command
        cmd_steer = self.controller_steer.get_steering(linear_velocity, angular_velocity, current_velocity)
        cmd_steer = self.lowpass_steer.filt(cmd_steer)
        print("cmd_steer:", cmd_steer)
        print("===")
        
        return cmd_throttle, cmd_break, cmd_steer
