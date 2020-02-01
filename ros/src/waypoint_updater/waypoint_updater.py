#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf
import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
WAYPOINT_HZ = 10
MAX_DECELERATION = rospy.get_param('/dbw_node/decel_limit')* 0.5
HALT_DISTANCE = 2.5
MAX_SPEED_MiPH = rospy.get_param('/waypoint_loader/velocity')


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # Needed for smooth slow down
        rospy.Subscriber('/current_velocity', TwistStamped, \
                                                     self.velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_wp_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        
        self.last_current_velocity = None
        self.last_base_waypoints = None
        self.last_current_pose = None
        self.traffic_wp = None
        
        self.loop()

        rospy.spin()
        
    def loop(self):
        pass
        rate = rospy.Rate(WAYPOINT_HZ)
        while not rospy.is_shutdown():
            self.publish_next_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.last_current_pose = msg       

    def waypoints_cb(self, msg):
        self.last_base_waypoints = msg
        
    def velocity_cb(self, data):
        self.last_current_velocity = data

    def traffic_wp_cb(self, msg):
        self.traffic_wp = msg.data
        
    def closest_forward_waypoint(self, car):
        # compute global heading angle of car
        quaternion = (car.pose.orientation.x, car.pose.orientation.y, car.pose.orientation.z, car.pose.orientation.w)
        _, _, car_yaw = tf.transformations.euler_from_quaternion(quaternion)  

        # Find index of closest candidate waypoint
        closest_idx = -1
        closest_dist = 999999999999

        for idx in range(len(self.last_base_waypoints.waypoints)):
            wp = self.last_base_waypoints.waypoints[idx]

            # Check if it has a smaller distance than the observed WPs
            this_wp_dist = math.sqrt((wp.pose.pose.position.y - car.pose.position.y)**2 + \
                                     (wp.pose.pose.position.x - car.pose.position.x)**2)
            if this_wp_dist < closest_dist:
                wp_glob_angle = math.atan2(wp.pose.pose.position.y - car.pose.position.y,\
                                           wp.pose.pose.position.x - car.pose.position.x)

                # Check if the wp is in front of the car
                if abs(car_yaw - wp_glob_angle) > math.pi/4:
                    closest_idx +=1
                    closest_dist = this_wp_dist
        return closest_idx 
    
    def normal_speed(self, car, selected_waypoints):
        #print("NORMAL SPEED")
        # Update the speed of all waypoints to maximum speed
        max_speed = MAX_SPEED_MiPH * 1609.340 / (60. * 60.)
        for i in range(len(selected_waypoints)):
            self.set_waypoint_velocity(selected_waypoints, i, max_speed)
        return selected_waypoints
    
    def decelerate(self, car, selected_waypoints, obstacle_id):
        #print("BRAKING")
        print("obstacle_id={} \t len(selected_waypoints)={}".format(obstacle_id, len(selected_waypoints)))
        assert 0 <= obstacle_id <= len(selected_waypoints)
        
        car_vx = self.last_current_velocity.twist.linear.x
        car_vy = self.last_current_velocity.twist.linear.y
        car_speed = math.sqrt(car_vx **2 + car_vy**2)
        max_speed = MAX_SPEED_MiPH * 1609.340 / (60. * 60.)
        #print ("CURRENT CAR SPEED - {}".format(car_speed))
        
        # all waypoints after the obstacle to 0
        for i in range(obstacle_id, len(selected_waypoints)):
            self.set_waypoint_velocity(selected_waypoints, i, 0.0)
        
        # backwards from obstacle to car position
        for i in reversed(range(0, obstacle_id)):
            dist_to_obstacle = max(self.distance(selected_waypoints, i, obstacle_id) - HALT_DISTANCE, 0.0)
            this_wp_speed = min(math.sqrt(0.0 - 2. * MAX_DECELERATION * dist_to_obstacle), max_speed)
            #print("--> wp {} \t set to speed {} \t (distance to obstacle is {})".format(i, this_wp_speed, dist_to_obstacle))
            self.set_waypoint_velocity(selected_waypoints, i, this_wp_speed)
        return selected_waypoints
    
    def publish_next_waypoints(self):
        # get the current angle and position of the car
        if self.last_current_pose is not None and \
            self.last_base_waypoints is not None and \
                self.traffic_wp is not None and \
                    self.last_current_velocity is not None:

            # The car
            car = self.last_current_pose
            
            # Select waypoints
            forward_wp_id = self.closest_forward_waypoint(car)
            selected_waypoints = self.last_base_waypoints.waypoints[forward_wp_id : forward_wp_id + LOOKAHEAD_WPS]
            
            # Check for obstacles ahead - self.traffic_wp holds halt line of red traffic light ahead
            is_obstacle_ahead = True if 0 < self.traffic_wp < forward_wp_id + LOOKAHEAD_WPS else False
            obstacle_id = self.traffic_wp  - forward_wp_id if is_obstacle_ahead else -1

            if is_obstacle_ahead:
               if obstacle_id < 0:
                    # EMERGENCY BRAKE
                    for i in range(len(selected_waypoints)):
                        self.set_waypoint_velocity(selected_waypoints, i, 0.0)
               else:
                    # Normal brake
                    selected_waypoints = self.decelerate(car, selected_waypoints, obstacle_id)
            else:
                selected_waypoints = self.normal_speed(car, selected_waypoints)

            # publish result
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = selected_waypoints
            self.final_waypoints_pub.publish(lane)
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    """
    Compute the acceleration to transform speed v0 to speed v1
    within a distance of delta_s
    """
    """
    def get_acceleration(self, v0, v1, delta_s):
        return (v1**2 - v0**2) / (2. * delta_s)
    
    def get_next_velocity(self, v0, a0, delta_s):
        return math.sqrt(v0**2 + 2 * a0 * delta_s)
    """

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
