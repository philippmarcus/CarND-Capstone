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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_next_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        print("setting last_current_pose")
        self.last_current_pose = msg
        

    def waypoints_cb(self, msg):
        # TODO: Implement
        print("setting last_base_waypoints")
        self.last_base_waypoints = msg
        
    def velocity_cb(self, data):
        self.last_current_velocity = data

    def traffic_wp_cb(self, msg):
        print("SETTING TRAFFIC WP", msg.data)
        self.traffic_wp = msg.data
        
    def map_wp_to_angle(car_pos, waypoint):
        pass
        
    
    def publish_next_waypoints(self):
        # get the current angle and position of the car
        print("publish_next_waypoints called")
        if self.last_current_pose is not None and \
            self.last_base_waypoints is not None and \
                self.traffic_wp is not None and \
                    self.last_current_velocity is not None:
            
            car = self.last_current_pose
            
            # Compute current speed
            vx = self.last_current_velocity.twist.linear.x
            vy = self.last_current_velocity.twist.linear.y
            current_speed = math.sqrt(vx **2 + vy**2)
            
            current_speed_mph = current_speed * (60 * 60) / 1609.34
            print("-----> Current Speed (m/s)= {} (mph)={}".format( current_speed,current_speed_mph ))

            # compute global heading angle of car
            quaternion = (car.pose.orientation.x, car.pose.orientation.y, car.pose.orientation.z, car.pose.orientation.w)
            _, _, car_yaw = tf.transformations.euler_from_quaternion(quaternion)  
            
            # Find index of closest candidate waypoint
            closest_idx = -1
            closest_dist = 999999999999
            
            for idx in range(len(self.last_base_waypoints.waypoints)):
                #if wp_is_candidate[idx] and wp_rel_dist[idx] < closest_dist:
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
            
            # select waypoints only until traffic obstacle and stop
            print("self.traffic_wp", self.traffic_wp)
            end_wp = closest_idx + LOOKAHEAD_WPS if self.traffic_wp == -1 else self.traffic_wp
            #end_wp = closest_idx+LOOKAHEAD_WPS
            selected_waypoints = self.last_base_waypoints.waypoints[closest_idx : end_wp]

            # adapt target speed to slow down in front of obstacles
            max_speed = 50. * 1609.340 / (60. * 60.)
            target_speed_mps = 0. if self.traffic_wp != -1 else  max_speed
            delta_v = (target_speed_mps - current_speed) / max(min(len(selected_waypoints)-1, 60), 1)
            print("==>delta_v: ", delta_v)
            
            # Smooth stopping - lineary reduce speed to stopping point
            for i in range(len(selected_waypoints)):
                #cur_wp_speed = min(max(current_speed + (i+1)*delta_v, 0), target_speed_mps)
                cur_wp_speed = 0. if self.traffic_wp != -1 else  max_speed
                #cur_wp_speed = 50. * 1609.340 / (60. * 60.)
                print("------ wp {} ----> {}".format(i, cur_wp_speed))
                selected_waypoints[i].twist.twist.linear.x = cur_wp_speed

            # publish result
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = selected_waypoints
            self.final_waypoints_pub.publish(lane)
        # find out the closest forward waypoinit
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
