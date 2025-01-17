#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
from light_classification.tl_data_collector import TLDataCollector

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        # Needed for ML training data collection
        if rospy.get_param("~ml_data_collection"):
            rospy.loginfo("COLLECTING ML TRAINING DATA")
            self.ml_training_data = TLDataCollector()

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #Only process every 4th image
        if msg.header.seq % 4 == 0:
            return
        
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights(msg)
        
        # Special case for ML training data collection
        if rospy.get_param("~ml_data_collection") and (self.pose is not None):
            # Collect data
            car_pos_x = self.pose.pose.position.x
            car_pos_y = self.pose.pose.position.y
            
            light_x = self.waypoints[light_wp].pose.pose.position.x if light_wp != -1 else -1
            light_y = self.waypoints[light_wp].pose.pose.position.y if light_wp != -1 else -1
            
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            self.ml_training_data.process_image(cv_image, state, light_x, light_y, car_pos_x, car_pos_y)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
    
    """
    Returns the id of the waypoint in waypoints that is closest to (x,y)
    """
    def closest_waypoint_id(self, waypoints, x, y):    
        # Euclidean distance lamba function
        dist = lambda wp, x, y: math.sqrt((wp.pose.pose.position.y - y)**2 + \
                                          (wp.pose.pose.position.x - x)**2)
        # Distance for each waypoint
        wp_distances = [dist(wp, x, y) for wp in waypoints]
        return np.argmin(wp_distances)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
           
        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self, msg):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
            
        Args:
            msg: Latest camera image received from the car

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Check if needed input is already available
        if self.waypoints is None or self.pose is None or self.lights is None:
            return -1, TrafficLight.UNKNOWN

        # Closest waypoint to the vehicle
        vehicle_x = self.pose.pose.position.x
        vehicle_y = self.pose.pose.position.y
        vehicle_wp_id = self.closest_waypoint_id(self.waypoints, vehicle_x, vehicle_y)
        
        # Select closest traffic light in front of the vehicle
        traffic_light_wp_ids = [self.closest_waypoint_id(self.waypoints, \
                                                         tl.pose.pose.position.x, \
                                                         tl.pose.pose.position.y)  
                                for tl in self.lights]

        # No future traffic light comming
        if (max(traffic_light_wp_ids) < vehicle_wp_id):
            return -1, TrafficLight.UNKNOWN
        
        # Select traffic light candidates, i.e. infront of the vehicle
        tl_candidate_wp_ids, tl_candidate_lights, light_ids = zip(*filter(lambda (id, light, light_id): id > vehicle_wp_id, zip(traffic_light_wp_ids,\
                                                                                                                                   self.lights, \
                                                                                                                                   range(len(self.lights)))))
        # Select traffic light from candidates
        light = tl_candidate_lights[np.argmin(tl_candidate_wp_ids)]
        light_id = light_ids[np.argmin(tl_candidate_wp_ids)]
        traffic_light_wp_id = min(tl_candidate_wp_ids)
                
        # No traffic light was found within the lookaehad distance
        if (traffic_light_wp_id  - vehicle_wp_id > 200):
            return -1, TrafficLight.UNKNOWN
        
        
        # Find closest waypoint for selected traffic light
        stop_line_positions = self.config['stop_line_positions']
        stopline_x = stop_line_positions[light_id][0]
        stopline_y = stop_line_positions[light_id][1]
        stop_line_wp_id = self.closest_waypoint_id(self.waypoints, stopline_x, stopline_y)
        
        # Vehicle already passed the stop line and is in the intersection! Emergency Brake!
        if stop_line_wp_id < vehicle_wp_id:
            stop_line_wp_id = vehicle_wp_id
        
        # closes
        state = None
        # Set state depending on current operation mode (data collection or online mode)
        if rospy.get_param("~ml_data_collection"):
            # In ML training data collection mode
            state = light.state # TODO debug only.
        else:
            # In operational mode
            state = self.get_light_state(light)
        
        return stop_line_wp_id, state # return the stop line,  not the traffic light

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
