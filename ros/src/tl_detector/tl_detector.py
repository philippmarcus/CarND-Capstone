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
        light_wp, state = self.process_traffic_lights()
        print("RECEIVED light_wp ", light_wp, " STATE ", state)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
            print("NOT PUBLISHING!!!")
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            print("PUBLISHING ", light_wp)
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            print("PUBLISHING ", self.last_wp)
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, car):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """ 
        # Catch invalid states
        if self.waypoints is None:
            return -1
        
        # Code copied from waypoint updater
        # compute global heading angle of car
        quaternion = (car.orientation.x, car.orientation.y, car.orientation.z, car.orientation.w)
        _, _, car_yaw = tf.transformations.euler_from_quaternion(quaternion)  

        # Find index of closest candidate waypoint
        closest_idx = -1
        closest_dist = 999999999999

        for idx in range(len(self.waypoints)):
            #if wp_is_candidate[idx] and wp_rel_dist[idx] < closest_dist:
            wp = self.waypoints[idx]

            # Check if it has a smaller distance than the observed WPs
            this_wp_dist = math.sqrt((wp.pose.pose.position.y - car.position.y)**2 + \
                                     (wp.pose.pose.position.x - car.position.x)**2)
            if this_wp_dist < closest_dist:

                wp_glob_angle = math.atan2(wp.pose.pose.position.y - car.position.y,\
                                           wp.pose.pose.position.x - car.position.x)

                # Check if the wp is in front of the car
                if abs(car_yaw - wp_glob_angle) > math.pi/4:
                    closest_idx +=1
                    closest_dist = this_wp_dist
        return closest_idx

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

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        
        if self.waypoints is None or car_position is None or self.lights is None:
            print("No car position or waypoints available...")
            return -1, TrafficLight.UNKNOWN
        
        # Find waypoint id that is nearest to the next traffic light
        light_wp = -1
        closest_wp_dist = 999999999
        # iterate over the next waypoints
        print("searching for closest traffic light")
        for idx in range(car_position, car_position+200):
            # Iterate over all traffic lights a
            for tl_id in range(len(self.lights)):
                x = self.lights[tl_id].pose.pose.position.x
                y = self.lights[tl_id].pose.pose.position.y
                
                wpx = self.waypoints[idx].pose.pose.position.x
                wpy = self.waypoints[idx].pose.pose.position.y
                light_wp_dist = math.sqrt((y - wpy)**2 + (x - wpx)**2)
                
                if light_wp_dist < closest_wp_dist and light_wp_dist < 10.:
                    light_wp = idx
                    closest_wp_dist = light_wp_dist
                    light = self.lights[tl_id]
        print("closest traffic light at waypoint ", light_wp, " car currently at", car_position)     
        
        # Find closest TL from stream
        
        for tl in self.lights:
            tl.pose.pose.position.x
            tl.pose.pose.position.y
            tl.state
        
        if light:
            state = self.get_light_state(light)
            state = light.state # TODO debug only
            return light_wp, state
        #self.waypoints = None
        return light_wp, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
