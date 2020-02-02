from collections import namedtuple
import cv2
import math

import glob
import csv
from random import random

SimulatorImage = namedtuple('SimulatorImage', 'img gt_status dist')


MAX_CAMERA_LOOKAHEAD_DISTANCE = 140.
MIN_CAMERA_LOOKAHEAD_DISTANCE = 0.

class TLDataCollector(object):
    def __init__(self):
        self.data = []
        self.data_statistics = {'UNKOWN': 0, \
                                'RED': 0, \
                                'YELLOW': 0,  \
                                'GREEN': 0}
        
        # Read in old images
        self.img_path = "./data/"
        existing_images = glob.glob(self.img_path + "*.png")
        for existing_image in existing_images:
            file_info = existing_image.split("_")
            self.data.append(SimulatorImage(img=file_info[0], gt_status=file_info[1], dist=file_info[2][:-4]))
            self.data_statistics[file_info[1]] += 1
                        
        print("===>EXISTING DATA: GREEN={} \t YELLOW={} \t RED={} \t UNKNOWN={}".format(self.data_statistics['GREEN'], \
                                                                                        self.data_statistics['YELLOW'], \
                                                                                        self.data_statistics['RED'], \
                                                                                        self.data_statistics['UNKOWN']))
        
        # Read in old csv file
        self.csv_writer = csv.writer(open(self.img_path + "simulator_data.csv", "wb"))
        
        # save last image status to avoid duplicates
        self.last_dist = None
        self.last_state = None
        
        return
    
    def process_image(self, cv_image, light_state, light_pos_x, light_pos_y, car_pos_x, car_pos_y):
        # Global id
        img_id = len(self.data)
        
        gt_status = "UNKNOWN"
        if light_state == 0:
            gt_status = "RED"
        elif light_state == 1:
            gt_status = "YELLOW"
        elif light_state == 2:
            gt_status = "GREEN"
        elif light_state == 4:
            gt_status = "UNKOWN"
        
        # Compute distance to car
        distance = round(math.sqrt((light_pos_x - car_pos_x)**2 + (light_pos_y - car_pos_y)**2))
        
        # No traffic light found or visible for the camera
        if not (MIN_CAMERA_LOOKAHEAD_DISTANCE <= distance <= MAX_CAMERA_LOOKAHEAD_DISTANCE) or \
                (light_pos_x == -1 and light_pos_y == -1):
            gt_status = "UNKOWN"
            distance = 0
            
        if gt_status == "UNKNOWN" and random() > 0.4:
            # Skip 60% of all UNKNOWN images
            print("SKIPPING IMAGE")
            return
        
        # Avoid collection of duplicate images
        if (self.last_state is not None and self.last_state == gt_status) and \
            (self.last_dist is not None and self.last_dist == distance) and \
             random() > 0.7:
            # Already collected this scene before
            print("SKIPPING IMAGE")
            return
        else:
            # Found a new image
            self.last_state = gt_status
            self.last_dist = distance

        # File name
        filename = self.img_path + "{}_{}_{}.png".format(img_id, gt_status, distance)
        cv2.imwrite(filename, cv_image)
        
        # Store information
        self.data.append(SimulatorImage(filename, gt_status, distance))
        self.data_statistics[gt_status] += 1
        self.csv_writer.writerow([img_id, gt_status, distance])
        
        if len(self.data) % 10 == 0:
            # Log output every 10 iterations
            print("===>EXISTING DATA: GREEN={} \t YELLOW={} \t RED={} \t UNKNOWN={}".format(self.data_statistics['GREEN'], \
                                                                                            self.data_statistics['YELLOW'], \
                                                                                            self.data_statistics['RED'], \
                                                                                            self.data_statistics['UNKOWN']))
        return