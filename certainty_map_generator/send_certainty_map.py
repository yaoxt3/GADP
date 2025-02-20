#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import torch
import logging
import os
from modules.cnn_predictor import CNNPredictor
from modules.param_set import params_set
from kinect_demo.kinect_client import AzureRGBDFetchNode

class CertaintyMapNode:
    def __init__(self):
        rospy.init_node('certainty_map_node', anonymous=True)

        self.args = params_set()

        logging.info('Start CNNPredictor ...')
        self.predictor = CNNPredictor(self.args.network, self.args.input_channel, self.args.label_type, self.args.tta_size, self.args.img_size)

        self.bridge = CvBridge()

        self.rgbd_node = AzureRGBDFetchNode(use_depth=True, use_rgb=True)

        self.certainty_map_pub = rospy.Publisher('/certainty_map', Image, queue_size=1)

    def process_images(self):
        fed_image, fed_cam_info = self.rgbd_node.fetch_image()
        if fed_image is not None:
            # Predict certainty map
            input = self.predictor.input_process(fed_image, self.args.mode == 'online', chann=self.args.input_channel)
            certainty_map, angle_map = self.predictor.inference(input)
            certainty_map = np.where(certainty_map > 0.7, 1, 0)

            # Convert certainty map to ROS Image message
            certainty_map_img = (certainty_map * 255).astype(np.uint8) 
            certainty_map_img = np.expand_dims(certainty_map_img, axis=-1)
            certainty_map_img = np.repeat(certainty_map_img, 3, axis=-1).astype(np.uint8)
            certainty_map_msg = self.bridge.cv2_to_imgmsg(certainty_map_img, encoding="bgr8")

            # Publish certainty map
            self.certainty_map_pub.publish(certainty_map_msg)

    def run(self):
        rate = rospy.Rate(5) 
        while not rospy.is_shutdown():
            self.process_images()
            rate.sleep()

if __name__ == '__main__':
    node = CertaintyMapNode()
    node.run()