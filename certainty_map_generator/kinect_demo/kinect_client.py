#!/usr/bin/env python3

import numpy as np
import cv2
from skimage.transform import resize
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

"""
Realsense RGBD ROS Node
"""
class AzureRGBDFetchNode:
    def __init__(self, depth_topic='/depth_to_rgb/image_raw', 
                 rgb_topic='/rgb/image_raw', CameraInfo_topic='/rgb/camera_info',
                 use_depth=True, use_rgb=True):

        self.use_depth = use_depth
        self.use_rgb = use_rgb

        self.br = CvBridge()

        self.Image_depth_buff = None
        rospy.Subscriber(depth_topic, Image, self.depth_callback)
        rospy.loginfo('ROS subscribing to {}'.format(depth_topic))

        self.Image_rgb_buff = None
        rospy.Subscriber(rgb_topic, Image, self.rgb_callback)
        rospy.loginfo('ROS subscribing to {}'.format(rgb_topic))

        self.CamInfo_buff = None
        self.is_updated_CI = False
        rospy.Subscriber(CameraInfo_topic, CameraInfo, self.CInfo_callback)
        rospy.loginfo('ROS subscribing to {}'.format(CameraInfo_topic))

    def depth_callback(self, msg):
        self.Image_depth_buff = msg

    def rgb_callback(self, msg):
        self.Image_rgb_buff = msg

    def CInfo_callback(self, msg):
        self.CamInfo_buff = msg
    
    def fetch_image(self, inpaint_depth=True):
        rgb_image, depth_image = None, None
        output_shape = 400
        
        ## Camera Info
        if self.CamInfo_buff is None: 
            return None, None
        else: cam_info = self.CamInfo_buff
        
        ## RGB Image
        if self.use_rgb and self.Image_rgb_buff is None: 
            return None, None
        elif self.use_rgb:
            rgb_image = self.br.imgmsg_to_cv2(self.Image_rgb_buff)

            if rgb_image.shape[2] == 4:
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGBA2RGB)
            
            alpha = 1.0 # Contrast control
            beta = 2 # Brightness control
            rgb_image = cv2.convertScaleAbs(rgb_image, alpha=alpha, beta=beta)

            rgb_image = resize(rgb_image, (480, 640), anti_aliasing=True)

            height, width, _ = rgb_image.shape 
            rgb_image = rgb_image[:, (width - height)//2 : (width - height)//2 + height, :]
             
            rgb_image = resize(rgb_image, output_shape=(output_shape, output_shape), 
                               preserve_range=True).astype(np.float32)
            
            rgb_image = rgb_image[...,[2,1,0]].copy()
            

        ## Depth Image
        if self.use_depth and self.Image_depth_buff is None: return None, None
        elif self.use_depth:
            depth_image = self.br.imgmsg_to_cv2(self.Image_depth_buff, desired_encoding="32FC1")
            depth_image = resize(depth_image, (480, 640), anti_aliasing=True)
            depth_image = np.array(depth_image, dtype=np.float32)/1000
            height, width = depth_image.shape
            depth_image = depth_image[:, (width - height)//2 : (width - height)//2 + height]
            
            if inpaint_depth: depth_image = self.depth_inpaint(depth_image)
            depth_image = resize(depth_image, output_shape=(output_shape, output_shape), 
                                 preserve_range=True).astype(np.float32)
            
        
        rospy.loginfo('Fetch depth_image size: {}'.format(depth_image.shape))
        
        zoom_ratio = float(output_shape) / float(height)
        fx_0, fy_0, cx_0, cy_0 = cam_info.K[0], cam_info.K[4], cam_info.K[2], cam_info.K[5]
        cam_info_update = {
            'fx': fx_0 * zoom_ratio, 'fy': fy_0 * zoom_ratio,
            'cx': (cx_0 - (width - height)//2) * zoom_ratio, 'cy': cy_0 * zoom_ratio
        }

        if self.use_depth and not self.use_rgb:
            image = np.expand_dims(depth_image, axis=0)
        elif self.use_depth and self.use_rgb:
            image = np.concatenate((np.expand_dims(depth_image, axis=0), 
                                    np.transpose(rgb_image, (2, 0, 1))), axis=0)
        else:
            raise KeyError("Not implemented.")
        return image, cam_info_update

    @staticmethod
    def depth_inpaint(image, missing_value=0):
        """
        Inpaint missing values in depth image.
        :param missing_value: Value to fill in the depth image.
        """
        image = cv2.copyMakeBorder(image, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        mask = (image == missing_value).astype(np.uint8)
        imax, imin = np.abs(image).max(), np.abs(image).min()
        irange = imax - imin
        image = ((image - imin) / irange * 255.0).astype(np.uint8)
        image = cv2.inpaint(image, mask, 2, cv2.INPAINT_NS)
        image = image[1:-1, 1:-1]
        image = image.astype(np.float32) / 255.0 * irange + imin
        return image

    @staticmethod
    def rgb_inpaint(image, mask):
        inpainted_image = np.zeros_like(image)
        for i in range(3):  # Assuming image has 3 channels (RGB)
            inpainted_image[:, :, i] = cv2.inpaint(image[:, :, i], mask, inpaintRadius=3, flags=cv2.INPAINT_NS)
        return inpainted_image
    
    @staticmethod
    def Reconstruct_XYZ(depth_value, point, intrinsics_info):
        """
        Convinient class 
        the first index of point is u, along x direction of image, or width
        Get X, Y, Z coordinate of a pixel on depth image.
        By: reconstructed from fxfypxpy
        where depth image is the z axis value
        """
        u, v = point[0], point[1] # in width, height
        cx, cy = intrinsics_info['cx'], intrinsics_info['cy']
        fx_inv, fy_inv = 1/intrinsics_info['fx'], 1/intrinsics_info['fy']
        _z = depth_value
        _x = _z * ((u - cx) * fx_inv)
        _y = _z * ((v - cy) * fy_inv)
        xyz = [_x, _y, _z]
        return xyz

def main():
    rospy.init_node('realsense_rgbd_fetch_node', anonymous=True)
    node = AzureRGBDFetchNode()

    # Create a figure for plotting
    plt.ion()
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    colorbars = [None, None]

    while not rospy.is_shutdown():
        # Fetch the images
        image, cam_info_update = node.fetch_image()

        if image is not None:
            depth_image = image[0]
            rgb_image = np.transpose(image[1:], (1, 2, 0))

            # Update the plots
            axes[0].clear()
            axes[0].set_title('Depth Image')
            im0 = axes[0].imshow(depth_image, cmap='viridis')

            axes[1].clear()
            axes[1].set_title('RGB Image')
            im1 = axes[1].imshow(rgb_image)

            plt.draw()
            plt.pause(1)
        else:
            rospy.logerr("Failed to fetch images.")

if __name__ == '__main__':
    main()