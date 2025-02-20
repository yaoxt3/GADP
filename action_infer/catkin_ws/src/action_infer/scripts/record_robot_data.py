#!/usr/bin/env python3
#coding=utf-8
import rospy
from std_msgs.msg import Float32MultiArray, Int16
from action_infer.msg import PoseAction
import pyrealsense2 as rs
import pickle
import numpy as np
import open3d as o3d
import copy
import time
import os
import cv2
from skimage.transform import resize
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

def depth_to_pointcloud_with_color(depth_frame, color_frame, pc, aligned_intrinsics, workspace):
    """
    Generate a 6D point cloud (XYZ + RGB) by mapping the color frame to the 3D points from the depth frame.
    :param depth_frame: The depth frame from the RealSense camera.
    :param color_frame: The color frame from the RealSense camera.
    :param pc: RealSense pointcloud object (rs.pointcloud()).
    :param aligned_intrinsics: The intrinsics of the aligned frames.
    :return: A numpy array of shape (N, 6), where each row is [X, Y, Z, R, G, B].
    """
    
    # Calculate point cloud (XYZ coordinates) from depth frame
    pc.map_to(color_frame)  # Align the point cloud to the color frame
    points = pc.calculate(depth_frame)  # Compute the point cloud

    # Retrieve vertices (3D points)
    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

    # Retrieve the texture coordinates (u, v for color mapping)
    tex_coords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)

    # Get the color data
    color_image = np.asanyarray(color_frame.get_data())

    # Convert the texture coordinates to pixel indices in the color image
    u = (tex_coords[:, 0] * color_image.shape[1]).astype(np.int32)
    v = (tex_coords[:, 1] * color_image.shape[0]).astype(np.int32)

    # Ensure pixel indices are valid (within the image boundaries)
    u = np.clip(u, 0, color_image.shape[1] - 1)
    v = np.clip(v, 0, color_image.shape[0] - 1)

    # Extract RGB values corresponding to the (u, v) coordinates
    colors = color_image[v, u, :] / 255.0  # Normalize to range [0, 1]
    
    mask = (
        (vertices[:, 0] > workspace[0][0]) & (vertices[:, 0] < workspace[0][1]) &
        (vertices[:, 1] > workspace[1][0]) & (vertices[:, 1] < workspace[1][1]) &
        (vertices[:, 2] > workspace[2][0]) & (vertices[:, 2] < workspace[2][1])
    )

    # Combine XYZ and RGB to form a 6D point cloud
    pointcloud_6d = np.hstack((vertices[mask], colors[mask]))

    return pointcloud_6d

def depth_to_pointcloud_wo_color(depth_frame, workspace, pc):
    points = pc.calculate(depth_frame)
    points             = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
    points = points[np.where((points[..., 0] > workspace[0][0]) & (points[..., 0] < workspace[0][1]) &
                    (points[..., 1] > workspace[1][0]) & (points[..., 1] < workspace[1][1]) &
                    (points[..., 2] > workspace[2][0]) & (points[..., 2] < workspace[2][1]))]
    return points

def visualize_6d_pointcloud(pointcloud_6d):
    """
    Visualize a 6D point cloud (XYZ + RGB) using Open3D.
    :param pointcloud_6d: A numpy array of shape (N, 6), where each row is [X, Y, Z, R, G, B].
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_6d[:, :3])  # XYZ coordinates
    pcd.colors = o3d.utility.Vector3dVector(pointcloud_6d[:, 3:])  # RGB values
    o3d.visualization.draw_geometries([pcd])
    
def visualize_pointcloud(pointcloud_3d):
    """
    Visualize a 3D point cloud (XYZ) using Open3D.
    :param pointcloud_3d: A numpy array of shape (N, 3), where each row is [X, Y, Z].
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_3d[:, :3])  # XYZ coordinates
    o3d.visualization.draw_geometries([pcd])

class RecordRobotData:
    def __init__(self, depth_topic='/depth_to_rgb/image_raw', 
                 rgb_topic='/rgb/image_raw', CameraInfo_topic='/rgb/camera_info'):
        rospy.init_node('record_robot_data')

        ctx = rs.context()
        if len(ctx.devices) > 0:
            for dev in ctx.devices:
                print('Found device:', dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number))
        else:
            print("No Intel Device connected")
        devices = ctx.query_devices()
        for dev in devices:
            dev.hardware_reset()
            print('Reset device:', dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number))

        self.pose = None
        self.action = None
        self.data_cache = {'agent_pose': [], 'action': [], 'point_cloud': [], 'azure_rgb': [], 'azure_depth': [], 'rs_img': []}

        rospy.Subscriber('/franka_pose_action', PoseAction, self.obs_callback, queue_size=1)
        rospy.Subscriber('/stop_command', Int16, self.stop_callback, queue_size=1) 
        self.br = CvBridge()

        self.Image_depth_buff = None
        rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.loginfo('ROS subscribing to {}'.format(depth_topic))

        self.Image_rgb_buff = None
        rospy.Subscriber(rgb_topic, Image, self.rgb_callback, queue_size=1)
        rospy.loginfo('ROS subscribing to {}'.format(rgb_topic))

        self.workspace = [(-0.35, 0.25),
                          (-0.36, 0.36),
                          (0.6, 1.2)]
        # self.workspace = [(-0.32, 0.25), (-0.36, 0.36), (0.6, 1.2)]
        # Configuring Realsense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.flag = False
        
        # first frame is not used
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        print("First frame captured")

        # rospy.on_shutdown(self.save_data)
    
    def depth_callback(self, msg):
        self.Image_depth_buff = msg

    def rgb_callback(self, msg):
        self.Image_rgb_buff = msg

    def CInfo_callback(self, msg):
        self.CamInfo_buff = msg
    
    def obs_callback(self, msg):
        if self.flag:
            return
        
        rospy.loginfo("Received observation data")
        self.pose = np.array(msg.pose_data.data, dtype=np.float32)
        self.action = np.array(msg.action_data.data, dtype=np.float32)
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        rs_img = np.asanyarray(color_frame.get_data())
        pc = rs.pointcloud()  
        points = depth_to_pointcloud_wo_color(depth_frame, self.workspace,pc)
        
        
        #Azure RGB Image
        azure_rgb = self.br.imgmsg_to_cv2(self.Image_rgb_buff)
        if azure_rgb.shape[2] == 4:
            azure_rgb = cv2.cvtColor(azure_rgb, cv2.COLOR_RGBA2RGB)
        alpha = 1.0 # Contrast control
        beta = 2 # Brightness control
        azure_rgb = cv2.convertScaleAbs(azure_rgb, alpha=alpha, beta=beta)
        azure_rgb = resize(azure_rgb, (480, 640), anti_aliasing=True)
        height, width, _ = azure_rgb.shape 
        azure_rgb = azure_rgb[:, (width - height)//2 : (width - height)//2 + height, :]
        azure_rgb = resize(azure_rgb, output_shape=(400, 400), 
                        preserve_range=True).astype(np.float32)
        azure_rgb = azure_rgb[...,[2,1,0]].copy()

        # print(f"Pc: {points.shape}, Kinect:{azure_rgb.shape}, action:{self.action.shape}, pose:{self.pose.shape}")
        print(f"Action: {self.action}, pc:{points.size}")
            
        ## Azure Depth Image
        azure_depth = self.br.imgmsg_to_cv2(self.Image_depth_buff, desired_encoding="32FC1")
        azure_depth = resize(azure_depth, (480, 640), anti_aliasing=True)
        azure_depth = np.array(azure_depth, dtype=np.float32)/1000.0
        height, width = azure_depth.shape
        azure_depth= azure_depth[:, (width - height)//2 : (width - height)//2 + height]
        azure_depth = self.depth_inpaint(azure_depth)
        azure_depth = resize(azure_depth, output_shape=(400, 400), 
                                 preserve_range=True).astype(np.float32)

        self.data_cache['point_cloud'].append(copy.deepcopy(points))
        self.data_cache['action'].append(copy.deepcopy(self.action))
        self.data_cache['agent_pose'].append(copy.deepcopy(self.pose)) 
        self.data_cache['azure_rgb'].append(copy.deepcopy(azure_rgb))
        self.data_cache['azure_depth'].append(copy.deepcopy(azure_depth))
        self.data_cache['rs_img'].append(copy.deepcopy(rs_img))
    
    def save_data(self, directory = '/media/yxt/Seagate 2TB/diffusion-data/1-12/pick-place-kinect-rs/'):
        cnt = len(os.listdir(directory))
        path = os.path.join(directory, 'traj'+str(cnt)+'.pkl')
        print(f"Saving data to: {path}")        
        with open(path, 'wb') as f:
            pickle.dump(self.data_cache, f)
    
    def stop_callback(self, msg):
        if msg.data == 0:
            rospy.loginfo("stop command received, saving data")
            self.pipeline.stop()
            
            self.save_data()
            self.flag = True
            rospy.signal_shutdown("Stop command received, shutting down node")
    
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
    
    def run(self):
        rospy.spin()
    
def main():
    recorder = RecordRobotData()  
    recorder.run()   

if __name__ == '__main__':
    main()  

