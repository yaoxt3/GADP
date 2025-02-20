#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from franky import *
import franky
import threading
from std_msgs.msg import Int16, Float32
from teng_arduino.msg import TwistGripper
import pyrealsense2 as rs
import numpy as np
import pickle
import copy
import traceback
import os
from scipy.spatial.transform import Rotation
import open3d as o3d

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
    points = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
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
    
def visualize_pointcloud(pointcloud_6d):
    """
    Visualize a 3D point cloud (XYZ) using Open3D.
    :param pointcloud_3d: A numpy array of shape (N, 3), where each row is [X, Y, Z].
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_6d[:, :3])  # XYZ coordinates
    o3d.visualization.draw_geometries([pcd])
    
def quaternion_to_euler(q):
    
    q_x, q_y, q_z, q_w = q[0], q[1], q[2], q[3]
    roll = np.arctan2(2.0 * (q_w * q_x + q_y * q_z), 1.0 - 2.0 * (q_x**2 + q_y**2))
    pitch = np.arcsin(2.0 * (q_w * q_y - q_z * q_x))
    yaw = np.arctan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y**2 + q_z**2))

    return roll, pitch, yaw

class FrankaRobotController:
    def __init__(self):
        rospy.init_node('franka_robot_controller')
        robot_ip = rospy.get_param('~robot_ip')
        self.robot = Robot(robot_ip)
        self.robot.recover_from_errors()
        self.robot.relative_dynamics_factor = 0.05
        self.robot.velocity_rel = 0.4
        self.robot.acceleration_rel = 0.2
        self.robot.jerk_rel = 0.02
        self.gripper = franky.Gripper(robot_ip)
        self.gripper_speed = 0.02
        self.gripper_force = 5
        print('Connecting to robot at {}'.format(robot_ip))
        
        target_positions = [0.2, 0, -0.28, -2.63, -0.03, 2.61, 0.75]
        joint_state_target = JointState(position=target_positions)
        waypoint = JointWaypoint(target=joint_state_target, reference_type=ReferenceType.Absolute)
        init_motion = JointWaypointMotion([waypoint])
        self.robot.move(init_motion)
        
        # self.twist_sub = rospy.Subscriber('/franka_twist', Twist, self.twist_callback)
        self.gripper_sub = rospy.Subscriber('/franka_gripper_command', Int16, self.gripper_open_callback, queue_size=1)
        self.twist_gripper_sub = rospy.Subscriber('/franka_twist_gripper', TwistGripper, self.twist_gripper_callback, queue_size=1)        
        self.translation = [0, 0, 0]
        self.angular_y = 0.0
        self.movement = [0, 0, 0, 0]
        self.new_command_received = False
        self.new_grippr_command_received = False
        self.gripper_cnt = 0
        self.gripper_command = 0
        self.max_gripper_width = 0.08
        
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.workspace = [(-0.35, 0.25),
                          (-0.36, 0.36),
                          (0.61, 1)]
        
        # self.workspace = [(-0.18, 0.18),
        #                   (-0.36, 0.36),
        #                   (0.61, 1)]
        
        self.data_cache = {
            'point_cloud': [],
            'agent_pose': [],
            'action': []
        }
        
        self.gripper_move = -0.1
        self.gripper_open = False
        
        self.lock = threading.Lock()
        
        self.control_thread = threading.Thread(target=self.robot_control_loop)
        self.control_thread.start()
        
        self.record_thread = threading.Thread(target=self.robot_record_loop)
        self.record_thread.start()
        
        self.gripper_thread = threading.Thread(target=self.gripper_control_loop)
        self.gripper_thread.start()
        
    def twist_gripper_callback(self, msg):
        with self.lock:
            self.movement = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.y]
            self.gripper_command = msg.gripper
            self.new_command_received = True
    
    def gripper_open_callback(self, msg):
        with self.lock:
            if msg.data == 1 and self.gripper_open == False:
                self.gripper_open = True
                self.new_grippr_command_received = True
            elif msg.data == 2 and self.gripper_open == True:
                self.gripper_open = False
                self.new_grippr_command_received = True
    
    # def save_data(self, directory = '/home/tumi6/yxt/pkl_data/panda_pick_color'):
    def save_data(self, directory = '/home/tumi6/yxt/pkl_data/10-31/panda_pick_455_1'):    
        cnt = len(os.listdir(directory))
        path = os.path.join(directory, 'traj'+str(cnt)+'.pkl')
        print(path)
        with open(path, 'wb') as f:
            pickle.dump(self.data_cache, f)
        
    def robot_control_loop(self):
        rate = rospy.Rate(2) 
        while not rospy.is_shutdown():
            with self.lock:
                if self.new_command_received:
                    movement = self.movement
                    self.new_command_received = False
                else:
                    movement = [0, 0, 0, 0]
                            
            if any(movement):
                translation = np.array([self.movement[0], self.movement[1], self.movement[2]])
                quat = Rotation.from_euler("xyz", [0, self.movement[3], 0]).as_quat()
                way = Affine(translation, quat)
                motion_forward = CartesianMotion(way, ReferenceType.Relative)
                self.robot.move(motion_forward, asynchronous=True)
                
            rate.sleep()
            
    def robot_record_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
                    
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            pc = rs.pointcloud()  
            intrinsic = depth_frame.profile.as_video_stream_profile().intrinsics
            # points = depth_to_pointcloud_with_color(depth_frame, color_frame, pc, intrinsic, self.workspace)
            points = depth_to_pointcloud_wo_color(depth_frame, self.workspace,pc)
            cartesian_state = self.robot.current_cartesian_state
            pose = cartesian_state.pose
            self.gripper_width = self.gripper.width

            x, y, z = pose.end_effector_pose.translation.flatten()
            q = pose.end_effector_pose.quaternion.flatten()
            roll, pitch, yaw = quaternion_to_euler(q)
            pose_arr = np.array([x, y, z, roll, pitch, yaw, self.gripper_width], dtype=np.float32)
            
            #visualize_pointcloud(points)
            # visualize_6d_pointcloud(points)
            
            action = np.array([self.movement[0], self.movement[1], self.movement[2], 0, self.movement[3], 0, self.gripper_open], dtype=np.float32) 
            print(points.shape)
            print(action)        
            with self.lock:    
                self.data_cache['point_cloud'].append(copy.deepcopy(points))
                self.data_cache['action'].append(copy.deepcopy(action))
                self.data_cache['agent_pose'].append(copy.deepcopy(pose_arr)) 
            
            rate.sleep()
            
    def gripper_control_loop(self):
        rate = rospy.Rate(2)
        #current_future = None
        while not rospy.is_shutdown():
            with self.lock:
                if self.new_grippr_command_received:
                    current_gripper_move = self.gripper_open

            if self.new_grippr_command_received:
                if current_gripper_move:
                    current_future = self.gripper.open_async(self.gripper_speed)
                elif not current_gripper_move:
                    current_future = self.gripper.grasp_async(0.03, self.gripper_speed, self.gripper_force, epsilon_outer=0.2)
            
            #if self.new_grippr_command_received:
                #if current_gripper_move and (current_future is None or current_future.wait(0)):
                    #current_future = self.gripper.open_async(self.gripper_speed)
                #elif not current_gripper_move and (current_future is None or current_future.wait(0)):
                    #current_future = self.gripper.grasp_async(0.03, self.gripper_speed, self.gripper_force, epsilon_outer=0.2)

            #if current_future is not None:
                #if current_future.wait(0.1):
                    #try:
                        #success = current_future.get()
                    #except Exception as e:
                        #rospy.logerr(f"Gripper operation failed: {e}")
                    #finally:
                        #current_future = None
            
            with self.lock:
                self.new_grippr_command_received = False
            rate.sleep()

    def run(self):
        try:
            #translation = np.array([[0.0], [0.1], [0.0]])
            #quaternion = np.array([[0.0], [0.0], [0.0], [1.0]])
            #way = Affine(translation=translation, quaternion=quaternion)
            #motion_forward = CartesianMotion(way, ReferenceType.Relative)
            #self.robot.move(motion_forward, asynchronous=True)
            #motion_backward = CartesianMotion(way.inverse, ReferenceType.Relative)
            #self.robot.move(motion_backward, asynchronous=True)
            self.gripper.move_async(self.gripper.max_width/2, self.gripper_speed)
            print('Gripper opened: {}'.format(self.gripper.max_width/2))
            rospy.spin()
        finally:
            self.save_data()

if __name__ == '__main__':
    controller = FrankaRobotController()
    controller.run()