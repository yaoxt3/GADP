#!/usr/bin/env python3
#coding=utf-8

import rospy
from std_msgs.msg import Float32, Int16, Bool
from action_infer.msg import ObsCache, TwistGripper
from franky import *
import hydra
import pathlib
import sys
sys.path.append('/home/yxt/thesis/yirui/imitation/3D-Diffusion-Policy/3D-Diffusion-Policy')
import os
import torch
import pytorch3d.ops as torch3d_ops
import torchvision
import numpy as np
import pyrealsense2 as rs
from diffusion_policy_3d.policy.dp3 import DP3
from omegaconf import OmegaConf
from hydra.core.hydra_config import HydraConfig
from diffusion_policy_3d.common.pytorch_util import dict_apply
from termcolor import cprint
import dill
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pytorch3d.ops as torch3d_ops
import threading
import matplotlib.pyplot as plt
import signal
import pickle
from collections import deque
from pynput import keyboard
import re

def preproces_image(image):
    img_size = 84
    image = image.astype(np.float32)
    image = torch.from_numpy(image).cuda()
    image = image.permute(2, 0, 1) # HxWx3 -> 3xHxW
    image = torchvision.transforms.functional.resize(image, (img_size, img_size))
    image = image.permute(1, 2, 0) # 3xHxW -> HxWx3
    image = image.cpu().numpy()
    return image

def farthest_point_sampling(points, num_points=1024, use_cuda=True):
    K = [num_points]
    if use_cuda:
        points = torch.from_numpy(points).cuda()
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.cpu().numpy()
    else:
        points = torch.from_numpy(points)
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.numpy()

    return sampled_points, indices

def preprocess_point_cloud(points, use_cuda=True):
    num_points = 2048
    # Define the extrinsic matrices
    extrinsics_matrix = np.array([[ 0.03909669,  0.37761151, -0.92513836,  1.14677914],
 [ 0.99880883, -0.04181959,  0.02514063, -0.05547239],
 [-0.02919552, -0.92501928, -0.37879672,  0.31100034],
 [ 0.,          0.,          0.,          1.        ]])

    point_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    point_homogeneous = np.dot(extrinsics_matrix, point_homogeneous.T).T
    # point_homogeneous = np.dot(point_homogeneous, extrinsics_matrix)
    points = point_homogeneous[..., :-1]
    points, sample_indices = farthest_point_sampling(points, num_points, use_cuda)
    return points

OmegaConf.register_new_resolver("eval", eval, replace=True)
dir = '/home/yxt/thesis/yirui/imitation/3D-Diffusion-Policy/3D-Diffusion-Policy/data/outputs/pick_place-0112-simple_dp3_mm-8000-datav123-dot'
chkpt = dir + '/checkpoints/latest.ckpt'
device = torch.device("cuda:0")

class FixedSizeQueue:
    def __init__(self, size):
        self.queue = deque(maxlen=size)
        self.size = size

    def enqueue(self, item):
        self.queue.append(item)

    def dequeue(self):
        if len(self.queue) == 0:
            print("Queue is empty.")
            return None
        return self.queue.popleft()
    
    def get_item(self, idx):
        return self.queue[idx]
    
    def __str__(self):
        return str(list(self.queue))


class ActionInfer:
    include_keys = ['global_step', 'epoch']
    exclude_keys = tuple()

    def __init__(self, cfg: OmegaConf):
        self.queue_size = 2
        self.cimg_queue = FixedSizeQueue(self.queue_size)
        self.pc_queue = FixedSizeQueue(self.queue_size)
        self.pose_queue = FixedSizeQueue(self.queue_size)
        self.obs_full = False
        self.current_pose = None
        self.record_pose = None
        self.br = CvBridge()

        ## realsense camera
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

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # first frame is not used
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        print("First frame captured")

        self.workspace = [(-0.32, 0.25), (-0.36, 0.36), (0.6, 1.2)]

        self.img1 = None
        self.img2 = None
        self.pc1 = None
        self.pc2 = None

        self.config = cfg
        self.horizon = self.config.horizon
        self.n_action_steps = self.config.n_action_steps
        self.n_obs_steps = self.config.n_obs_steps

        self.new_map_received = False
        self.new_obs_received = False

        self.output_dir = dir
        self.gripper_pub = rospy.Publisher('/franka_gripper_command', Int16, queue_size=10)
        self.action_pub = rospy.Publisher('/franka_twist_gripper', TwistGripper, queue_size=10)
        self.obs_sub = rospy.Subscriber('observation_data', ObsCache, self.obs_callback_with_queue, queue_size=1)
        self.map_sub = rospy.Subscriber('/certainty_map', Image, self.map_callback_with_queue, queue_size=1)
        self.action_done_sub = rospy.Subscriber('/action_done', Bool, self.action_done_callback, queue_size=1)

        
        rospy.init_node('action_infer', anonymous=True)

        self.model: DP3 = hydra.utils.instantiate(cfg.policy)
        self.end_effector_length = 0.0625
        self.ema_model: DP3 = None
        if cfg.training.use_ema:
            try:
                self.ema_model = copy.deepcopy(self.model)
            except: # minkowski engine could not be copied. recreate it
                self.ema_model = hydra.utils.instantiate(cfg.policy)
        
        # configure training state
        self.optimizer = hydra.utils.instantiate(
            cfg.optimizer, params=self.model.parameters())

        lastest_ckpt_path = self.get_checkpoint_path(tag="latest")
        if lastest_ckpt_path.is_file():
            cprint(f"Resuming from checkpoint {lastest_ckpt_path}", 'magenta')
            self.load_checkpoint(path=lastest_ckpt_path)

        self.latest_obs = {
            'agent_pos': None,
            'certainty_map': None,
            'point_cloud': None
        }
        
        self.height_changes = []
        self.time_changes = []
        self.start_time = rospy.get_time()
        self.adopt_flag = False
        self.action_done_event = threading.Event()
        self.require_new_obs = threading.Event()
        self.obs_lock = threading.Lock()

        self.traj ={
            'agent_pos': [],
            'time': [],
            'adopt_flag': [],
        }

        self.traj_output_dir = '/home/yxt/thesis/yirui/catkin_ws/experiment_imgs/0.14_cube'
        self.file_index = self.get_next_file_index()
        self.keyboard_listener = keyboard.Listener(on_press=self.save_on_press)
        self.keyboard_listener.start()
    
    def depth_to_pointcloud_wo_color(self, depth_frame, workspace, pc):
        points = pc.calculate(depth_frame)
        points = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        points = points[np.where((points[..., 0] > workspace[0][0]) & (points[..., 0] < workspace[0][1]) &
                        (points[..., 1] > workspace[1][0]) & (points[..., 1] < workspace[1][1]) &
                        (points[..., 2] > workspace[2][0]) & (points[..., 2] < workspace[2][1]))]
        return points

    def map_callback_with_queue(self, msg): # manage the certainty map data via queue
        image = self.br.imgmsg_to_cv2(msg)
        self.cimg_queue.enqueue(image)
        if len(self.cimg_queue.queue) == self.queue_size:
            self.new_map_received = True
        else:
            self.new_map_received = False
            
    def action_done_callback(self, msg):
        action_done = msg.data
        if action_done:
            self.action_done_event.set()
    
    def save_on_press(self, key):
        if key.char == 's':
            self.save_graph()
            self.save_traj()
            rospy.signal_shutdown('Graph and traj saved')
            return
    
    def get_next_file_index(self):
        pattern = re.compile(rf'recorded_traj(\d+)gripper{self.end_effector_length}\.pkl')
        existing_files = os.listdir(self.traj_output_dir)
        count = sum(1 for f in existing_files if pattern.search(f))
        return count + 1

    def obs_callback_with_queue(self, msg): # manage the point cloud data via queue
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            rospy.loginfo("Depth frame is None")
            return
        pc = rs.pointcloud()
        points = self.depth_to_pointcloud_wo_color(depth_frame, self.workspace, pc)
        self.pc_queue.enqueue(points)
        pose1 = np.array(msg.pose_1).reshape(-1, 7)
        pose2 = np.array(msg.pose_2).reshape(-1, 7)
        self.current_pose = np.concatenate((pose1, pose2), axis=0).reshape(1, 2, 7)
        self.record_pose = self.current_pose.copy()
        if len(self.pc_queue.queue) == self.queue_size:
            self.obs_full = True
            self.require_new_obs.set()
        else:
            self.obs_full = False

    def load_checkpoint(self, path=None, tag='latest',
            exclude_keys=None, 
            include_keys=None, 
            **kwargs):
        if path is None:
            path = self.get_checkpoint_path(tag=tag)
        else:
            path = pathlib.Path(path)
        payload = torch.load(path.open('rb'), pickle_module=dill, map_location='cpu')
        self.load_payload(payload, 
            exclude_keys=exclude_keys, 
            include_keys=include_keys)
        return payload
    
    def load_payload(self, payload, exclude_keys=None, include_keys=None, **kwargs):
        if exclude_keys is None:
            exclude_keys = tuple()
        if include_keys is None:
            include_keys = payload['pickles'].keys()

        for key, value in payload['state_dicts'].items():
            if key not in exclude_keys:
                self.__dict__[key].load_state_dict(value, **kwargs)
        for key in include_keys:
            if key in payload['pickles']:
                self.__dict__[key] = dill.loads(payload['pickles'][key])
    
    def get_checkpoint_path(self, tag='latest'):
        if tag=='latest':
            return pathlib.Path(self.output_dir).joinpath('checkpoints', f'{tag}.ckpt')
        elif tag=='best': 
            # the checkpoints are saved as format: epoch={}-test_mean_score={}.ckpt
            # find the best checkpoint
            checkpoint_dir = pathlib.Path(self.output_dir).joinpath('checkpoints')
            all_checkpoints = os.listdir(checkpoint_dir)
            best_ckpt = None
            best_score = -1e10
            for ckpt in all_checkpoints:
                if 'latest' in ckpt:
                    continue
                score = float(ckpt.split('test_mean_score=')[1].split('.ckpt')[0])
                if score > best_score:
                    best_ckpt = ckpt
                    best_score = score
            return pathlib.Path(self.output_dir).joinpath('checkpoints', best_ckpt)
        else:
            raise NotImplementedError(f"tag {tag} not implemented")
    
    def save_graph(self):
        plt.figure()
        plt.plot(self.time_changes, self.height_changes)
        plt.xlabel('Time (s)')
        plt.ylabel('Height (m)')
        plt.title(f'Height Changes Over Time - Model: {self.model.__class__.__name__}')
        graph_output_path = os.path.join(self.traj_output_dir, f'height_time_graph{self.file_index}.png')
        print(f"Saving traj graph to {graph_output_path}")
        plt.savefig(graph_output_path)
        plt.close()
    
    def save_traj(self):
        traj_output_path = os.path.join(self.traj_output_dir, f'recorded_traj{self.file_index}gripper{self.end_effector_length}.pkl')
        print(f"Saving traj to {traj_output_path}")
        with open(traj_output_path, 'wb') as f:
            pickle.dump(self.traj, f)
    
    ## process the latest observation at 5Hz
    def lastest_obs_processing(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.obs_full and self.new_map_received:
                img1 = preproces_image(self.cimg_queue.get_item(0)) # the oldest image
                img2 = preproces_image(self.cimg_queue.get_item(1)) # the newest image
                img1 = torch.from_numpy(img1).unsqueeze(0)
                img2 = torch.from_numpy(img2).unsqueeze(0)
                certainty_img = torch.concat((img1, img2), axis=0).reshape(1, 2, 84, 84, 3).to(device)
                
                pc1 = preprocess_point_cloud(self.pc_queue.get_item(0)).reshape(1, -1, 3)
                pc2 = preprocess_point_cloud(self.pc_queue.get_item(1)).reshape(1, -1, 3)
                pc1 = torch.from_numpy(pc1)
                pc2 = torch.from_numpy(pc2)
                point_cloud = torch.concat((pc1, pc2), axis=0).reshape(1, 2, -1, 3).to(device)
                
                self.latest_obs = {
                'agent_pos': self.current_pose,
                'certainty_map': certainty_img,
                'point_cloud': point_cloud
                }

                x_min, y_min = 0.48, -0.134
                x_max, y_max = 0.69, 0.0473
                z_max = 0.08

                x, y, z = self.current_pose[0, -1, 0], self.current_pose[0, -1, 1], self.current_pose[0, -1, 2] # x and y at the last timestep
                if x_min <= x <= x_max and y_min <= y <= y_max and z - self.end_effector_length < z_max:
                    self.adopt_flag = True

                self.time_changes.append(rospy.get_time() - self.start_time)
                self.height_changes.append(self.record_pose[0, -1, 2])
                self.traj['agent_pos'].append(self.record_pose[0, -1, :])
                self.traj['adopt_flag'].append(self.adopt_flag)
                
                with self.obs_lock:
                    self.new_obs_received = True
                rate.sleep()
                
    
    def run_with_queue_data(self):
        max_action_steps = 200
        executed_action_steps = 0
        exec_start_time = rospy.Time.now()
        gripper_command = Int16()
        while not rospy.is_shutdown():
            if self.new_obs_received:
                with self.obs_lock:
                    self.new_obs_received = False
                rospy.loginfo("Predicting action")
                with torch.no_grad():
                    action_dict, _ = self.model.predict_action(self.latest_obs)
                    
                
                np_action_dict = dict_apply(action_dict,
                                        lambda x: x.detach().to('cpu').numpy())
                
                exec_actions = np_action_dict['action'].squeeze(0)
                elapsed_time = (rospy.Time.now() - exec_start_time).to_sec()
                sleep_time = max(0, 0.5 - elapsed_time)
                rospy.sleep(sleep_time)
                
                # for i in range(len(exec_actions)):
                for i, action in enumerate(exec_actions): # use enumerate to instead of range(len())
                    if executed_action_steps >= max_action_steps:
                        rospy.loginfo("Maximum action steps reached. Shutting down node.")
                        rospy.signal_shutdown("Maximum action steps reached.")
                        break
                    start_time = rospy.Time.now()
                    
                    # action = exec_actions[i]
                    twist_gripper_msg = TwistGripper()
                    twist_gripper_msg.twist.linear.x = action[0]
                    twist_gripper_msg.twist.linear.y = action[1]
                    twist_gripper_msg.twist.linear.z = action[2]
                    twist_gripper_msg.twist.angular.x = action[3]
                    twist_gripper_msg.twist.angular.y = action[4]
                    twist_gripper_msg.twist.angular.z = action[5]
                    gripper_array = np.array([action[6], action[7]])

                    if abs(action[6] - action[7]) >= 0.5:
                        gripper_command.data = int(np.argmax(gripper_array))
                    else:
                        gripper_command.data = 2  # Publish the same value
                    self.gripper_pub.publish(gripper_command) 

                    self.action_pub.publish(twist_gripper_msg)
                    rospy.loginfo(f"Action {i} executed: {action}")
                    executed_action_steps += 1

                    if i == len(exec_actions) - 1:
                        if not self.action_done_event.wait(timeout=5):
                            rospy.logwarn("Action done event timeout. Ending script.")
                            rospy.signal_shutdown("Action done event timeout.")
                            return
                        self.action_done_event.clear()
                        exec_start_time = rospy.Time.now()
                        self.require_new_obs.clear()
                        if not self.require_new_obs.wait(timeout=1):
                            rospy.logwarn("Arequire new obs event timeout. Ending script.")
                            rospy.signal_shutdown("require new obs event timeout.")
                            return
                    else:
                        if not self.action_done_event.wait(timeout=5):
                            rospy.logwarn("Action done event timeout. Ending script.")
                            rospy.signal_shutdown("Action done event timeout.")
                            return
                        self.action_done_event.clear()
                        elapsed_time = (rospy.Time.now() - start_time).to_sec()
                        sleep_time = max(0, 0.5 - elapsed_time)
                        rospy.sleep(sleep_time)
                
                

@hydra.main(version_base=None, config_path="/home/yxt/thesis/yirui/imitation/3D-Diffusion-Policy/3D-Diffusion-Policy/diffusion_policy_3d/config", config_name="simple_dp3_mm_proj_flag")
def main(cfg):
    try:
        action_infer = ActionInfer(cfg)
        thread1 = threading.Thread(target=action_infer.run_with_queue_data)
        thread2 = threading.Thread(target=action_infer.lastest_obs_processing)
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()