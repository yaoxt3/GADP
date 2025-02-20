import numpy as np
import random

import torch
import torch.utils.data


class GraspDatasetBase(torch.utils.data.Dataset):
    """
    Class base for different dataset, include cornell and others.
    """
    def __init__(self, label_map, output_size=300, include_depth=True, include_rgb=False, 
                random_rotate=False, random_zoom=False):
        """
        :param output_size: Image output size in pixels (square)
        :param include_depth: Whether depth image is included
        :param include_rgb: Whether RGB image is included
        :param random_rotate: Whether random rotations are applied
        :param random_zoom: Whether random zooms are applied
        :param label_map: labels are label_map
        """
        self.output_size = output_size
        self.random_rotate = random_rotate
        self.random_zoom = random_zoom
        self.include_depth = include_depth
        self.include_rgb = include_rgb
        self.label_map = label_map
        self.WIDTH_MAX = 150.0


        self.data_files = []

        if include_depth is False and include_rgb is False:
            raise ValueError('Please indicate depth or rgb data to load')

    def get_bbs(self, idx, rot=0, zoom=1.0):
        """
        Should be implemented by the derived class 

        :param int idx: index of data
        :param int rot: rotation of the data, defaults to 0
        :param float zoom: zoom factor, defaults to 1.0
        :raises NotImplementedError: None
        """
        raise NotImplementedError()

    def get_depth(self, idx, rot=0, zoom=1.0):
        raise NotImplementedError()

    def get_rgb(self, idx, rot=0, zoom=1.0):
        raise NotImplementedError()

    @staticmethod
    def numpy_to_torch(map):
        if map is None: return None

        if(len(map.shape) == 2):
            return torch.from_numpy(np.expand_dims(map, 0).astype(np.float32))
        else:
            return torch.from_numpy(map.astype(np.float32))

    def __getitem__(self, idx):
        if self.random_rotate:
            rotations = [0, np.pi/2, 2*np.pi/2, 3*np.pi/2]
            rot = random.choice(rotations) # choose one of the rotations in array
        else:
            rot = 0.0
        
        if self.random_zoom:
            zoom_factor = np.random.uniform(0.5, 1)
        else:
            zoom_factor = 1.0

        if self.include_depth:
            depth_img = self.get_depth(idx, rot, zoom_factor)

        if self.include_rgb:
            rgb_img = self.get_rgb(idx, rot, zoom_factor)

        if self.include_rgb and self.include_depth:
            x = self.numpy_to_torch(
                np.concatenate(
                    (np.expand_dims(depth_img, 0), # (1, 300, 300)
                    rgb_img), # (3, 300, 300)
                    0 # concatenate in axis 0
                ) # (4, 300, 300)
            )
        elif self.include_depth:
            x = self.numpy_to_torch(depth_img)
        elif self.include_rgb:
            x = self.numpy_to_torch(rgb_img)
        
        bbs = self.get_bbs(idx, rot, zoom_factor)
        
        label_array = {
            'Pos':{ 'Qua':None, 'Ang':None, 'Wid':None },
            'Tip':{ 'Qua':None, 'Ang':None, 'Wid':None },
            'Non':{ 'Qua':None, 'Ang':None, 'Wid':None }
        }
        label_array['Pos']['Qua'], label_array['Pos']['Ang'], label_array['Pos']['Wid'] = \
            bbs.draw_pos_map((self.output_size, self.output_size), pos_ratio = 1.0/3)

        label_array['Tip']['Qua'], label_array['Tip']['Ang'], label_array['Tip']['Wid'] = \
            bbs.draw_tip_map((self.output_size, self.output_size), tip_ratio=1.0/4)
        
        label_array['Pos']['Ang'] = label_array['Pos']['Ang'] * 2 # for position only!!! 
        label_array['Pos']['Wid'] = np.clip(label_array['Pos']['Wid'], 0.0, self.WIDTH_MAX)/self.WIDTH_MAX
        # For tip, the (Tipdir) is width / 4, and length is width * 2
        label_array['Tip']['Wid'] = np.clip(label_array['Tip']['Wid'] / 4.0, 0.0, self.WIDTH_MAX)/self.WIDTH_MAX 

        # For faster tensor loading using loader workers
        _loc = self.numpy_to_torch(label_array[self.label_map['Qua']]['Qua'])
        _cos = self.numpy_to_torch(np.cos(label_array[self.label_map['Cos']]['Ang']))
        _sin = self.numpy_to_torch(np.sin(label_array[self.label_map['Sin']]['Ang']))
        _dis = self.numpy_to_torch(label_array[self.label_map['Wid']]['Wid'])
            
        y_all = (_loc, _cos, _sin, _dis)
        y = tuple(yi for yi in y_all if yi is not None)
        return x, y, idx, rot, zoom_factor

    def __len__(self):
        return len(self.data_files)