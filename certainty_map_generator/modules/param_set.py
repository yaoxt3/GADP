class params_set:
    def __init__(self):
        ## EFFFPN
        self.network = './tipdircnn/weights/efffpn_std/efffpn_dirtip.model'
        # self.network = '../tipdircnn/weights/efffpn_std/efffpn_dircen.model'
 
        ## GGCNN
        # self.network = '../tipdircnn/weights/ggcnn_std/ggcnn_dirtip.model'
        # self.network = '../tipdircnn/weights/ggcnn_std/ggcnn_dircen.model'

        self.mode = 'dataset' # 'online'[ip_bias -> 0] or 'dataset' [ip_bias -> 1-3]
        self.ip_bias = 1
        self.receive_channel = 4

        self.use_rgb = 1
        self.tta_size = 8
        self.use_depth = 1
        self.label_type = 'dirtip'

        # self.try_times = 4 # 4
        # self.iterate_times = 1 # 200 for testset 10
        
        self.test_epoches = 1
        self.dataset = 'cornell'
        self.dataset_path = './tipdircnn/dataset/cornell/'
        self.split = 0.9 # 0.9
        self.ds_rotate = 0.0
        self.img_size = 300
        self.augment = False
        self.shuffle = False
        self.num_workers = 4
        # BubbleG_1F
        # Robotiq_2F, FishGri_2F, RochuGr_2F
        # HybridG_3F, FishGri_3F, RochuGr_3F # Robotiq_3F
        # RochuGr_4F, FishGri_4F
        self.gripper_type = "Robotiq_2F"
        
    @property
    def gripper_category(self):
        """
        BubbleG_1F, Robotiq_2F, FishGri_2F, RochuGr_2F, HybridG_3F, RochuGr_3F, FishGri_3F, Robotiq_3F, FishGri_4F, RochuGr_4F
        Gripper_1F, Gripper_2F, Gripper_3F, Gripper_RQ, Gripper_4F
        """
        gripper_banks = {
            'BubbleG_1F': 'Gripper_1F',
            'Robotiq_2F': 'Gripper_2F',
            'FishGri_2F': 'Gripper_2F',
            'RochuGr_2F': 'Gripper_2F',
            'HybridG_3F': 'Gripper_3F',
            'RochuGr_3F': 'Gripper_3F',
            'FishGri_3F': 'Gripper_3F',
            'Robotiq_3F': 'Gripper_RQ',
            'FishGri_4F': 'Gripper_4F',
            'RochuGr_4F': 'Gripper_4F',
        }
        return gripper_banks[self.gripper_type]
     
    @property
    def input_channel(self):
        return self.use_rgb * 3 + self.use_depth
    
    @property
    def result_dir(self):
        self.result_dir_base = './output/generation_testset'
        if self.mode == 'dataset': return str(self.result_dir_base + '_' + str(self.gripper_type))
        elif self.mode == 'online': return './output/generation_online'
        else: raise KeyError('mode is wrong: {}'.format(self.mode))

    @staticmethod
    def qaw_to_tipspace(q_img, angle_img, width_img, density):
        """
        Conver the tip quality, angle, width image to 2d tipspace map
        :param q_img: as tipspace transparency
        :param angle_img: as tipspace angle
        :param width_img: as tipspace wipe distance
        :return: tipspace
        """
        import numpy as np
        H_img = q_img.shape[0]
        W_img = q_img.shape[1]
        tipspace_map = []
        for dir_y in np.arange(0, H_img, density, dtype=np.int16):
            for dir_x in np.arange(0, W_img, density, dtype=np.int16):
                start = np.array([dir_y, dir_x])
                width = width_img[dir_y][dir_x] + 10
                angle = angle_img[dir_y][dir_x]
                dir_a = width * np.array([-np.sin(angle), np.cos(angle)])

                end = np.clip(start + dir_a, 0, 300)
                transparent = np.clip(q_img[dir_y][dir_x], 0.0, 1.0) #  + 0.5

                tipspace_map.append((np.vstack((start, end)), transparent))
                # print(tipspace_map[-1])
        return tipspace_map