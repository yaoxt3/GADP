import sys
import numpy as np
import torch

sys.path.append('../tipdircnn')

from utils.dataset.misc import load_testset
from utils.augment import param_mana, tta_process
from utils.process import grasp_definition, network_evaluation


class CNNPredictor:
    def __init__(self, network, input_chans, label_type, tta_size=4, img_size=300):
        self.img_size = img_size
        self.label_type = label_type
        self.tta_size = tta_size
        self.input_channels = input_chans
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print('Torch Using device:', self.device)
        # Load network
        self.net = torch.load(network)
        self.net.eval()
        from torchsummary import summary
        summary(self.net, (input_chans, img_size, img_size))
        label_id_str, self.label_map = param_mana.analysis_labels(self.label_type)
        print('(input_chans, img_size, img_size):', (input_chans, img_size, img_size), \
            '\nlabel_id_str, self.label_map:', label_id_str, self.label_map)

    def inference(self, input):
        # input ([1/3/4], 300, 300)
        x_arr, tta_belta = tta_process.TTA_input_generation(input, self.tta_size)
        x_arr_tensor = torch.from_numpy(x_arr).to(self.device)

        # Inference on TTA inputs
        post_preds = network_evaluation.post_process_output(self.net(x_arr_tensor), self.label_map)
        # Gether aug. params/inputs/preds
        tta_samples = tta_process.TTA_organize_data(tta_belta, x_arr, post_preds)

        # TTA modeling
        tta_estimation, regulated_sample_data = tta_process.TTA_modeling_std(tta_samples, self.label_map)

        # Fetch prediction
        (qua_vote, ang_vote, wid_vote), grasp_type = \
            param_mana.decode_grasp_detection(tta_estimation, self.label_map, (None, None))

        return qua_vote, ang_vote


    def input_process(self, input_o, direct_return=True, chann = 1):
        """
        Standardize and normalize input to fit the network input
        Crop to 300 x 300 should be done before sent here:
        self.img = self.img[top_left[0]:botton_right[0],top_left[1]:botton_right[1]]
        """
        input = input_o.copy()
        # noise = np.random.normal(0, .01, input.shape).astype(input.dtype)
        # input = input + noise

        if direct_return: return input
        if chann == 1: # depth
            input[0] = np.clip((input[0] - np.mean(input[0])), -1, 1)
        elif chann == 3: # rgb
            input = input/255.0
            input -= np.mean(input)
        elif chann == 4: # depth + rgb
            # depth
            input[0] = np.clip((input[0] - np.mean(input[0])), -1, 1)
            # rgb
            input[1:4] = input[1:4]/255.0
            input[1:4] -= np.mean(input[1:4])
        else:
            raise ValueError('input is wrong: {}'.format(input.shape))
        return input[0:chann]


