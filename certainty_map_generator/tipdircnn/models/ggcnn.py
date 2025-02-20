"""
From official repository:
https://github.com/dougsm/ggcnn
Paper:
http://www.roboticsproceedings.org/rss14/p21.pdf
"""

import torch.nn as nn
import torch.nn.functional as F


filter_sizes = [32, 16, 8, 8, 16, 32]
kernel_sizes = [9, 5, 3, 3, 5, 9]
strides = [3, 2, 2, 2, 2, 3]


class GGCNN_standard(nn.Module):
    """
    GG-CNN without considering quality and width, but cos / sin of dir.
    """
    def __init__(self, input_channels=1, output_channels=4, structure_type='ggcnn_std'):
        super().__init__()
        # self.input_channels = input_channels
        self.output_channels = output_channels
        self.structure_type = structure_type
        self.conv1 = nn.Conv2d(input_channels, filter_sizes[0], kernel_sizes[0], stride=strides[0], padding=3)
        self.conv2 = nn.Conv2d(filter_sizes[0], filter_sizes[1], kernel_sizes[1], stride=strides[1], padding=2)
        self.conv3 = nn.Conv2d(filter_sizes[1], filter_sizes[2], kernel_sizes[2], stride=strides[2], padding=1)
        self.convt1 = nn.ConvTranspose2d(filter_sizes[2], filter_sizes[3], kernel_sizes[3], stride=strides[3], padding=1, output_padding=1)
        self.convt2 = nn.ConvTranspose2d(filter_sizes[3], filter_sizes[4], kernel_sizes[4], stride=strides[4], padding=2, output_padding=1)
        self.convt3 = nn.ConvTranspose2d(filter_sizes[4], filter_sizes[5], kernel_sizes[5], stride=strides[5], padding=3, output_padding=1)
        
        self.conv_heads = nn.ModuleList()
        for _ in range(self.output_channels):
            self.conv_heads.append(nn.Conv2d(filter_sizes[5], 1, kernel_size=2))

        for m in self.modules(): # weight initialization
            if isinstance(m, (nn.Conv2d, nn.ConvTranspose2d)):
                nn.init.xavier_uniform_(m.weight, gain=1)
    
    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.convt1(x))
        x = F.relu(self.convt2(x))
        x = F.relu(self.convt3(x))

        head_ouputs = []
        for ind in range(self.output_channels):
            head_ouputs.append(self.conv_heads[ind](x))

        return head_ouputs


class GGCNN(nn.Module):
    """
    GG-CNN
    Equivalient to the Keras Model used in the RSS Paper (https://arxiv.org/abs/1804.05172)
    """
    def __init__(self, input_channels=1):
        super().__init__()
        # self.input_channels = input_channels
        self.conv1 = nn.Conv2d(input_channels, filter_sizes[0], kernel_sizes[0], stride=strides[0], padding=3)
        self.conv2 = nn.Conv2d(filter_sizes[0], filter_sizes[1], kernel_sizes[1], stride=strides[1], padding=2)
        self.conv3 = nn.Conv2d(filter_sizes[1], filter_sizes[2], kernel_sizes[2], stride=strides[2], padding=1)
        self.convt1 = nn.ConvTranspose2d(filter_sizes[2], filter_sizes[3], kernel_sizes[3], stride=strides[3], padding=1, output_padding=1)
        self.convt2 = nn.ConvTranspose2d(filter_sizes[3], filter_sizes[4], kernel_sizes[4], stride=strides[4], padding=2, output_padding=1)
        self.convt3 = nn.ConvTranspose2d(filter_sizes[4], filter_sizes[5], kernel_sizes[5], stride=strides[5], padding=3, output_padding=1)

        self.pos_conv = nn.Conv2d(filter_sizes[5], 1, kernel_size=2)
        self.cos_conv = nn.Conv2d(filter_sizes[5], 1, kernel_size=2)
        self.sin_conv = nn.Conv2d(filter_sizes[5], 1, kernel_size=2)
        self.wid_conv = nn.Conv2d(filter_sizes[5], 1, kernel_size=2)

        for m in self.modules():
            if isinstance(m, (nn.Conv2d, nn.ConvTranspose2d)):
                nn.init.xavier_uniform_(m.weight, gain=1)
    
    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.convt1(x))
        x = F.relu(self.convt2(x))
        x = F.relu(self.convt3(x))

        pos_output = self.pos_conv(x)
        cos_output = self.cos_conv(x)
        sin_output = self.sin_conv(x)
        wid_output = self.wid_conv(x)
        # pos_output = F.sigmoid(self.pos_conv(x))
        # cos_output = F.tanh(self.cos_conv(x))
        # sin_output = F.tanh(self.sin_conv(x))
        # wid_output = F.sigmoid(self.wid_conv(x))

        return pos_output, cos_output, sin_output, wid_output

    def compute_loss(self, xc, yc):
        y_pos, y_cos, y_sin, y_wid = yc
        pred_pos, pred_cos, pred_sin, pred_wid = self(xc)

        pos_loss = F.mse_loss(pred_pos, y_pos)
        cos_loss = F.mse_loss(pred_cos, y_cos)
        sin_loss = F.mse_loss(pred_sin, y_sin)
        wid_loss = F.mse_loss(pred_wid, y_wid)


        return{
            'loss': pos_loss + cos_loss + sin_loss + wid_loss,
            'losses': {
                'pos_loss': pos_loss,
                'cos_loss': cos_loss,
                'sin_loss': sin_loss,
                'wid_loss': wid_loss,
            },
            'pred': {
                'pos': pred_pos,
                'cos': pred_cos,
                'sin': pred_sin,
                'wid': pred_wid
            }
        }


