"""
https://arxiv.org/abs/1611.05431
official code:
https://github.com/facebookresearch/ResNeXt
implement code:
https://github.com/Hsuxu/ResNeXt/
"""

import torch.nn as nn
import torch.nn.functional as F
from collections import OrderedDict


class BasicBlock(nn.Module):
    def __init__(self, in_planes, bottleneck_width=4, cardinality=32, stride=1, expansion=2):
        super(BasicBlock, self).__init__()
        inner_width = cardinality * bottleneck_width
        self.expansion = expansion
        self.basic = nn.Sequential(OrderedDict(
            [
                ('conv1_0', nn.Conv2d(in_planes, inner_width, 1, stride=1, bias=False)),
                ('bn1', nn.BatchNorm2d(inner_width)),
                ('act0', nn.ReLU()),
                ('conv3_0', nn.Conv2d(inner_width, inner_width, 3, stride=stride, padding=1, groups=cardinality, bias=False)),
                ('bn2', nn.BatchNorm2d(inner_width)),
                ('act1', nn.ReLU()),
                ('conv1_1', nn.Conv2d(inner_width, inner_width * self.expansion, 1, stride=1, bias=False)),
                ('bn3', nn.BatchNorm2d(inner_width * self.expansion))
            ]
        ))
        self.shortcut = nn.Sequential()
        if stride != 1 or in_planes != inner_width * self.expansion:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_planes, inner_width * self.expansion, 1, stride=stride, bias=False)
            )
        self.bn0 = nn.BatchNorm2d(self.expansion * inner_width)

    def forward(self, x):
        out = self.basic(x)
        out += self.shortcut(x)
        out = F.relu(self.bn0(out))
        return out

class ResNeXt(nn.Module):
    def __init__(self, num_blocks, cardinality, bottleneck_width, expansion=2, num_classes=10):
        super(ResNeXt, self).__init__()
        self.cardinality = cardinality
        self.bottleneck_width = bottleneck_width
        self.in_planes = 64
        self.expansion = expansion

        self.conv1 = nn.Conv2d(3, self.in_planes, kernel_size=7, stride=2, padding=3, bias=False)
        self.bn1 = nn.BatchNorm2d(self.in_planes)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)

        self.layer0 = nn.Sequential(self.conv1, self.bn1, self.relu, self.maxpool)
        self.layer1=self._make_layer(num_blocks[0],1)
        self.layer2=self._make_layer(num_blocks[1],2)
        self.layer3=self._make_layer(num_blocks[2],2)
        self.layer4=self._make_layer(num_blocks[3],2)
        self.linear = nn.Linear(self.cardinality * self.bottleneck_width, num_classes)

    def forward(self, x):
        out = F.relu(self.bn0(self.conv0(x)))
        # out = self.pool0(out)
        out = self.layer1(out)
        out = self.layer2(out)
        out = self.layer3(out)
        out = self.layer4(out)
        out = F.avg_pool2d(out, 4)
        out = out.view(out.size(0), -1)
        out = self.linear(out)
        return out

    def _make_layer(self, num_blocks, stride):
        strides = [stride] + [1] * (num_blocks - 1)
        layers = []
        for stride in strides:
            layers.append(BasicBlock(self.in_planes, self.bottleneck_width, self.cardinality, stride, self.expansion))
            self.in_planes = self.expansion * self.bottleneck_width * self.cardinality
        self.bottleneck_width *= 2
        return nn.Sequential(*layers)


def resnext14_4x32d():
    return ResNeXt(num_blocks=[1, 1, 1, 1], cardinality=4, bottleneck_width=8)

def resnext26_2x64d():
    return ResNeXt(num_blocks=[2, 2, 2, 2], cardinality=2, bottleneck_width=64)

def resnext26_4x32d():
    return ResNeXt(num_blocks=[2, 2, 2, 2], cardinality=4, bottleneck_width=32)

def resnext26_64x2d():
    return ResNeXt(num_blocks=[2, 2, 2, 2], cardinality=32, bottleneck_width=4)

def resnext50_2x64d():
    return ResNeXt(num_blocks=[3, 4, 6, 3], cardinality=2, bottleneck_width=64)

def resnext50_32x4d(): # here
    return ResNeXt(num_blocks=[3, 4, 6, 3], cardinality=32, bottleneck_width=4)

"""
Above is Resx net
Below is FPN
"""

resxnet14_sizes = [64, 128, 256, 512]
resxnet50_sizes = [256, 512, 1024, 2048]

from .fpn import BottleFPN
class ResxFpn_standard(BottleFPN):
    """
    ResxFpn-CNN
    """
    def __init__(self, input_channels=1, output_channels=4, structure_type='resxnet14', pre_trained=False):
        self.input_channels = input_channels
        self.pre_trained = pre_trained
        self.structure_type = structure_type
        super().__init__(output_channels)

    def make_bottleneck(self):
        if self.structure_type == 'resxnet50':
            resxnet = resnext50_32x4d()
            resxnet_sizes = resxnet50_sizes
        elif self.structure_type == 'resxnet26':
            resxnet = resnext26_4x32d()
            resxnet_sizes = resxnet50_sizes
        elif self.structure_type == 'resxnet14':
            resxnet = resnext14_4x32d()
            resxnet_sizes = resxnet14_sizes
        else: KeyError("Wrong resx_type type : {}".format(self.structure_type))

        # Modify reset net here
        resxnet.conv1 = nn.Conv2d(self.input_channels, 64, kernel_size=7, stride=2, padding=3,bias=False)
        layer0 = nn.Sequential(resxnet.conv1, resxnet.bn1, resxnet.relu, resxnet.maxpool)
        layer1 = nn.Sequential(resxnet.layer1)
        layer2 = nn.Sequential(resxnet.layer2)
        layer3 = nn.Sequential(resxnet.layer3)
        layer4 = nn.Sequential(resxnet.layer4)
        
        # Freeze those weights
        if self.pre_trained:
            for p in resxnet.parameters():
                p.requires_grad = False
        resxnet_layers = layer0, layer1, layer2, layer3, layer4


        return resxnet_layers, resxnet_sizes