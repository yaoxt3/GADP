from .fpn import BottleFPN
import torchvision.models.resnet as resnets
import torch.nn as nn

resnet18_sizes = [64, 128, 256, 512]
resnet34_sizes = [64, 128, 256, 512]
resnet50_sizes = [256, 512, 1024, 2048]
resnet101_sizes = [256, 512, 1024, 2048]

class ResFpn_standard(BottleFPN):
    """
    ResFpn-CNN test
    """
    def __init__(self, input_channels=1, output_channels=4, structure_type='resnet18', pre_trained=False):
        self.input_channels = input_channels
        self.structure_type = structure_type
        self.pre_trained = pre_trained
        super().__init__(output_channels)
        
    def make_bottleneck(self):
        if self.structure_type == 'resnet18':
            resnet = resnets.resnet18(pretrained=self.pre_trained)
            resnet_sizes = resnet18_sizes
        elif self.structure_type == 'resnet34':
            resnet = resnets.resnet34(pretrained=self.pre_trained)
            resnet_sizes = resnet34_sizes
        elif self.structure_type == 'resnet50':
            resnet = resnets.resnet50(pretrained=self.pre_trained)
            resnet_sizes = resnet50_sizes
        elif self.structure_type == 'resnet101':
            resnet = resnets.resnet101(pretrained=self.pre_trained)
            resnet_sizes = resnet101_sizes
        else: KeyError("Wrong resnet type : {}".format(self.structure_type))

        # Modify reset net here
        resnet.conv1 = nn.Conv2d(self.input_channels, 64, kernel_size=7, stride=2, padding=3,bias=False)
        layer0 = nn.Sequential(resnet.conv1, resnet.bn1, resnet.relu, resnet.maxpool)
        layer1 = nn.Sequential(resnet.layer1)
        layer2 = nn.Sequential(resnet.layer2)
        layer3 = nn.Sequential(resnet.layer3)
        layer4 = nn.Sequential(resnet.layer4)
        
        # Freeze those weights
        if self.pre_trained:
            for p in resnet.parameters():
                p.requires_grad = False
        resnet_layers = layer0, layer1, layer2, layer3, layer4

        return resnet_layers, resnet_sizes




