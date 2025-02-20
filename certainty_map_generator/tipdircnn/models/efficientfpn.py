"""
from official repository:
https://github.com/lukemelas/EfficientNet-PyTorch
1. 
install through pip in conda:
pip install efficientnet_pytorch

EfficientNet.from_name('efficientnet-b0', input_channels)
[from_name(cls, model_name, in_channels=3, **override_params)]

or use [include_top] argument:
model = EfficientNet.from_name("efficientnet-b0", num_classes=2, include_top=False, in_channels=1)

or only pass x to model works: 
endpoints = model.extract_endpoints(inputs)

>>> print(endpoints['reduction_1'].shape)  # torch.Size([1, 16, 112, 112])
>>> print(endpoints['reduction_2'].shape)  # torch.Size([1, 24, 56, 56])
>>> print(endpoints['reduction_3'].shape)  # torch.Size([1, 40, 28, 28])
>>> print(endpoints['reduction_4'].shape)  # torch.Size([1, 112, 14, 14])
>>> print(endpoints['reduction_5'].shape)  # torch.Size([1, 320, 7, 7])
>>> print(endpoints['reduction_6'].shape)  # torch.Size([1, 1280, 7, 7])

2. 
or use:
    model = torch.hub.load(
        'lukemelas/EfficientNet-PyTorch',
        name, # 'efficientnet_b0'
        num_classes=num_classes,
        pretrained=pretrained,
        in_channels=in_channels)
"""

import torch
from efficientnet_pytorch import EfficientNet
from .fpn import BottleFPN

effb0_sizes = [16, 24, 40, 112, 320, 1280]
effb1_sizes = [16, 24, 40, 112, 320, 1280]


class EfficientFpn_standard(BottleFPN):
    """
    ResFpn-CNN test
    """
    def __init__(self, input_channels=1, output_channels=4, structure_type='efficientnet-b0', pre_trained=False):
        self.input_channels = input_channels
        self.structure_type = structure_type
        self.pre_trained = pre_trained
        super().__init__(output_channels)
        
    def make_bottleneck(self):
        if self.structure_type == 'efficientnet-b0':
            effnet = EfficientNet.from_name("efficientnet-b0", num_classes=2, include_top=False, in_channels=self.input_channels)
            effnet_sizes = effb0_sizes
        elif self.structure_type == 'efficientnet-b1':
            effnet = EfficientNet.from_name("efficientnet-b1", num_classes=2, include_top=False, in_channels=self.input_channels)
            effnet_sizes = effb1_sizes
        else: KeyError("Wrong effnet type : {}".format(self.structure_type))

        # Modify reset net here
        layer0, layer1, layer2, layer3, layer4 = None, None, None, None, None

        # Freeze those weights
        if self.pre_trained:
            for p in effnet.parameters():
                p.requires_grad = False

        effnet_layers = layer0, layer1, layer2, layer3, layer4
        self.efficientnet = effnet
        effnet_sizes = effnet_sizes[1:]
        return effnet_layers, effnet_sizes

    def forward(self, x):
        # Bottom-up
        layers = self.efficientnet.extract_endpoints(x)
        # print(layers)
        # c1 = layers['reduction_1']
        c2 = layers['reduction_2']
        c3 = layers['reduction_3']
        c4 = layers['reduction_4']
        c5 = layers['reduction_5']
        # c6 = layers['reduction_6']

        # Top-down
        p5 = self.toplayer(c5)
        p4 = self._upsample_add(p5, self.latlayer1(c4))
        p4 = self.smooth1(p4)
        p3 = self._upsample_add(p4, self.latlayer2(c3))
        p3 = self.smooth2(p3)
        p2 = self._upsample_add(p3, self.latlayer3(c2))
        p2 = self.smooth3(p2)

        # Top-down predict and refine
        d5, d4, d3, d2 = self.up1(self.agg1(p5)), self.up2(self.agg2(p4)), \
            self.up3(self.agg3(p3)), self.agg4(p2)
        _,_,H,W = d2.size()
        # bottle_size/2 *4 = bottle_size*2, size=d2.size()
        upsp_cated = torch.cat([ torch.nn.functional.upsample(
            d, size=(H,W), mode='bilinear') for d in [d5,d4,d3,d2] ], dim=1)
        final_preconv = self.up_final(self.smooth_final(upsp_cated)) # img_HW = 4 * upsp_cated.size()

        head_ouputs = []
        for ind in range(self.output_channels):
            head_ouputs.append(self.conv_heads[ind](final_preconv))

        return head_ouputs