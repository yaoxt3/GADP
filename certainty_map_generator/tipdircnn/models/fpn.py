"""
FPN structure from:
https://github.com/haofengac/MonoDepth-FPN-PyTorch
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

class BottleFPN(nn.Module):
    """
    FPN for various bottleneck nets 
    """
    def __init__(self, output_channels=4):
        super().__init__()
        self.output_channels = output_channels
        net_layers, net_sizes = self.make_bottleneck()

        self.layer0, self.layer1, self.layer2, \
            self.layer3, self.layer4 = net_layers

        # Top layer / Reduce channels
        bottle_size = net_sizes[0]
        half_bottle_size = int(bottle_size/2)
        self.toplayer = nn.Conv2d(net_sizes[3], bottle_size, kernel_size=1, stride=1, padding=0)
        # Lateral layers
        self.latlayer1 = nn.Conv2d(net_sizes[2], bottle_size, kernel_size=1, stride=1, padding=0)
        self.latlayer2 = nn.Conv2d(net_sizes[1], bottle_size, kernel_size=1, stride=1, padding=0)
        self.latlayer3 = nn.Conv2d(net_sizes[0], bottle_size, kernel_size=1, stride=1, padding=0)

        # Smooth layers
        self.smooth1 = nn.Conv2d(bottle_size, bottle_size, kernel_size=3, stride=1, padding=1)
        self.smooth2 = nn.Conv2d(bottle_size, bottle_size, kernel_size=3, stride=1, padding=1)
        self.smooth3 = nn.Conv2d(bottle_size, bottle_size, kernel_size=3, stride=1, padding=1)

        # Aggregate layers
        self.agg1 = self.agg_node(bottle_size, half_bottle_size)
        self.agg2 = self.agg_node(bottle_size, half_bottle_size)
        self.agg3 = self.agg_node(bottle_size, half_bottle_size)
        self.agg4 = self.agg_node(bottle_size, half_bottle_size)
        
        # Upshuffle layers
        self.up1 = self.upshuffle(half_bottle_size,half_bottle_size,8)
        self.up2 = self.upshuffle(half_bottle_size,half_bottle_size,4)
        self.up3 = self.upshuffle(half_bottle_size,half_bottle_size,2)

        self.up_final = self.upshuffle(half_bottle_size,half_bottle_size,4)

        # Prediction
        self.smooth_final = nn.Sequential(
            nn.Conv2d(bottle_size*2, half_bottle_size, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            )

        self.conv_heads = nn.ModuleList()
        for _ in range(self.output_channels):
            self.conv_heads.append(nn.Conv2d(half_bottle_size, out_channels=1, kernel_size=3, stride=1, padding=1))

        if not self.pre_trained:
            for m in self.modules():
                if isinstance(m, (nn.Conv2d, nn.ConvTranspose2d)):
                    nn.init.xavier_uniform_(m.weight, gain=1)

    def make_bottleneck(self):
        NotImplementedError("Please implement bottleneck method in child class first to use fpn.")

    @staticmethod
    def agg_node(in_planes, out_planes):
        return nn.Sequential(
            nn.Conv2d(in_planes, in_planes, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
        )

    @staticmethod
    def upshuffle(in_planes, out_planes, upscale_factor):
        return nn.Sequential(
            nn.Conv2d(in_planes, out_planes*upscale_factor**2, kernel_size=3, stride=1, padding=1),
            nn.PixelShuffle(upscale_factor),
            nn.ReLU()
        )
        
    def _upsample_add(self, x, y):
        '''Upsample and add two feature maps.
        Args:
          x: (Variable) top feature map to be upsampled.
          y: (Variable) lateral feature map.
        Returns:
          (Variable) added feature map.
        Note in PyTorch, when input size is odd, the upsampled feature map
        with `F.upsample(..., scale_factor=2, mode='nearest')`
        maybe not equal to the lateral feature map size.
        e.g.
        original input size: [N,_,15,15] ->
        conv2d feature map size: [N,_,8,8] ->
        upsampled feature map size: [N,_,16,16]
        So we choose bilinear upsample which supports arbitrary output sizes.
        '''
        _,_,H,W = y.size()
        return F.upsample(x, size=(H,W), mode='bilinear') + y

    def forward(self, x):
        # Bottom-up
        c1 = self.layer0(x)
        c2 = self.layer1(c1)
        c3 = self.layer2(c2)
        c4 = self.layer3(c3)
        c5 = self.layer4(c4)

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
        upsp_cated = torch.cat([ F.upsample(d, size=(H,W), mode='bilinear') for d in [d5,d4,d3,d2] ], dim=1)
        final_preconv = self.up_final(self.smooth_final(upsp_cated)) # img_HW = 4 * upsp_cated.size()

        head_ouputs = []
        for ind in range(self.output_channels):
            head_ouputs.append(self.conv_heads[ind](final_preconv))

        return head_ouputs

