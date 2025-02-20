from .dla import *
from .fpn import BottleFPN

dla34_sizes = [16, 32, 64, 128, 256, 512]
dla46_c_sizes = [16, 32, 64, 64, 128, 256]
dla46x_c_sizes = [16, 32, 64, 64, 128, 256]

class DlaFpn_standard(BottleFPN):
    """
    DlaFpn-CNN
    """
    def __init__(self, input_channels=1, output_channels=4, structure_type='dla46_c', pre_trained=False):
        self.input_channels = input_channels
        self.pre_trained = pre_trained
        self.structure_type = structure_type
        super().__init__(output_channels)

    def make_bottleneck(self):
        if self.pre_trained == False: pre_trained_dir = None
        if self.structure_type == 'dla34':
            dlanet = dla34(pre_trained_dir)
            dlanet_sizes = dla34_sizes
        elif self.structure_type == 'dla46_c':
            dlanet = dla46_c(pre_trained_dir)
            dlanet_sizes = dla46_c_sizes
        elif self.structure_type == 'dla46x_c':
            dlanet = dla46x_c(pre_trained_dir)
            dlanet_sizes = dla46x_c_sizes
        else: KeyError("Wrong dla_type type : {}".format(self.structure_type))

        # Modify reset net here
        dlanet.base_layer = nn.Sequential(
            nn.Conv2d(self.input_channels, dlanet_sizes[0], kernel_size=7, stride=2,
                      padding=3, bias=False),
            BatchNorm(dlanet_sizes[0]),
            nn.ReLU(inplace=True))

        layer0 = nn.Sequential(dlanet.base_layer, dlanet.level0)
        layer1 = nn.Sequential(dlanet.level1) # here chan[0]
        layer2 = nn.Sequential(dlanet.level2)
        layer3 = nn.Sequential(dlanet.level3)
        layer4 = nn.Sequential(dlanet.level4)
        
        # Freeze those weights
        if self.pre_trained:
            for p in dlanet.parameters():
                p.requires_grad = False
        dlanet_layers = layer0, layer1, layer2, layer3, layer4

        dlanet_sizes = dlanet_sizes[1:]
        return dlanet_layers, dlanet_sizes