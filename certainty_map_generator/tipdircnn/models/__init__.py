def get_network(network_name):
    network_name = network_name.lower()
    if network_name == 'ggcnn':
        from .ggcnn import GGCNN
        return GGCNN
    elif network_name == 'ggcnn_std':
        from .ggcnn import GGCNN_standard
        return GGCNN_standard
    elif network_name == 'resfpn_std':
        from .resfpn import ResFpn_standard
        return ResFpn_standard
    elif network_name == 'resxfpn_std':
        from .resxfpn import ResxFpn_standard
        return ResxFpn_standard
    elif network_name == 'dlafpn_std':
        from .dlafpn import DlaFpn_standard
        return DlaFpn_standard
    elif network_name == 'efffpn_std':
        from .efficientfpn import EfficientFpn_standard
        return EfficientFpn_standard
    elif network_name == 'segnet':
        # from .segnet import SegNet_Full
        # return SegNet_Full
        from .segnet import SegNet_Basic
        return SegNet_Basic
    else:
        raise NotImplementedError('Network {} is not implemented'.format(network_name))

        