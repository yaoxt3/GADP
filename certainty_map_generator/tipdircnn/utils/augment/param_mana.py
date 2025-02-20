import argparse

### TRAIN ###
def parse_args_train():
    parser = argparse.ArgumentParser(description='Train tipdir cnn')
    # Network
    parser.add_argument('--network', type=str, default='ggcnn', help='Network name: ggcnn_std / resfpn_std / resxfpn_std / dlafpn_std / efffpn_std / segnet') 
    parser.add_argument('--label-type', type=str, default='dircen', help='label type: rect/ dirwip / dirtip / dircen')
    parser.add_argument('--dataset', type=str, help='Dataset name ("cornell")')
    parser.add_argument('--dataset-path', type=str, help='Path to dataset')
    parser.add_argument('--num-workers', type=int, default=4, help='Dataset workers')
    parser.add_argument('--use-depth', type=int, default=1, help='Use Depth image for training (1/0)')
    parser.add_argument('--use-rgb', type=int, default=0, help='Use RGB image for training (0/1)')
    parser.add_argument('--img-size', type=int, default=300, help='width and height of image of dataset')
    parser.add_argument('--split', type=float, default=0.9, help='Fraction of data for training (remains is validation)')
    parser.add_argument('--ds-rotate', type=float, default=0.0, help='Shift begin of test/train split for cross validation')

    parser.add_argument('--from-trained', type=str, default=None, help='Path of trained network params')
    parser.add_argument('--batch-size', type=int, default=8, help='Batch size')
    parser.add_argument('--epoches', type=int, default=50, help='Training epoches')
    parser.add_argument('--tra-batches', type=int, default=1000, help='Batches per Epoch')
    parser.add_argument('--val-batches', type=int, default=0, help='Validation Batches, set to valid')
    parser.add_argument('--val-augment', action='store_true', help='Whether data augmentation in val')
    parser.add_argument('--tta-size', type=int, default=4, help='Num of average image size')

    parser.add_argument('--description', type=str, default='train', help='Training description')
    parser.add_argument('--outdir', type=str, default='output/', help='Training Output Directory')
    parser.add_argument('--vis', action='store_true', help='Visualise the training process')

    args = parser.parse_args()
    return args

### TEST ###
def parse_args_eval():
    parser = argparse.ArgumentParser(description='Evaluate GGCNN or others')
    # Network
    parser.add_argument('--network', type=str, help='Path to saved network to evaluate')
    parser.add_argument('--dataset', type=str, help='Dataset Name ("cornell")')
    parser.add_argument('--dataset-path', type=str, help='Path to dataset')
    parser.add_argument('--use-depth', type=int, default=1, help='Use depth image (1/0)')
    parser.add_argument('--use-rgb', type=int, default=0, help='Use RGB image (0/1)')
    parser.add_argument('--label-type', type=str, default='dircen', help='label type: rect/ dirwip / dirtip / dircen')
    parser.add_argument('--augment', action='store_true', help='Whether data augmentation') # flag
    parser.add_argument('--shuffle', action='store_true', help='Whether data shuffle') # flag
    parser.add_argument('--split', type=float, default=0.9, help='Fraction of data for training')
    parser.add_argument('--ds-rotate', type=float, default=0, 
                        help='Shift the start point of dataset for split train/evaluate set')
    parser.add_argument('--num-workers', type=int, default=4, help='Dataset workers')
    parser.add_argument('--img-size', type=int, default=300, help='width and height of image of dataset')
    
    parser.add_argument('--n-grasps', type=int, default=1, help='Num of grasp to detect in image')
    parser.add_argument('--tta-size', type=int, default=4, help='Num of average image size')
    parser.add_argument('--iou-eval', action='store_true', help='Compute success on IoU metric')
    parser.add_argument('--vis', action='store_true', help='Visualise network output')
    parser.add_argument('--vis-timeout', action='store_true', help='Auto close window after display')
    parser.add_argument('--val-size', type=int, default=0, help='Num of validation trial, default test set, set to active')
    

    args = parser.parse_args()
    return args


### Grasp Type ###
def analysis_labels(label_type):
    assert label_type in ['rect', 'dirwip', 'dirtip', 'dircen', 'rectrv']

    # label_map = ['QP', 'CP', 'SP', 'WP'] # ['QuaPos', 'CosPos', 'SinPos', 'WidPos']
    # label_map = ['QT', 'CT', 'ST', 'WT'] # ['QuaTip', 'CosTip', 'SinTip', 'WidTip']
    # label_map['detect'] = {'qua':'q_mean/a_var', 'ang':'a_mean', 'wid':'w_mean/w_fixed/30', 'typ': 'Tipdir'}

    ## Cos and Sin must have Pos or Tip.
    if label_type == 'rect':
        label_map = {'Qua':'Pos', 'Cos':'Pos', 'Sin':'Pos', 'Wid':'Pos', 'size': 4}
        label_map['detect'] = {'qua':'q_mean', 'ang':'a_mean', 'wid':'w_mean', 'typ': 'Grasp'}
    elif label_type == 'dirwip':
        label_map = {'Qua':'Tip', 'Cos':'Tip', 'Sin':'Tip', 'Wid':'Tip', 'size': 4}
        label_map['detect'] = {'qua':'q_mean', 'ang':'a_mean', 'wid':'w_amax', 'typ': 'Tipdir'}
    elif label_type == 'dirtip':
        label_map = {'Qua':'Tip', 'Cos':'Tip', 'Sin':'Tip', 'Wid':'Non', 'size': 3}
        label_map['detect'] = {'qua':'q_mean', 'ang':'a_mean', 'wid':10, 'typ': 'Tipdir'}
    elif label_type == 'dircen':
        label_map = {'Qua':'Non', 'Cos':'Tip', 'Sin':'Tip', 'Wid':'Non', 'size': 2}
        label_map['detect'] = {'qua':'a_vari', 'ang':'a_mean', 'wid':10, 'typ': 'Tipdir'}
    elif label_type == 'rectrv': # for exp
        label_map = {'Qua':'Pos', 'Cos':'Pos', 'Sin':'Pos', 'Wid':'Pos', 'size': 4}
        label_map['detect'] = {'qua':'a_vari', 'ang':'a_mean', 'wid':'w_mean', 'typ': 'Grasp'}
    else: 
        KeyError("checking wrong label_type.")
    label_map['type'] = label_type
    label_id_str = label_map['Qua'][0] + label_map['Cos'][0] + label_map['Sin'][0] + label_map['Wid'][0]
    return label_id_str, label_map


def decode_grasp_detection(tta_results, label_map, grasp_type_array):
    (Grasp, Tipdir) = grasp_type_array
    ## decompose grasp detection
    if label_map['detect']['qua']  == 'q_mean':
        qua_vote = tta_results['qua_mean']
    elif label_map['detect']['qua']  == 'q_amax':
        qua_vote = tta_results['qua_amax']
    elif label_map['detect']['qua']  == 'a_vari':
        qua_vote = tta_results['csT_vari_cs_mean']
    else: KeyError('No such detect wid key: {}'.format(label_map['detect']['qua']))
    if label_map['detect']['ang']  == 'a_mean':
        ang_vote = tta_results['csT_ang_mean']
    else: KeyError('No such detect wid key: {}'.format(label_map['detect']['ang']))
    if label_map['detect']['wid']  == 'w_mean':
        wid_vote = tta_results['wid_mean']
    elif label_map['detect']['wid']  == 'w_amax':
        wid_vote = tta_results['wid_amax']
    elif not isinstance(label_map['detect']['wid'], str):
        wid_vote = tta_results['wid_mean'] # TTA model std implement unit wid map
    else: KeyError('No such detect wid key: {}'.format(label_map['detect']['wid']))

    if label_map['detect']['typ'] == "Grasp":
        grasp_type = Grasp
    elif label_map['detect']['typ'] == "Tipdir":
        grasp_type = Tipdir
    else: KeyError('Wrong grasp_type: {}'.format(label_map['detect']['typ']))

    return (qua_vote, ang_vote, wid_vote), grasp_type
