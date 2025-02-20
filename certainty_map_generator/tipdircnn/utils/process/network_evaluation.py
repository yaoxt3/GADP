from .grasp_definition import GraspRectangles
from skimage.feature import peak_local_max

import numpy as np
from skimage.filters import gaussian


def detect_grasps(qua_vote, ang_vote, wid_vote, Grasp_Type, no_grasps=1):
    """
    Detect grasps in a GG-CNN output.
    :param q_img: Q image network output
    :param ang_img: Angle image network output
    :param width_img: (optional) Width image network output
    :param no_grasps: Max number of grasps to return
    :return: list of Grasps
    """
    local_max = peak_local_max(qua_vote, min_distance=20, threshold_abs=0.2, num_peaks=no_grasps)
    grasps = []
    for grasp_point_array in local_max:
        grasp_point = tuple(grasp_point_array)
        g = Grasp_Type(grasp_point, ang_vote[grasp_point], wid_vote[grasp_point])
        grasps.append(g)
    return grasps


def calculate_iou(detect_bbs, ground_truth_bbs):
    """
    Calculate grasp success using the IoU (Jacquard) metric 
    (e.g. in https://arxiv.org/abs/1301.3592)
    A success is counted if grasp rectangle has a 25% IoU with a ground truth, 
    and is withing 30 degrees.
    :param ground_truth_bbs: Corresponding ground-truth BoundingBoxes
    :param no_grasps: Maximum number of grasps to consider per image.
    :return: success
    """
    if not isinstance(ground_truth_bbs, GraspRectangles):
        gt_bbs = GraspRectangles.load_from_array(ground_truth_bbs)
    else:
        gt_bbs = ground_truth_bbs
    for g in detect_bbs: # g is Grasp() or Tipdir()
        if g.max_iou(gt_bbs) > 0.25:
            return True
    else:
        return False


def post_process_output(N_preds, label_map, width_scale = 150.0):
    """
    Post-process the raw output, convert to numpy arrays, apply filtering.
    :return: Filtered Q output, Filtered Angle output, Filtered Width output
    """
    # INPUT: preds:[c, N, 300, 300]
    arr_qua_img, arr_cos_img, arr_sin_img, arr_wid_img, arr_ang_img = \
        None, None, None, None, None # img [N, 300, 300]
    # print(len(N_preds), N_preds[0].shape) # 4 [8, 1, 300, 300]
    
    preds_post = []
    for preds in N_preds: # c class
        _pred = preds.detach().cpu().numpy().squeeze(axis=1) # _pred: [8, 300, 300]
        _pred  = gaussian(_pred, 1.0, preserve_range=True, channel_axis=0)
        preds_post.append(np.copy(_pred))

    fetch_ind = 0
    if label_map['Qua'] in ['Pos', 'Tip']:
        arr_qua_img = preds_post[fetch_ind] # [8, 300, 300]
        fetch_ind += 1
    if label_map['Cos'] in ['Pos', 'Tip']:
        arr_cos_img = preds_post[fetch_ind]
        fetch_ind += 1
    if label_map['Sin'] in ['Pos', 'Tip']:
        arr_sin_img = preds_post[fetch_ind]
        fetch_ind += 1
    if label_map['Wid'] in ['Pos', 'Tip']:
        arr_wid_img = preds_post[fetch_ind]

    arr_ang_img = (np.arctan2(arr_sin_img, arr_cos_img) \
        / (2.0 if (label_map['Cos'] == 'Pos') else 1.0))
    if arr_wid_img is not None: arr_wid_img = arr_wid_img * width_scale

    return (arr_qua_img, arr_cos_img, arr_sin_img, arr_wid_img, arr_ang_img)

