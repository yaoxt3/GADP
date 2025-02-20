import numpy as np
from ..process.image_definition import Image


def TTA_input_generation(x_np, tta_size):
    depth_ori = None
    rgb_ori = None
    # x_np ([1/3/4], 300, 300)
    if x_np.shape[0] == 1:
        depth_ori = x_np[0]
    elif x_np.shape[0] == 3:
        rgb_ori = x_np
    elif x_np.shape[0] == 4:
        depth_ori = x_np[0]
        rgb_ori = x_np[1:]
    
    x_arr = []
    tta_belta = np.linspace(0, 2*np.pi, num=tta_size, endpoint=False)
    for ang in tta_belta:
        if depth_ori is not None:
            depth_rot_c = Image(depth_ori).rotated(ang)
            depth_rot = np.expand_dims(depth_rot_c.img, 0)
        if rgb_ori is not None:
            # Back to (300, 300, 3) to make a proper rotation in skimage
            rgb_rot_c = Image(np.transpose(np.copy(rgb_ori), (1, 2, 0))).rotated(ang)
            rgb_rot = rgb_rot_c.img.transpose((2, 0, 1)) # To (3, 300, 300) as net input dims

        if x_np.shape[0] == 1:
            i_rot = depth_rot
        elif x_np.shape[0] == 3:
            i_rot = rgb_rot
        elif x_np.shape[0] == 4:
            # (1, 300, 300) + (3, 300, 300) = (4, 300, 300)
            i_rot = np.concatenate((depth_rot, rgb_rot), 0)
        x_arr.append(i_rot)
    x_arr = np.array(x_arr)
    return x_arr, tta_belta


def TTA_organize_data(tta_belta, x_arr, post_preds):
    (arr_qua_img, arr_cos_img, arr_sin_img, arr_wid_img, arr_ang_img) = post_preds # img:[N, 300, 300]
    tta_samples = []
    gs = 0
    for g_belta in tta_belta: # N 
        tta_sample = {
            'tta_belta': g_belta,
            'img': np.mean(x_arr[gs], 0).squeeze(),
            'pred': {
                'qua' : arr_qua_img[gs] if arr_qua_img is not None else None,
                'cos' : arr_cos_img[gs],
                'sin' : arr_sin_img[gs],
                'wid' : arr_wid_img[gs] if arr_wid_img is not None else None,
                'ang' : arr_ang_img[gs],
            }
        }
        tta_samples.append(tta_sample)
        gs += 1
    return tta_samples


def TTA_modeling_std(tta_samples, label_map):
    regulated_sample_data = {kn : [] for kn in ['img', 'qua', 'wid', 'aP', 'cP', 'sP', 'cT', 'sT']}

    # For each TTA rotated samples
    for row_i in range(len(tta_samples)):
        g_belta = 2 * np.pi - tta_samples[row_i]['tta_belta'] # reverse operation
        img = Image(tta_samples[row_i]['img'])

        # Visually rotation
        cos_img_np = np.array(Image(tta_samples[row_i]['pred']['cos']).rotated(g_belta))
        sin_img_np = np.array(Image(tta_samples[row_i]['pred']['sin']).rotated(g_belta))
        angle_img_np = np.array(Image(tta_samples[row_i]['pred']['ang']).rotated(g_belta))

        # Y = g(Y_n) g(\belta) = R(\belta), R is rotation matrix
        img_shape = cos_img_np.shape
        directional_belta = g_belta * (2.0 if (label_map['Cos'] == 'Pos') else 1.0)
        cos_img_triangle_b = cos_img_np * (np.ones(img_shape) * np.cos(2 * np.pi - directional_belta)) + \
                           sin_img_np * (np.ones(img_shape) * np.sin(2 * np.pi - directional_belta))

        sin_img_triangle_b = sin_img_np * (np.ones(img_shape) * np.cos(2 * np.pi - directional_belta)) - \
                           cos_img_np * (np.ones(img_shape) * np.sin(2 * np.pi - directional_belta))

        # unit vector regularization
        norm_cs = np.sqrt(cos_img_triangle_b**2 + sin_img_triangle_b**2)
        cos_img_triangle = ( 1 / norm_cs) * cos_img_triangle_b
        sin_img_triangle = ( 1 / norm_cs) * sin_img_triangle_b
        # cos_img_triangle = cos_img_triangle_b
        # sin_img_triangle = sin_img_triangle_b

        regulated_sample_data['img'].append(img.copy())
        regulated_sample_data['aP'].append(angle_img_np.copy())
        regulated_sample_data['cP'].append(cos_img_np.copy())
        regulated_sample_data['sP'].append(sin_img_np.copy())
        regulated_sample_data['cT'].append(cos_img_triangle.copy())
        regulated_sample_data['sT'].append(sin_img_triangle.copy())

        if label_map['Qua'] != 'Non':
            quality_img_np = np.array(Image(tta_samples[row_i]['pred']['qua']).rotated(g_belta))
        else:
            temp_img_np = np.sqrt(np.power(cos_img_np, 2) + np.power(sin_img_np, 2))
            quality_img_np = np.array(Image(temp_img_np).rotated(g_belta)) # temp solution just for fill it
        regulated_sample_data['qua'].append(quality_img_np.copy())

        if label_map['Wid'] != 'Non':
            width_img_np = np.array(Image(tta_samples[row_i]['pred']['wid']).rotated(g_belta))
        else: 
            width_img_np = np.ones_like(cos_img_np) * label_map['detect']['wid']
        regulated_sample_data['wid'].append(width_img_np.copy())

    # TTA estimation, directional statistics
    tta_estimation = {}
    for (key, value) in regulated_sample_data.items():
        if len(value) == 0: continue
        if key in ('img'):
            tta_estimation[key + '_mean'] = value[0].img
        elif key in ('qua', 'wid'):
            tta_estimation[key + '_mean'] = np.mean(value, 0)
            tta_estimation[key + '_amax'] = np.amax(value, 0)
            tta_estimation[key + '_vari'] = np.var(value, 0)
        elif key in ('cT', 'sT'):
            tta_estimation[key + '_mean'] = np.mean(value, 0)
            tta_estimation[key + '_vari'] = np.var(value, 0)
        else:
            continue
    tta_estimation['csT_ang' + '_mean'] = np.arctan2(tta_estimation['sT_mean'], tta_estimation['cT_mean'])\
         / (2.0 if (label_map['Cos'] == 'Pos') else 1.0)
    tta_estimation['csT_vari' + '_mean'] = np.sqrt(-np.log(np.power(tta_estimation['sT_mean'], 2)\
         + np.power(tta_estimation['cT_mean'], 2))) # short for below
    tta_estimation['csT_vari_cs' + '_mean'] = np.sqrt(np.power(tta_estimation['sT_mean'], 2)\
         + np.power(tta_estimation['cT_mean'], 2)) # \bar{R} 
    
    if label_map['detect']['qua']  == 'a_vari':
        tta_estimation['qua' + '_mean'] = tta_estimation['csT_vari_cs_mean']
    # tta_estimation['csT_var_cs' + '_mean'] = tta_estimation['sT_var'] + tta_estimation['cT_var'] # sum of angle variance
    # tta_estimation['csT_var'] = np.sqrt(-2 * np.log(np.sqrt(np.power(tta_estimation['sT_mean'], 2) + np.power(tta_estimation['cT_mean'], 2))))

    return tta_estimation, regulated_sample_data


######### REPLACE WITH TTA INPUT DATA MODULE #############

                # ## TTA data generation of tips estimate
                # depth_ori = None
                # rgb_ori = None
                # x_np = x[0].cpu().detach().numpy() # ([1/3/4], 300, 300)
                # if x_np.shape[0] == 1:
                #     depth_ori = x_np[0]
                # elif x_np.shape[0] == 3:
                #     rgb_ori = x_np
                # elif x_np.shape[0] == 4:
                #     depth_ori = x_np[0]
                #     rgb_ori = x_np[1:]
                
                # q_out = []
                # w_out = []
                # c_out = []
                # s_out = []

                # for ang in np.linspace(0, 2*np.pi, num=tta_size, endpoint=False):
                #     if depth_ori is not None:
                #         depth_rot_c = Image(np.copy(depth_ori)).rotated(ang)
                #         depth_rot = np.expand_dims(depth_rot_c.img, 0)
                #     if rgb_ori is not None:
                #         # Back to (300, 300, 3) to make a proper rotation in skimage
                #         rgb_rot_c = Image(np.transpose(np.copy(rgb_ori), (1, 2, 0))).rotated(ang)
                #         rgb_rot = rgb_rot_c.img.transpose((2, 0, 1)) # To (3, 300, 300) as net input dims

                #     if x_np.shape[0] == 1:
                #         i_rot = depth_rot
                #     elif x_np.shape[0] == 3:
                #         i_rot = rgb_rot
                #     elif x_np.shape[0] == 4:
                #         # (1, 300, 300) + (3, 300, 300) = (4, 300, 300)
                #         i_rot = np.concatenate((depth_rot, rgb_rot), 0)
                #     x_rot = torch.from_numpy(np.expand_dims(i_rot, 0))
                    
                #     (qua_img_rot, cos_img_rot, sin_img_rot, wid_img_rot, ang_out_rot) = \
                #         post_process_output(net(x_rot.to(device)), label_map)
                    
                #     ## TTA modeling
                #     re_ang = 2 * np.pi - ang
                #     # ang_out_rot_f = Image(ang_out_rot).rotated(re_ang)
                #     cos_img_rot_f = Image(cos_img_rot).rotated(re_ang)
                #     sin_img_rot_f = Image(sin_img_rot).rotated(re_ang)
                #     cos_img_np = cos_img_rot_f.img
                #     sin_img_np = sin_img_rot_f.img
                #     img_shape = cos_img_np.shape
                #     cos_img_triangle_b = cos_img_np * (np.ones(img_shape) * np.cos(2 * np.pi - re_ang)) + \
                #                     sin_img_np * (np.ones(img_shape) * np.sin(2 * np.pi - re_ang))

                #     sin_img_triangle_b = sin_img_np * (np.ones(img_shape) * np.cos(2 * np.pi - re_ang)) - \
                #                     cos_img_np * (np.ones(img_shape) * np.sin(2 * np.pi - re_ang))
                #     # unit vector regularization
                #     norm_cs = np.sqrt(cos_img_triangle_b**2 + sin_img_triangle_b**2)
                #     cos_img_triangle = ( 1 / norm_cs) * cos_img_triangle_b
                #     sin_img_triangle = ( 1 / norm_cs) * sin_img_triangle_b
                #     c_out.append(cos_img_triangle)
                #     s_out.append(sin_img_triangle)

                #     if qua_img_rot is not None: 
                #         qua_img_rot_f = Image(qua_img_rot).rotated(re_ang)
                #         q_out.append(qua_img_rot_f.img)

                #     if wid_img_rot is not None: 
                #         wid_img_rot_f = Image(wid_img_rot).rotated(re_ang)
                #         w_out.append(wid_img_rot_f.img)

                # sm, cm = np.mean(s_out, 0), np.mean(c_out, 0)
                # ang_out_mean =  np.arctan2(sm, cm) / (2.0 if (label_map['Cos'] == 'Pos') else 1.0)
                # ang_out_vari = np.sqrt(np.power(cm, 2) + np.power(sm, 2))
                
                # if qua_img_rot is not None: 
                #     qua_out_mean = np.mean(q_out, 0)
                #     qua_out_amax = np.amax(q_out, 0)
                # if wid_img_rot is not None: 
                #     wid_out_mean = np.mean(w_out, 0)
                #     wid_out_amax = np.amax(w_out, 0)

                # ## decompose grasp detection
                # if label_map['detect']['qua']  == 'q_mean':
                #     qua_vote = qua_out_mean
                # elif label_map['detect']['qua']  == 'q_amax':
                #     qua_vote = qua_out_amax
                # elif label_map['detect']['qua']  == 'a_vari':
                #     qua_vote = ang_out_vari
                # else: KeyError('No such detect wid key: {}'.format(label_map['detect']['qua']))
                # if label_map['detect']['ang']  == 'a_mean':
                #     ang_vote = ang_out_mean
                # else: KeyError('No such detect wid key: {}'.format(label_map['detect']['ang']))
                # if label_map['detect']['wid']  == 'w_mean':
                #     wid_vote = wid_out_mean
                # elif label_map['detect']['wid']  == 'w_amax':
                #     wid_vote = wid_out_amax
                # elif not isinstance(label_map['detect']['wid'], str):
                #     wid_vote = np.ones_like(ang_out_mean) * label_map['detect']['wid']
                # else: KeyError('No such detect wid key: {}'.format(label_map['detect']['wid']))

                # if label_map['detect']['typ'] == "Grasp":
                #     grasp_type = Grasp
                # elif label_map['detect']['typ'] == "Tipdir":
                #     grasp_type = Tipdir
                # else: KeyError('Wrong grasp_type: {}'.format(label_map['detect']['typ']))
            
######### REPLACE WITH TTA INPUT DATA MODULE #############