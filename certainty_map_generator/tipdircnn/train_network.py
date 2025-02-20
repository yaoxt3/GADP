import datetime
import os
import sys
import logging

import cv2
import numpy as np

import torch
import torch.utils.data
import torch.optim as optim
import tensorboardX
from torchsummary import summary

from models import get_network
from utils.dataset import get_dataset
from utils.visual.output_display import gridshow
from utils.augment import param_mana
from utils.process.grasp_definition import Grasp, Tipdir
from utils.augment import tta_process
from utils.process import network_evaluation


def dataset(args, label_map):
    Dataset = get_dataset(args.dataset)

    train_dataset = Dataset(
        args.dataset_path, start=0.0, end=args.split, ds_rotate=args.ds_rotate,
        label_map = label_map, output_size=args.img_size,
        random_rotate=True, random_zoom=True, # rotation[n * pi/2] zoom[uniform(0.5, 1)]
        include_depth=args.use_depth, include_rgb=args.use_rgb)
    train_data = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers
    )
    val_dataset = Dataset(
        args.dataset_path, start=args.split, end=1.0, ds_rotate=args.ds_rotate, 
        label_map = label_map, output_size=args.img_size,
        random_rotate=args.val_augment, random_zoom=args.val_augment,
        include_depth=args.use_depth, include_rgb=args.use_rgb)
    val_data = torch.utils.data.DataLoader(
        val_dataset,
        batch_size=1,
        shuffle=True, # False
        num_workers=args.num_workers
    )
    return train_data, val_data


def validate(net, device, val_data, loss_func, val_batches, label_map, tta_size):
    """
    Run validation.
    :param net: Network
    :param device: Torch device
    :param val_data: Validation Dataset
    :param val_batches: Number of batches to run; val_dataset(89) but with augmentation
    :param label_map: 
    :return: Successes, Failures and Losses
    """
    net.eval()
    results = {'correct': 0, 'failed': 0,'loss': 0, 'losses':{}, 'val_vis':()}

    ld = len(val_data)
    if val_batches == 0: val_batches = ld
    with torch.no_grad():
        batch_idx = 0
        while batch_idx <= val_batches:
            for x, y, didx, rot, zoom in val_data:
                batch_idx += 1
                if val_batches is not None and batch_idx > val_batches:
                    break
                xc = x.to(device)
                yc = [yy.to(device) for yy in y]
                yp = net(xc)

                fetch_ind = 0
                loss = None
                for key, val in label_map.items():
                    if val in ['Pos', 'Tip']:
                        _loss = loss_func(yp[fetch_ind], yc[fetch_ind])
                        if loss is not None: loss += _loss
                        else: loss = _loss
                        
                        key_loss = key.lower() + '_loss'
                        if key_loss not in results['losses']:
                            results['losses'][key.lower() + '_loss'] = _loss.item()/ld
                        else:
                            results['losses'][key.lower() + '_loss'] += _loss.item()/ld
                        fetch_ind += 1
                results['loss'] += loss.item()/ld

                x_arr, tta_belta = tta_process.TTA_input_generation(x.cpu().numpy().squeeze(axis=0), tta_size)
                x_arr_tensor = torch.from_numpy(x_arr).to(device)
                post_preds = network_evaluation.post_process_output(net(x_arr_tensor), label_map)
                tta_samples = tta_process.TTA_organize_data(tta_belta, x_arr, post_preds)
                tta_estimation, regulated_sample_data = tta_process.TTA_modeling_std(tta_samples, label_map)
                (qua_vote, ang_vote, wid_vote), grasp_type = \
                    param_mana.decode_grasp_detection(tta_estimation, label_map, (Grasp, Tipdir))

                dete_bbs = network_evaluation.detect_grasps(qua_vote, ang_vote, wid_vote, grasp_type)
                gt_bbs = val_data.dataset.get_bbs(didx, rot, zoom)
                s = network_evaluation.calculate_iou(dete_bbs, gt_bbs)
                results['correct' if s else 'failed'] += 1
    
    if not isinstance(label_map['detect']['wid'], str): 
        wid_vote = tta_estimation['csT_vari_cs_mean'] * 100 # For fancy visualization
    img_ori = tta_estimation['img_mean'] # here is depthimage (300,300)
    results['val_vis'] = (qua_vote, ang_vote, wid_vote, img_ori, dete_bbs)
    return results 


def validate_log(test_results, tb, epoch, label_map, vis, img_val_name):
    logging.info('%d/%d = %f' % (test_results['correct'], (test_results['correct'] + test_results['failed']),
                                    test_results['correct']/ (test_results['correct']+test_results['failed'])))
    # Log validate result to tensorboard
    tb.add_scalar('loss/IOU', test_results['correct'] / 
                    (test_results['failed'] + test_results['correct']), epoch)
    tb.add_scalar('loss/val_loss', test_results['loss'], epoch)
    for n, l in test_results['losses'].items():
        tb.add_scalar('val_loss/' + n, l, epoch)
    
    # Visual validation result
    (qua_vote, ang_vote, wid_vote, img_ori, gs) = test_results['val_vis']
    from utils.process.grasp_definition import Tipdir
    for g in gs:
        if isinstance(g, Tipdir): g = g.as_rect
        elif isinstance(g, Grasp): g = g.as_grarect
        points = np.vstack((g.points, g.points[0])).astype(np.int32)
        for i in range(len(points) - 1):
            img_ori = cv2.line(img_ori, (points[i][1], points[i][0]), (points[i+1][1], points[i+1][0]),\
                color=[img_ori.max() * 1.0], thickness=2)
    imgs, rangs, cmaps = [], [], []
    angle_range = np.pi/2 if(label_map['Cos'] == 'Pos') else np.pi 
    imgs.extend([qua_vote, ang_vote, wid_vote, img_ori])
    rangs.extend([(0.0, 1.0), (-angle_range, angle_range), (0.0, 30.0), (img_ori.min(), img_ori.max())])
    cmaps.extend([cv2.COLORMAP_JET, cv2.COLORMAP_JET, cv2.COLORMAP_JET, cv2.COLORMAP_BONE])
    img_out_val = gridshow(imgs, rangs, cmaps, width=2, border=10)
    tb.add_image('image_valdate', torch.flip(torch.permute(torch.from_numpy(img_out_val), (2, 0, 1)), dims=(0,)), epoch)
    if vis:
        cv2.imshow(img_val_name, img_out_val)
        cv2.waitKey(1)


def train(epoch, net, device, train_data, optimizer, loss_func, label_map, tra_batches):
    """
    Run one training epoch
    :param epoch: Current epoch
    :param net: Network
    :param device: Torch device
    :param train_data: Training Dataset
    :param optimizer: Optimizer
    :param tra_batches:  Data batches to train on
    :param vis:  Visualise training progress
    :return:  Average Losses for Epoch
    """
    batch_results = {'loss': 0, 'losses':{ } }
    train_snap = None

    net.train()

    batch_idx = 0
    while batch_idx < tra_batches:
        for x, y, _, _, _ in train_data:
            batch_idx += 1
            if batch_idx > tra_batches: break
            xc = x.to(device)
            yc = [yy.to(device) for yy in y]
            yp = net(xc)

            fetch_ind = 0
            loss = None
            for key, val in label_map.items():
                if val in ['Pos', 'Tip']:
                    _loss = loss_func(yp[fetch_ind], yc[fetch_ind])
                    if loss is not None: loss += _loss
                    else: loss = _loss
                    
                    key_loss = key.lower() + '_loss'
                    if key_loss not in batch_results['losses']:
                        batch_results['losses'][key.lower() + '_loss'] = _loss.item()
                    else:
                        batch_results['losses'][key.lower() + '_loss'] += _loss.item()
                    fetch_ind += 1

            batch_results['loss'] += loss.item()
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            if (batch_idx + 1) % 50 == 0:
                logging.info('Epoch: {}, Batch: {:0>3d}, Loss: {:0.4f}'\
                    .format(epoch, batch_idx + 1, loss.item()))

            # take a snap on the label and pred when training
            if (batch_idx == (tra_batches - 1)): # the latest batch
                train_snap = [x, y, yp]

    batch_results['loss'] /= batch_idx
    for l in batch_results['losses']:
        batch_results['losses'][l] /= batch_idx
    return batch_results, train_snap


def train_log(epoch, train_results, train_snap, tb, label_map, vis, img_tra_name):
        # Train label and pred: visual_window/save_local/tensorboard
        imgs = []
        [x, y, yp] = train_snap
        n_img = min(4, x.shape[0]) # equal to batch_size
        for idx in range(n_img):
            imgs.extend(
            [x[idx, ].numpy().squeeze()] + # xc.min/max
            [yi[idx, ].numpy().squeeze() for yi in y] + # [qua, cos, sin, wid] someof
            [x[idx, ].numpy().squeeze()] + # [qua, cos, sin, wid] someof
            [pc[idx, ].detach().cpu().numpy().squeeze() for pc in yp])
        
        img_num = (len(y) + 1) * 2
        range_array =[]
        range_array.append((x.min().item(), x.max().item()))
        if(label_map['Qua'] != 'Non'): range_array.append((0.0, 1.0))
        range_array.append((-1.0, 1.0))
        range_array.append((-1.0, 1.0))
        if(label_map['Wid'] != 'Non'): range_array.append((0.0, .50))
        range_array = range_array * 2 * n_img
        image_label_output = gridshow(imgs, range_array, [cv2.COLORMAP_BONE] * img_num *n_img, width=img_num, border=10)
        # cv2.imwrite(os.path.join(image_path, 'vis_epoch_%02d'%epoch + '.jpg'), image_label_output) # train label/pred
        if vis: # visual it in window
            cv2.imshow(img_tra_name, image_label_output)
            cv2.waitKey(1)
        tb.add_image('image_train', torch.flip(torch.permute(torch.from_numpy(image_label_output), (2, 0, 1)), dims=(0,)), epoch)
        tb.add_scalar('loss/train_loss', train_results['loss'], epoch)
        for n, l in train_results['losses'].items():
            tb.add_scalar('train_loss/' + n, l, epoch)

    
def run():
    args = param_mana.parse_args_train()
    label_id_str, label_map = param_mana.analysis_labels(args.label_type)
    # Torch device
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print('Torch Using device:', device)

    # dir management of Tensorboard/model/image 
    data_type = ('rgb' if args.use_rgb == 1 else 'mm') + ('depth' if args.use_depth else 'mm')
    log_head = '{}-{}_{}-{}-bs{}'.format(args.network, args.label_type, label_id_str,
        data_type, args.batch_size)
    logging.basicConfig(format='[%(asctime)s ' + log_head + ']:  %(message)s', level=logging.INFO)

    dt = datetime.datetime.now().strftime('%y%m%d_%H%M')
    net_desc = '{}-{}-{}-{}_{}-{}-bs{}'.format(dt, args.description, 
        args.network, args.label_type, label_id_str,
        data_type, args.batch_size)
    img_tra_name = 'Train_label_predition_' + net_desc
    img_val_name = 'Validate_Qua_Ang_Wid_REct_' + net_desc
    output_path = os.path.join(args.outdir, net_desc)
    if not os.path.exists(output_path): os.makedirs(output_path)
    # visual window setsize
    if args.vis : 
        cv2.namedWindow(img_tra_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(img_tra_name, 1200, 540)
        cv2.namedWindow(img_val_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(img_val_name, 600, 600)

    # tensonboard dir set 
    tb = tensorboardX.SummaryWriter(output_path, flush_secs=30)

    # Load Dataset
    logging.info('Loading {} Dataset...'.format(args.dataset.title()))
    train_data, val_data = dataset(args, label_map)
    logging.info('Finished data loading.')

    # Load network and set optimizer
    logging.info('Create Network...')
    input_channels = 1 * args.use_depth + 3 * args.use_rgb
    if args.from_trained is not None:
        logging.info('Loading network from {}'.format(args.from_trained))
        net = torch.load(args.from_trained)
    else:
        cnn_model = get_network(args.network)
        net = cnn_model(input_channels=input_channels, output_channels=label_map['size'])
    net = net.to(device=device)
    optimizer = optim.Adam(net.parameters(), lr=0.002)
    # scheduler = optim.lr_scheduler.MultiStepLR(optimizer, \
    #     milestones=[int(args.epoches/2), int(args.epoches/4*3)], gamma=0.1)
    logging.info('Use optim.lr_scheduler.StepLR as: {}'.format(int(args.epoches/4)))
    scheduler = optim.lr_scheduler.StepLR(optimizer, int(args.epoches/4), gamma=0.2)
    loss_func = torch.nn.MSELoss()
    logging.info('Finished network preparation.')

    # Log network information and feature to local record
    file_dir = os.path.join(output_path, 'model_info.txt')
    f = open(file_dir, 'w')
    sys.stdout = f
    print("======= HEAD INFORMATION ======= ")
    print('Network type: {} with structure: {}'.format(args.network, net.structure_type))
    print('input dims: ({}, {}, {})'.format(input_channels, args.img_size, args.img_size))
    print('Label type: {} specifying: {} with size: {}'\
        .format(args.label_type, label_id_str, label_map['size']))
    print("Detect Grasp information and Label map: ", label_map)
    print("======= HEAD INFORMATION ======= \n\n")
    summary(net, (input_channels, args.img_size, args.img_size))
    print(net)
    sys.stdout = sys.__stdout__
    f.close()
    with open(file_dir) as f:
        for line in f: print(line.strip())

    best_iou = 0.0
    for epoch in range(args.epoches):
        # Train
        logging.info('Beginning Epoch {:02d}'.format(epoch))
        train_results, train_snap = train(epoch, net, device, train_data, optimizer, loss_func, label_map, args.tra_batches)
        scheduler.step()
        # Train label and pred: visual_window/save_local/tensorboard
        train_log(epoch, train_results, train_snap, tb, label_map, args.vis, img_tra_name)
        
        # Validate
        logging.info('Validating ...')
        test_results = validate(net, device, val_data, loss_func, args.val_batches, label_map, args.tta_size)
        validate_log(test_results, tb, epoch, label_map, args.vis, img_val_name)

        # Save best performing network
        iou = test_results['correct'] / (test_results['correct'] + test_results['failed'])
        if iou > best_iou or epoch == 0 or ((epoch + 1) % 20) == 0:
            torch.save(net, os.path.join(output_path, 'epoch_%02d_iou_%0.2f.model' % (epoch, iou)))
            torch.save(net.state_dict(), os.path.join(output_path, 'epoch_%02d_iou_%0.2f_statedict.pt' % (epoch, iou)))
            best_iou = iou


if __name__ == '__main__':
    run()