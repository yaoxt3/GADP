import logging

import torch
import torch.utils.data

from utils.dataset.misc import load_testset
from utils.visual import output_display
from utils.augment import param_mana
from utils.augment import tta_process
from utils.process import network_evaluation
from utils.process.grasp_definition import Grasp, Tipdir
from torchsummary import summary

logging.basicConfig(level=logging.INFO)


if __name__=='__main__':
    args = param_mana.parse_args_eval()
    label_id_str, label_map = param_mana.analysis_labels(args.label_type)
    # Torch device
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print('Torch Using device:', device)
    # Load network
    net = torch.load(args.network)
    summary(net, (1*args.use_depth + 3*args.use_rgb, args.img_size, args.img_size))
    # summary(net, (input_channels, args.img_size, args.img_size))

    # Load dataset
    logging.info('Loading {} Dataset...'.format(args.dataset.title()))
    test_data, test_dataset = load_testset(args, label_map)
    logging.info('Load dataset and network done')

    # Test
    results_array = []
    trial_time = 1

    for i in range(trial_time):
        results = {'correct': 0, 'failed': 0}
        val_size = args.val_size if args.val_size != 0 else len(test_dataset)
        print("Action \n\n", i, "\n\n")
        with torch.no_grad():
            val_size_count = 0
            while not val_size_count > val_size:
                for idx, (x, y, didx, rot, zoom) in enumerate(test_dataset):
                    val_size_count +=1
                    if val_size_count > val_size: break
                    # Generate test time augmentation data
                    x_arr, tta_belta = tta_process.TTA_input_generation(x.cpu().numpy(), args.tta_size)
                    x_arr_tensor = torch.from_numpy(x_arr).to(device)
                    
                    # Inference on TTA inputs
                    post_preds = network_evaluation.post_process_output(net(x_arr_tensor), label_map)
                    
                    # Gether aug. params/inputs/preds
                    tta_samples = tta_process.TTA_organize_data(tta_belta, x_arr, post_preds)

                    # TTA modeling
                    tta_estimation, regulated_sample_data = tta_process.TTA_modeling_std(tta_samples, label_map)

                    # Visualize inference midpoints
                    gt_bbs = test_data.dataset.get_bbs(idx, rot, zoom)
                    (qua_vote, ang_vote, wid_vote), grasp_type = \
                        param_mana.decode_grasp_detection(tta_estimation, label_map, (Grasp, Tipdir))
                    detect_gs = network_evaluation.detect_grasps(qua_vote, ang_vote, wid_vote, grasp_type)

                    if args.vis:
                        output_display.tta_gripmap(regulated_sample_data, args.vis_timeout)
                        output_display.plot_output(tta_estimation, detect_gs, gt_bbs)
                    # IOU evaluation
                    if args.iou_eval:
                        # grsap type different
                        s = network_evaluation.calculate_iou(detect_gs, gt_bbs)
                        logging.info('Processing {:0>3d}/{}; Augment:r {:.2f}, z {:.2f} -- Result: {}'\
                            .format(val_size_count, val_size, rot, zoom, 'correct' if s else 'failed'))
                        results ['correct' if s else 'failed'] += 1

        # IOU summary
        if args.iou_eval:
            logging.info('IOU Results: %d/%d = %f' % (results['correct'],
                        results['correct'] + results['failed'],
                        results['correct'] / (results['correct'] + results['failed'])))
        
        results_array.append(results['correct'])
        import numpy as np
        print('mean: ', np.mean(results_array))
        print('mean: ', np.mean(results_array) / val_size)
    for ind in range(trial_time): print(results_array[ind])

    