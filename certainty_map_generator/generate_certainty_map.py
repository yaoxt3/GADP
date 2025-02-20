from modules.cnn_predictor import CNNPredictor
from modules.param_set import params_set

import numpy as np
import logging, pickle
import os
from termcolor import cprint

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

if __name__ == '__main__':
    args = params_set()
    logging.info('Start CNNPredictor ...')
    predictor = CNNPredictor(args.network, args.input_channel, args.label_type, args.tta_size, args.img_size)

    file_path = '/media/yxt/Seagate 2TB/diffusion-data/2-5/'  # Set the path to the folder containing the pickle files
    demo_dirs = [os.path.join(file_path, f) for f in os.listdir(file_path) if f.endswith('.pkl')]

    for demo_dir in demo_dirs:
        dir_name = os.path.dirname(demo_dir)

        logging.info(f'Processing file: {demo_dir}')
        with open(demo_dir, 'rb') as f:
            demo = pickle.load(f)

        pcd_dirs = os.path.join(dir_name, 'pcd')
        if not os.path.exists(pcd_dirs):
            os.makedirs(pcd_dirs)

        # Add a new key to the dictionary
        certainty_key = 'certainty_map'
        certainty_maps = []
        demo[certainty_key] = certainty_maps

        demo_length = len(demo['point_cloud'])
        logging.info(f'Number of steps in demo: {demo_length}')
        for step_idx in range(demo_length):
            logging.info(f'Processing step {step_idx + 1}/{demo_length}')
            azure_rgb = demo['azure_rgb'][step_idx]
            azure_depth = demo['azure_depth'][step_idx]
            image = np.concatenate((np.expand_dims(azure_depth, axis=0), 
                                    np.transpose(azure_rgb, (2, 0, 1))), axis=0)
            
            input = predictor.input_process(image, args.mode == 'online', chann=args.input_channel)
            certainty_map, angle_map = predictor.inference(input)

            certainty_maps.append(certainty_map)

        # Save the updated dictionary back to the pickle file
        with open(demo_dir, 'wb') as f:
            pickle.dump(demo, f)
        logging.info(f'Updated file saved: {demo_dir}')