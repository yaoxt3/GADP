from . import get_dataset
import torch 

def load_testset(args, label_map):
    Dataset = get_dataset(args.dataset)
    test_dataset = Dataset(
        args.dataset_path, start=args.split, end=1.0, ds_rotate=args.ds_rotate,
        label_map = label_map, output_size=args.img_size,
        random_rotate=args.augment, random_zoom=args.augment, 
        include_depth=args.use_depth, include_rgb=args.use_rgb)
    test_data = torch.utils.data.DataLoader(
        test_dataset,
        batch_size=1,
        shuffle=args.shuffle,
        num_workers=args.num_workers
    )
    return test_data, test_dataset
