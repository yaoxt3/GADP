def get_dataset(dataset_name):
    if dataset_name == 'cornell':
        from .cornell_dataset import CornellDataset
        return CornellDataset
    else:
        raise NotImplementedError('Dataset Type {} is not implemented yet.'.format(dataset_name))
    

