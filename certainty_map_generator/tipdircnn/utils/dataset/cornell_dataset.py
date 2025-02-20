import os
import glob

from .dataset_base import GraspDatasetBase
from utils.process.grasp_definition import GraspRectangles
from utils.process.image_definition import DepthImage, Image

class CornellDataset(GraspDatasetBase):
    """
    Dataset wrapper for the Cornell dataset.
    """
    def __init__(self, file_path, start=0.0, end=1.0, ds_rotate=0, **kwargs):
        """
        :param file_path: Cornell Dataset directory.
        :param start: If splitting the dataset, start at this fraction [0,1]
        :param end: If splitting the dataset, finish at this fraction
        :param ds_rotate: If splitting the dataset, rotate the list of items by this fraction first
        :param kwargs: kwargs for GraspDatasetBase
        """
        super(CornellDataset, self).__init__(**kwargs)

        graspf = glob.glob(os.path.join(file_path, '*', 'pcd*cpos.txt'))
        graspf.sort()

        l = len(graspf)
        if l == 0:
            raise FileNotFoundError('No dataset found. Check path: {}'.format(file_path))

        if ds_rotate:
            graspf = graspf[int(l*ds_rotate):] + graspf[:int(l*ds_rotate)]

        depthf = [f.replace('cpos.txt', 'd.tiff') for f in graspf]
        rgbf = [f.replace('d.tiff', 'r.png') for f in depthf]

        self.grasp_files = graspf[int(l*start):int(l*end)]
        self.depth_files = depthf[int(l*start):int(l*end)]
        self.rgb_files = rgbf[int(l*start):int(l*end)]

        self.data_files = self.grasp_files # base attribute

    def _get_crop_attrs(self, idx):
        """
        As network input is 300x300 croped by 640x480, all data need obtain the crop center 

        :param idx: index
        """
        gtbbs = GraspRectangles.load_from_cornell_file(self.grasp_files[idx])
        center = gtbbs.center

        left = max(0, min(center[1] - self.output_size // 2, 640 - self.output_size))
        top = max(0, min(center[0] - self.output_size // 2, 480 - self.output_size))
        return center, left, top

    def get_bbs(self, idx, rot=0, zoom=1.0):
        bbs = GraspRectangles.load_from_cornell_file(self.grasp_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        bbs.rotate(rot, center)
        bbs.offset((-top, -left))
        bbs.zoom(zoom, (self.output_size//2, self.output_size//2))
        return bbs

    def get_depth(self, idx, rot=0, zoom=1):
        depth_img = DepthImage.from_tiff(self.depth_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        depth_img.rotate(rot, center)
        depth_img.crop((top, left), (min(480, top + self.output_size), 
            min(640, left + self.output_size)))
        depth_img.normalise()
        depth_img.zoom(zoom)
        depth_img.resize((self.output_size, self.output_size))
        return depth_img.img

    def get_rgb(self, idx, rot=0, zoom=1.0, normalise=True):
        rgb_img = Image.from_file(self.rgb_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        rgb_img.rotate(rot, center)
        rgb_img.crop((top, left), (min(480, top + self.output_size), 
            min(640, left + self.output_size)))
        rgb_img.zoom(zoom)
        rgb_img.resize((self.output_size, self.output_size))
        if normalise:
            rgb_img.normalise()
            rgb_img.img = rgb_img.img.transpose((2, 0, 1))
            # from(300, 300, 3) to (3, 300, 300); but rgb {not bgr in cv2}
            # to recover to skimage type (300, 300, 3), \
            # use re = np.transpose(ar, (1, 2, 0))
        return rgb_img.img



