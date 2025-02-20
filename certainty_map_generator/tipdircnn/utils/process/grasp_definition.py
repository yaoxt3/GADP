import math
import numpy as np
import matplotlib.pyplot as plt

from skimage.draw import polygon
from torch import float32


class GraspRectangles:
    """
    Convenience class for loading and operating on sets of Grasp Rectangles.
    """
    def __init__(self, grarects=None):
        if grarects:
            self.grarects = grarects
        else:
            self.grarects = []
    
    def __getitem__(self, idx):
        return self.grarects[idx]

    def __iter__(self):
        return self.grarects.__iter__()

    def __getattr__(self, attr): # question
        """
        Test if GraspRectangle has the desired attr as a function and call it. 
        """
        # Fuck yeah python.
        if hasattr(GraspRectangle, attr) and callable(getattr(GraspRectangle, attr)):
            return lambda *args, **kwargs: list(map(lambda gr: getattr(gr, attr)(*args, **kwargs), 
                self.grarects))
        else:
            raise AttributeError("Couldn't find function %s in BoundingBoxes or BoundingBox" % attr)
    
    @staticmethod
    def _gr_text_to_no(l, offset=(0, 0)): # string convert to int(type) point coordinate
        """
        Transform a single point from a Cornell file line to a pair of ints.
        :param l: Line from Cornell grasp file (str)
        :param offset: Offset to apply to point positions
        :return: Point [y, x]
        """
        x, y = l.split()
        return [int(round(float(y))) - offset[0], int(round(float(x))) - offset[1]]

    @classmethod
    def load_from_array(cls, arr):
        """
        Load grasp rectangles from numpy array.

        :param Nx4x2 array arr: where each 4x2 array is the 4 corner pixels of a grasp rectangle.
        :return GraspRectangles(): GraspRectangles()
        """
        grarects = []
        for i in range(arr.shape[0]):
            grp_i = arr[i, :, :].squeeze()
            if grp_i.max() == 0: # in case that data initialized by not given
                break
            else:
                grarects.append(GraspRectangle(grp_i))
        return cls(grarects)
    
    @classmethod
    def load_from_cornell_file(cls, fname):
        """
        Load grasp rectangles from a Cornell dataset grasp file.

        :param (text) fname: Path to text file.
        :return GraspRectangles(): GraspRectangles()
        """
        grarects = []
        with open(fname) as f:
            while True:
                # Load 4 lines at a time for corners
                p0 = f.readline()
                if not p0:
                    break # EOF
                p1, p2, p3  = f.readline(), f.readline(), f.readline()
                try:
                    grp_i = np.array([
                        __class__._gr_text_to_no(p0),
                        __class__._gr_text_to_no(p1),
                        __class__._gr_text_to_no(p2),
                        __class__._gr_text_to_no(p3)
                    ])
                    grarects.append(GraspRectangle(grp_i))

                except ValueError:
                    # some files contain weird values.
                    continue
        return cls(grarects)
    
    @classmethod
    def load_from_jacquard_file(cls, fname, scale=1.0):
        """
        Load grasp rectangles from a Jacquard dataset file.
        :param fname: Path to file.
        :param scale: Scale to apply (e.g. if resizing images)
        :return: GraspRectangles()
        """
        gras = []
        with open(fname) as f:
            for l in f:
                x, y, theta, w, h = [float(v) for v in l[:-1].split(';')]
                gras.append(Grasp(np.array([y,x]), -theta/180.1*np.pi, w, h).as_grarect)
        grarects = cls(gras)
        grarects.scale(scale) # may be how cls.__getattr__() works
        return grarects

    def append(self, grarect):
        """
        Add a grasp rectangle to this GraspRectangles object
        :param gr: GraspRectangle
        """
        self.grarects.append(grarect)
    
    def copy(self):
        """
        :return: A deep copy of this object and all of its GraspRectangles.
        """
        new_grs = GraspRectangles()
        for grarect in self.grarects:
            new_grs.append(grarect.copy())
        return new_grs

    def show(self, ax=None, shape=None):
        """
        Draw all GraspRectangles on a matplotlib plot.
        :param ax: (optional) existing axis
        :param shape: (optional) Plot shape if no existing axis
        """
        if ax is None:
            f = plt.figure()
            ax = f.add_subplot(1, 1, 1)
            ax.imshow(np.zeros(shape))
            ax.axis([0, shape[1], shape[0], 0])
            self.plot(ax) # may be how cls.__getattr__() works
            plt.show()
        else:
            self.plot(ax)
    

    def draw_pos_map(self, shape, position=True, angle=True, width=True, pos_ratio = 1.0/3):
        """
        Plot all GraspRectangles as solid rectangles in a numpy array, one third (default) of the rect
        :param shape: output shape
        :param position: If True, Q output will be produced
        :param angle: If True, Angle output will be produced
        :param width: If True, Width output will be produced
        :return: Q, Angle, Width outputs (or None)
        """
        if position:
            qua_pos = np.zeros(shape)
        else:
            qua_pos = None
        if angle:
            ang_pos = np.zeros(shape)
        else:
            ang_pos = None
        if width:
            wid_pos = np.zeros(shape)
        else:
            wid_pos = None
        for grarect in self.grarects:
            rr, cc = grarect.compact_polygon_coords(shape, pos_ratio)
            if position:
                qua_pos[rr, cc] = 1.0
            if angle:
                ang_pos[rr, cc] = grarect.angle
            if width:
                # network input and grasp definition not matched
                wid_pos[rr, cc] = grarect.width
        return qua_pos, ang_pos, wid_pos
        

    def draw_tip_map(self, shape, position=True, angle=True, width=True, tip_ratio=1.0/4): # create label
        """
        Plot all GraspRectangles as tipdir representation in a numpy array
        :param shape: output shape
        :param position: If True, Q output will be produced
        :param angle: If True, Angle output will be produced
        :param width: If True, Width output will be produced
        :return: Q, Angle, Width outputs (or None)
        """
        if position:
            qua_tip = np.zeros(shape)
        else:
            qua_tip = None
        if angle:
            ang_tip = np.zeros(shape)
        else:
            ang_tip = None
        if width:
            wid_tip = np.zeros(shape)
        else:
            wid_tip = None
        for grarect in self.grarects:
            (lrr, lcc), (rrr, rcc) = grarect.compact_polygon_coords_tip(shape, tip_ratio)
            if position:
                qua_tip[lrr, lcc] = 1.0
                qua_tip[rrr, rcc] = 1.0
            if angle:
                ang_tip[lrr, lcc] = grarect.langle
                ang_tip[rrr, rcc] = grarect.rangle
            if width:
                wid_tip[lrr, lcc] = grarect.width
                wid_tip[rrr, rcc] = grarect.width

        return qua_tip, ang_tip, wid_tip
    

    def to_array(self, pad_to=0):
        """
        Convert all GraspRectangles to a single array.
        :param pad_to: Length to 0-pad the array along the first dimension
        :return: Nx4x2 numpy array
        """
        a = np.vstack([grarect.points for grarect in self.grarects])
        if pad_to:
            if pad_to > len(self.grarects):
                a = np.concatenate((a, np.zeros((pad_to - len(self.grarects), 4, 2))))
        return a.astype(np.intc)
    
    @property
    def center(self):
        """
        Compute mean center of all GraspRectangles
        :return: float, mean centre of all GraspRectangles
        """
        points = [grarect.points for grarect in self.grarects]
        return np.mean(np.vstack(points), axis=0).astype(np.intc) 



class GraspRectangle:
    """
    Representation of well-known grasp rectangle
    """
    def __init__(self, points):
        self.points = points

    def __str__(self):
        return str(self.points)

    @property
    def angle(self):
        """
        :return: Angle of the grasp to the horizontal (-pi/2, pi/2)
        """
        dx = self.points[1, 1] - self.points[0, 1]
        dy = self.points[1, 0] - self.points[0, 0]
        return (np.arctan2(-dy, dx) + np.pi/2) % np.pi - np.pi/2
    
    @property
    def full_angle(self):
        """
        :return: Angle of the grasp to the horizontal (-pi, pi)
        """
        dx = self.points[1, 1] - self.points[0, 1]
        dy = self.points[1, 0] - self.points[0, 0]
        return (np.arctan2(-dy, dx) + np.pi/2) % np.pi - np.pi/2

    @property
    def langle(self):
        """
        :return: Angle of the left tip direction to the horizontal
        """
        dx = self.points[1, 1] - self.points[0, 1]
        dy = self.points[1, 0] - self.points[0, 0]
        return np.arctan2(-dy, dx)

    @property
    def rangle(self):
        """
        :return: Angle of the right tip direction to the horizontal
        """
        dx = self.points[0, 1] - self.points[1, 1]
        dy = self.points[0, 0] - self.points[1, 0]
        return np.arctan2(-dy, dx)

    @property
    def as_grasp(self):
        """
        Return Grasp format(center, angle, length, width) of rectangle form

        :return Grasp class: object type
        """
        return Grasp(self.center, self.angle, self.width, self.length)

    @property
    def as_tipdir(self, tip_rate = 1./4):
        """
        tipdir format(center, angle, length, width) of rectangle form
        :param tip_rate: the ratio of mapping
        :return Tipdir class: object type
        """
        lps, rps = self.tip_points(tip_rate)

        left_rect = GraspRectangle(lps)
        right_rect = GraspRectangle(rps)
        
        letd = Tipdir(left_rect.center, self.langle, left_rect.width, left_rect.length)
        ritd = Tipdir(right_rect.center, self.rangle, right_rect.width, right_rect.length)
        return letd, ritd

    @property
    def center(self):
        """
        :return int: Rectangle center point
        """
        return self.points.mean(axis=0).astype(np.int32)
    
    @property
    def width(self):
        """
        :return float: rectangle width
        """
        dx = self.points[1, 1] - self.points[0, 1]
        dy = self.points[1, 0] - self.points[0, 0]
        return np.sqrt(dx ** 2 + dy ** 2)
    
    @property
    def length(self):
        dy = self.points[2, 1] - self.points[1, 1]
        dx = self.points[2, 0] - self.points[1, 0]
        return np.sqrt(dx ** 2 + dy ** 2)

    def tip_points(self, tip_rate):
        """
        :param tip_rate: the ratio of mapping
        """
        ps = self.points
        lps = np.array([ps[0],  ps[0] + (ps[1]-ps[0])*tip_rate, 
                        ps[3] + (ps[2]-ps[3])*tip_rate, ps[3]]).astype(np.intc)
        rps = np.array([ps[0] + (ps[1]-ps[0])*(1.0-tip_rate), ps[1], 
                        ps[2], ps[3] + (ps[2]-ps[3])*(1.0-tip_rate)]).astype(np.intc)
        return lps, rps

    def polygon_coords(self, shape=None):
        """
        :param shape: Output Shape
        :return: Indices of pixels within the grasp rectangle polygon.
        """
        return polygon(self.points[:, 0], self.points[:, 1], shape)

    def compact_polygon_coords(self, shape, pos_rate = 1.0/3):
        """
        :param shape: Output shape
        :param pos_rate: the ratio of mapping
        :return: Indices of pixels within the centre thrid of the grasp rectangle.
        """
        return Grasp(self.center, self.angle, self.width*pos_rate, self.length).as_grarect.polygon_coords(shape)

    def compact_polygon_coords_tip(self, shape, tip_rate = 1.0/4):
        """
        :param shape: Output shape
        :param tip_rate: the ratio of mapping
        :return: Indices of pixels on both sides(left right) of grasp in quanter.
        """
        lps, rps = self.tip_points(tip_rate)
        return GraspRectangle(lps).polygon_coords(shape), GraspRectangle(rps).polygon_coords(shape)

    def iou(self, gr, angle_threshold = np.pi/6):
        """
        Compute IoU with another grasping rectangle
        :param gr: GraspingRectangle to compare
        :param angle_threshold: Maximum angle difference between GraspRectangles
        :return: IoU between Grasp Rectangles
        """
        if abs((self.angle - gr.angle + np.pi/2) % np.pi - np.pi/2) > angle_threshold: # angle thre
            return 0
        rr1, cc1 = self.polygon_coords()
        rr2, cc2 = polygon(gr.points[:, 0], gr.points[:, 1])

        try:
            r_max = max(rr1.max(), rr2.max()) + 1 # max empty map 
            c_max = max(cc1.max(), cc2.max()) + 1
        except:
            return 0
        canvas = np.zeros((r_max, c_max)) # empty map
        canvas[rr1, cc1] += 1 # first rect +1
        canvas[rr2, cc2] += 1 # second rect +1
        union = np.sum(canvas > 0) # covered area
        if union == 0:
            return 0
        intersection = np.sum(canvas == 2) # both grasps covered area
        return intersection/union # percent of both in either covered

    def copy(self):
        """
        :return: Copy of self.
        """
        return GraspRectangle(self.points.copy())

    def offset(self, offset):
        """
        Offset grasp rectangle
        :param offset: array [y, x] distance to offset
        """
        self.points += np.array(offset).reshape((1,2)) # ?

    def rotate(self, angle, center):
        """
        Rotate grasp retangle

        :param angle: in radians
        :param center: point to rotate around (center of images)
        """
        R = np.array(
            [
                np.array([math.cos(-angle), math.sin(-angle)]),
                np.array([-1 * math.sin(-angle), math.cos(-angle)])
            ]
        )
        c = np.array(center).reshape((1, 2))
        self.points = ((np.dot(R, (self.points - c).T)).T + c).astype(np.int32)

    def scale(self, factor):
        """
        :param float factor: factor
        """
        if factor == 1.0:
            return
        self.points *= factor

    def plot(self, ax, color=None):
        """
        Plot grasping rectangle.
        :param ax: Existing matplotlib axis
        :param color: matplotlib color code (optional)
        """
        points = np.vstack((self.points, self.points[0]))
        ax.plot(points[:, 1], points[:, 0], linewidth=4, color=color, alpha = 0.6)


    def zoom(self, factor, center):
        """
        Zoom grasp rectangle by given factor.
        :param factor: Zoom factor e.g. 0.5 will keep the center 50% of the image.
        :param center: Zoom zenter (focus point, e.g. image center) 
        """
        
        T = np.array([
                np.array([1, 0]),
                np.array([0, 1])
            ])
        T = T/factor
        c = np.array(center).reshape((1, 2))
        self.points = ((np.dot(T, (self.points - c).T)).T + c).astype(np.int32)

    
class Grasp:
    """
    A Grasp represented by a center pixel, rotation angle and gripper width (length)
    """
    def __init__(self, center, angle, width=60, length=None):
        self.center = center
        self.angle = angle
        self.width = width
        self.length = self.width / 2 if length==None else length

    @property
    def as_grarect(self):
        """
        Convert to GraspRectangle
        :return: GraspRectangle representation of grasp.
        """
        xo = np.cos(self.angle)
        yo = np.sin(self.angle)

        y1 = self.center[0] + self.width / 2 * yo
        x1 = self.center[1] - self.width / 2 * xo
        y2 = self.center[0] - self.width / 2 * yo
        x2 = self.center[1] + self.width / 2 * xo

        return GraspRectangle(np.array(
            [
             [y1 - self.length/2 * xo, x1 - self.length/2 * yo],
             [y2 - self.length/2 * xo, x2 - self.length/2 * yo],
             [y2 + self.length/2 * xo, x2 + self.length/2 * yo],
             [y1 + self.length/2 * xo, x1 + self.length/2 * yo],
            ]
        ).astype(np.float32))


    def max_iou(self, grarects):
        """
        Return maximum IoU between self and a list of GraspRectangles
        :param grs: List of GraspRectangles
        :return: Maximum IoU with any of the GraspRectangles
        """
        # convert to grasp rect to get the iou max fragtion
        self_grarect = self.as_grarect
        max_iou = 0
        for grarect in grarects:
            iou = self_grarect.iou(grarect)
            max_iou = max(max_iou, iou)
        return max_iou

    def plot(self, ax, color=None):
        """
        Plot Grasp
        :param ax: Existing matplotlib axis
        :param color: (optional) color
        """
        self.as_grarect.plot(ax, color)

    def to_jacquard(self, scale=1):
        """
        Output grasp in "Jacquard Dataset Format" (https://jacquard.liris.cnrs.fr/database.php)
        :param scale: (optional) scale to apply to grasp
        :return: string in Jacquard format
        """
        # Output in jacquard format.
        return '%0.2f;%0.2f;%0.2f;%0.2f;%0.2f' % (self.center[1]*scale, self.center[0]*scale,
            -1*self.angle*180/np.pi, self.width*scale, self.length*scale)

class Tipdir:
    """
    A representation of finger tip and direction.
    """
    def __init__(self, center, angle, width=60/4, length=None):
        """
        Definition of the tipdir
        :param center: tip center
        :param angle: angle -pi to pi to horizontal
        :param width: quanter of rectangle, defaults to 60/4
        :param length:same length of rectangle, double of tip width, defaults to 60/2
        """
        self.center = center
        self.angle = angle
        self.width = width
        self.length = self.width * 2 if length == None else length

    def plot(self, ax, color=None, draw_circle=False, draw_rectangle=True, startdot=False):
        """
        Plot grasping tip dir.
        :param ax: Existing matplotlib axis
        :param color: matplotlib color code (optional)
        """
        td_rect = self.as_rect
        points = np.vstack((td_rect.points, td_rect.points[0]))
        if draw_rectangle: ax.plot(points[:, 1], points[:, 0], linewidth=4,  color=color, alpha = 0.6)

        # dirction = np.array([self.center, self.center + self.width * np.array([-np.sin(self.angle), np.cos(self.angle)])])
        # ax.plot(dirction[:, 1], dirction[:, 0], marker='o', markersize=6, color=color)
        
        radius = 18
        cir_space = np.linspace(0, 2*np.pi, num=50)
        circle_curve = np.tile(self.center, (50, 1)) + radius * np.array([np.sin(cir_space), np.cos(cir_space)]).transpose()
        if draw_circle: ax.plot(circle_curve[:, 1], circle_curve[:, 0], linewidth=2,  color=color, alpha = 0.6)

        circle_curve_dot = np.tile(self.center, (50, 1)) + 2 * np.array([np.sin(cir_space), np.cos(cir_space)]).transpose()
        if startdot: ax.plot(circle_curve_dot[:, 1], circle_curve_dot[:, 0], linewidth=4,  color=color, alpha = 1)

        x, y = self.center[1], self.center[0]
        dy ,dx = radius * 1.2 * -np.sin(self.angle), radius * 1.2 * np.cos(self.angle)
        size_arrow = 1.0
        ax.arrow(x, y, dx, dy, width=size_arrow*2, head_length=size_arrow*10, 
                    head_width = size_arrow*10, color=color, alpha = 1)

    @property
    def as_rect(self):
        """
        :return: return the rectangle of the finger tip with diretion
        """
        return Grasp(self.center, self.angle, self.width, self.length).as_grarect

    def polygon_coords(self, shape=None):
        """
        :param shape: Output Shape
        :return: Indices of pixels within the grasp rectangle polygon.
        """
        as_rc = self.as_rect
        return polygon(as_rc.points[:, 0], as_rc.points[:, 1], shape)

    def iou(self, td, angle_threshold=np.pi/6):
        """
        Calculate two tipdirs iou
        :param td: tipdir object
        :param angle_threshold: defaults to np.pi/6
        """
        
        delta_angle = abs(self.angle - td.angle)
        if min(delta_angle, np.pi*2 - delta_angle) > angle_threshold:
            return 0

        rr1, cc1 = self.polygon_coords()
        as_rc = td.as_rect
        rr2, cc2 = polygon(as_rc.points[:, 0], as_rc.points[:, 1])

        try:
            r_max = max(rr1.max(), rr2.max()) + 1 # max empty map 
            c_max = max(cc1.max(), cc2.max()) + 1
        except:
            return 0
        canvas = np.zeros((r_max, c_max)) # empty map
        canvas[rr1, cc1] += 1 # first rect +1
        canvas[rr2, cc2] += 1 # second rect +1
        union = np.sum(canvas > 0) # covered area
        if union == 0:
            return 0
        intersection = np.sum(canvas == 2) # both grasps covered area
        return intersection/union # percent of both in either covered

    def dis_cross(self, td, angle_threshold=np.pi/6, dist_threshold=30):
        delta_angle = abs(self.angle - td.angle)
        if min(delta_angle, np.pi*2 - delta_angle) > angle_threshold:
            return 0
        
        delta_xy = np.array(self.center) - np.array(td.center)
        dist_act = np.sqrt(np.sum(delta_xy**2))
        iou = 0.25/(float(dist_act + 0.01)/float(dist_threshold))
        # print('delta_xy', delta_xy, 'iou', iou)
        return iou

    def max_iou(self, grarects, angle_threshold = np.pi/6, cal_dis=20):
        """
        Compute IoU with grasp rectangle in single tipdir 
        :param  gr: other grasp rectangle (not tipdir) 
        :param  angle_threshold: defaults to np.pi/6
        :return: IoU
        """
        max_iou = 0
        for gr in grarects:
            for tdx in gr.as_tipdir:
                if cal_dis == 0: 
                    iou = self.iou(tdx, angle_threshold)
                else: 
                    iou = self.dis_cross(tdx, angle_threshold, cal_dis)
                max_iou = max(max_iou, iou)
        return max_iou



