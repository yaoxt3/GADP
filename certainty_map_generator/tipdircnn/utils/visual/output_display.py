import numpy as np
import cv2
import matplotlib.pyplot as plt
from utils.process.network_evaluation import detect_grasps
from skimage.filters import gaussian
import random

def key_close_event():
    while not plt.waitforbuttonpress():
        continue
    plt.close()


def qaw_to_tipspace(q_img, angle_img, width_img, density):
    """
    Conver the tip quality, angle, width image to 2d tipspace map
    :param q_img: as tipspace transparency
    :param angle_img: as tipspace angle
    :param width_img: as tipspace wipe distance
    :return: tipspace
    """
    H_img = q_img.shape[0]
    W_img = q_img.shape[1]
    tipspace_map = []
    for dir_y in np.arange(0, H_img, density, dtype=np.int16):
        for dir_x in np.arange(0, W_img, density, dtype=np.int16):
            start = np.array([dir_y, dir_x])
            width = width_img[dir_y][dir_x] + 4
            angle = angle_img[dir_y][dir_x]
            dir_a = width * np.array([-np.sin(angle), np.cos(angle)])

            end = np.clip(start + dir_a, 0, 300)
            # transparent = np.clip(q_img[dir_y][dir_x] + 0.5, 0.0, 1.0)
            transparent = q_img[dir_y][dir_x]

            tipspace_map.append((np.vstack((start, end)), transparent))
            # print(tipspace_map[-1])
    return tipspace_map

def plot_output_rect(rgb_img, depth_img, grasp_q_img, grasp_width_img, grasp_angle_img, 
                    no_grasps=1, gt_bbs=None, tipdir_rectg=False, timeout=False):
    """
    Plot the output of a GG-CNN
    :param rgb_img: RGB Image
    :param depth_img: Depth Image
    :param grasp_q_img: Q output of GG-CNN
    :param grasp_angle_img: Angle output of GG-CNN
    :param no_grasps: Maximum number of grasps to plot
    :param grasp_width_img: (optional) Width output of GG-CNN
    """
    gs = detect_grasps(grasp_q_img, grasp_angle_img, width_img=grasp_width_img, no_grasps=no_grasps,
                       tipdir_rectg=tipdir_rectg)
    vb = 2
    hb = 3
    fig = plt.figure(figsize=(hb * 5, vb * 5))
    ax = fig.add_subplot(vb, hb, 1)
    ax.imshow(rgb_img)
    for g in gs: # g is Grasp(), plot is privite function and plotted in rgb_img
        g.plot(ax)
    ax.set_title('RGB')

    ax = fig.add_subplot(vb, hb, 2)
    ax.imshow(depth_img, cmap='gray')
    for gr in gt_bbs:
        if tipdir_rectg:
            for td_i in gr.as_tipdir:
                td_i.plot(ax, color='white') # plot GROUNDTRUTH tipdir space densely
        else:
            gr.as_grasp.plot(ax, color='white') # plot GROUNDTRUTH tipdir space densely
    for g in gs:
        g.plot(ax) # plot [no_grasps] detection of tipdir
    ax.set_title('Depth')

    ts_map = qaw_to_tipspace(grasp_q_img, grasp_angle_img, grasp_width_img, density=6)
    ax = fig.add_subplot(vb, hb, 3)
    ax.imshow(rgb_img)
    for wiping, trans in ts_map:
        if trans < 0.2: # tipdir threshold
            continue
        x, y = wiping[0][1], wiping[0][0]
        dx, dy = wiping[1][1] - wiping[0][1], wiping[1][0] - wiping[0][0]
        colors = ['b', 'r', 'm', 'k']
        color = random.choice(colors) # choose one of the rotations in array
        size_arrow = 0.02
        ax.arrow(x, y, dx, dy, width=size_arrow, head_length=size_arrow*10, 
                    head_width = size_arrow*10, color=color, alpha = trans)
        # ax.plot(wiping[:, 1], wiping[:, 0], linewidth=1, alpha = trans)
    ax.set_title('Tipspace')

    ax = fig.add_subplot(vb, hb, 4)
    plot = ax.imshow(grasp_q_img, cmap='jet', vmin=0, vmax=1)
    ax.set_title('Q')
    plt.colorbar(plot)

    ax = fig.add_subplot(vb, hb, 5)
    width_range = 50 
    plot = ax.imshow(grasp_width_img, cmap='jet', vmin=0, vmax=width_range)
    ax.set_title('Width')
    plt.colorbar(plot)

    ax = fig.add_subplot(vb, hb, 6)
    angle_range = np.pi 
    plot = ax.imshow(grasp_angle_img, cmap='hsv', vmin=-angle_range, vmax=angle_range)
    ax.set_title('Angle')
    plt.colorbar(plot)

    timer = fig.canvas.new_timer(interval = 2500 if timeout else 10)
    timer.add_callback(plt.close if timeout else key_close_event)
    timer.start()
    plt.show()

def plot_output(tta_estimation, detect_gs, gt_bbs=None, timeout=False):
    """
    Plot the output
    :param img: Image
    :param grasp_q_img: Q output
    :param grasp_angle_img: Angle output
    :param no_grasps: Maximum number of grasps to plot
    :param grasp_width_img: (optional) Width output
    """
    img = tta_estimation['img_mean']
    grasp_q_img = tta_estimation['qua_mean']
    grasp_width_img = tta_estimation['wid_mean']
    grasp_angle_img = tta_estimation['csT_ang_mean']
    var_angle_img = tta_estimation['csT_vari_cs_mean']

    hb = 3 # width
    vb = 2 # height
    img_id = 1
    fig = plt.figure(figsize=(hb * 5, vb * 5))

    ax = fig.add_subplot(vb, hb, img_id)
    img_id += 1
    ax.imshow(img, cmap='gray')
    for gr in gt_bbs:
        for td in gr.as_tipdir:
            td.plot(ax, color='white') # plot GROUNDTRUTH tipdir space densely
    for g in detect_gs:
        g.plot(ax) # plot [no_grasps] detection of tipdir
    ax.set_title('Image')

    ts_map = qaw_to_tipspace(grasp_q_img, grasp_angle_img, grasp_width_img, density=8)
    ax = fig.add_subplot(vb, hb, img_id)
    img_id += 1
    ax.imshow(img)
    for wiping, trans in ts_map:
        if trans < 0.2 : 
            continue
        x, y = wiping[0][1], wiping[0][0]
        dx, dy = wiping[1][1] - wiping[0][1], wiping[1][0] - wiping[0][0]
        colors = ['b', 'r', 'm', 'k']
        color = random.choice(colors) # choose one of the rotations in array
        size_arrow = 1.0
        ax.arrow(x, y, dx, dy, width=size_arrow, head_length=size_arrow*4, 
                    head_width = size_arrow*5, color=color, alpha = trans)
        # ax.plot(wiping[:, 1], wiping[:, 0], linewidth=1, alpha = trans)
    ax.set_title('Tipspace')

    ax = fig.add_subplot(vb, hb, img_id)
    img_id += 1
    plot = ax.imshow(grasp_q_img, cmap='jet', vmin=0, vmax=1)
    ax.set_title('Quality')
    plt.colorbar(plot)

    ax = fig.add_subplot(vb, hb, img_id)
    img_id += 1
    angle_range = np.pi 
    plot = ax.imshow(grasp_angle_img, cmap='hsv', vmin=-angle_range, vmax=angle_range)
    ax.set_title('Angle')
    plt.colorbar(plot)

    ax = fig.add_subplot(vb, hb, img_id)
    img_id += 1
    width_range = 50 
    plot = ax.imshow(grasp_width_img, cmap='jet', vmin=0, vmax=width_range)
    ax.set_title('Width')
    plt.colorbar(plot)

    ax = fig.add_subplot(vb, hb, img_id)
    img_id += 1
    var_range = 1
    plot = ax.imshow(var_angle_img, cmap='jet', vmin=0, vmax=var_range)
    ax.set_title('var_angle')
    plt.colorbar(plot)

    timer = fig.canvas.new_timer(interval = 2500 if timeout else 10)
    timer.add_callback(plt.close if timeout else key_close_event)
    timer.start()
    plt.show()


def tta_gripmap(sample_data, timeout = False):
    rows = len(sample_data['img'])
    cols = len(sample_data)
    fig_size = 3
    fig_id = 0

    cmap_dic = {'img':None, 'qua':'jet', 'wid':'jet', 'aP':'jet', 'cP':'jet', 'sP':'jet', 
                'cT':'jet', 'sT':'jet', 'csT_ang':'jet', 'csT_var':'jet', 'csT_var_cs':'jet'}
    range_min = {'img':None, 'qua':0, 'wid':0, 'aP':-np.pi, 'cP':-1, 'sP':-1, 
                'cT':-1, 'sT':-1, 'csT_ang':-np.pi, 'csT_var':0, 'csT_var_cs':0}
    range_max = {'img':None, 'qua':1, 'wid':50, 'aP':np.pi, 'cP':1, 'sP':1, 
                'cT':1, 'sT':1, 'csT_ang':np.pi, 'csT_var':4, 'csT_var_cs':1}

    ## single data plot 
    fig = plt.figure(figsize=(cols * fig_size, rows * fig_size))
    for row_i in range(rows):
        for (kn, value) in sample_data.items():
            fig_id += 1
            ax = fig.add_subplot(rows, cols, fig_id)
            ax.set_title(kn)
            if (len(value) == 0): continue
            plot = ax.imshow(value[row_i], cmap=cmap_dic[kn], vmin=range_min[kn], vmax=range_max[kn])
            plt.colorbar(plot)

    timer = fig.canvas.new_timer(interval = 3000 if timeout else 10)
    timer.add_callback(plt.close if timeout else key_close_event)
    timer.start()
    plt.show()


def gridshow(imgs, scales, cmaps, width, border=10): 
    """
    Display images in a grid.
    :param imgs: List of Images (np.ndarrays)
    :param scales: The min/max scale of images to properly scale the colormaps
    :param cmaps: List of cv2 Colormaps to apply
    :param width: Number of images in a row
    :param border: Border (pixels) between images.
    """
    imgrows = []
    imgcols = []

    maxh = 0
    # imgs is 4 collections of 10 data maps 
    for i, (img, cmap, scale) in enumerate(zip(imgs, cmaps, scales)):
        # Scale images into range 0-1
        if scale is not None:
            img = (np.clip(img, scale[0], scale[1]) - scale[0])/(scale[1]-scale[0]+ 1e-6)
        elif img.dtype == np.float32:
            img = (img - img.min())/(img.max() - img.min() + 1e-6)

        # Apply colormap (if applicable) and convert to uint8
        # cmap is [0, 1] and may be [depth, rgb]
        if cmap is not None:
            try: 
                imgc = cv2.applyColorMap((img * 255).astype(np.uint8), cmap)
            except:
                imgc = (img*255.0).astype(np.uint8)
        else:
            imgc = img
        
        # Get the rgb data
        if imgc.shape[0] == 3:
            imgc = imgc.transpose((1, 2, 0))
            imgc = imgc[:, :, ::-1] # rgb to bgr for cv2
        elif imgc.shape[0] == 4:
            imgc = imgc[1:, :, :].transpose((1, 2, 0))
            imgc = imgc[:, :, ::-1] # rgb to bgr for cv2

        # Arrange row of images
        maxh = max(maxh, imgc.shape[0])
        imgcols.append(imgc)
        if i > 0 and i % width == (width - 1):
            imgrows.append(np.hstack([np.pad(c, ((0, maxh - c.shape[0]), (border//2, border//2), 
                (0, 0)), mode='constant') for c in imgcols]))
            imgcols = []
            maxh = 0
    
    # Unfinished row
    if imgcols:
        imgrows.append(np.hstack([np.pad(c, ((0, maxh - c.shape[0]), (border//2, border//2), 
            (0, 0)), mode='constant') for c in imgcols]))
        
    maxw = max([c.shape[1] for c in imgrows])

    image = np.vstack([np.pad(r, ((border//2, border//2), (0, maxw - r.shape[1]), (0, 0)), 
    mode='constant') for r in imgrows])

    return image
