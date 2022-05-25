################################################################################
# ROBOTIC VISION - EXERCISE 3. GET CAMERA POSITION USING TAG DETECTION.
# MASTER DEGREE IN COMPUTER VISION.
# UNIVERSIDAD REY JUAN CARLOS (URJC).
# 
# Autor:
# VICENTE GILABERT MAÑÓ
#
#################################################################################
#################################################################################
#IMPORTS

import cv2
import glob
import copy
import numpy as np
import scipy.misc as scpm
import matplotlib.pyplot as plt
import os
import argparse
from datetime import datetime
import time

################################################################################
#FUNCTIONS

def parse_arguments():
    '''
    use argparse to get aguments.
    '''
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--folder", type=str, required=True, help="path to folder with images to calibrate camera.")
    ap.add_argument("-sh", "--chess_shape", type=tuple, default=(9,6), help= "Shape. Number of rows and columns to detect in chessboard")
    ap.add_argument("-sz", "--chess_size", type=float, default=24, help= "Size of every rectangle of chessboards (mm)")
    ap.add_argument("-sv", "--path_save", type=str, default="results_calibration/", help="Path to save calibration result.")
    ap.add_argument("-p", "--plot_cameras", default=False, action='store_true', help="Activate flag to 3D plot cameras position.")

    return vars(ap.parse_args())


def load_images(path):
    '''
    load images in a folder.
    '''
    images = []
    names = []
    if os.path.isdir(path):
        for file in os.listdir(path):
            if file.lower().endswith(('.png', '.jpg', '.jpeg')):
                img_path = os.path.join(path, file)
                image = cv2.imread(img_path)
                images.append(image)
                names.append(file)

    return images, names


def corners_detection(images, files, chessboard_shape, verbose=True):
    '''
    corners detection of chessboard images.
    '''
    corners = []
    imgs = []

    for img, name in zip(images, files):
        found, img_corners = cv2.findChessboardCorners(img, chessboard_shape)
        if found:
            if verbose: print(name, ': chessboard found!')
            # img_corners[:,0,:]
            corners.append(img_corners[:, 0, :])
            imgs.append(img)
        else:
            if verbose: print(name, ': chessboard NOT found!')

    return corners, imgs


def refine_corners_detection(image, corners, search_window=(11, 11), zero_zone=(-1, -1),
                 criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)):
    '''
    Refine corners detection of chessboard images.
    '''
    cornersRefined = cv2.cornerSubPix(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), corners, search_window, zero_zone, criteria)

    return cornersRefined


def get_chessboard_points(chessboard_shape, dx, dy):
    '''
    Create a 3D chessboard points.
    '''
    dim = chessboard_shape[0] * chessboard_shape[1]
    cb_points = np.zeros((dim, 3))

    d = 0
    for x in range(chessboard_shape[1]):
        for y in range(chessboard_shape[0]):
            cb_points[d][0] = x * dx
            cb_points[d][1] = y * dy
            d += 1

    return cb_points


def plot3DPoints(Pts, axes, dist):
    '''
    Plot points in 3D. Alse plot references axis.
    '''
    x = Pts[:, 0]
    y = Pts[:, 1]
    z = Pts[:, 2]
    axes.scatter3D(x, y, z, color='yellow', edgecolor='black')
    # se pintan los ejes del sistema de referencia del plano
    axes.plot3D((0, dist), (0, 0), (0, 0), '-r', linewidth=2)
    axes.plot3D((0, 0), (0, dist), (0, 0), '-g', linewidth=2)
    axes.plot3D((0, 0), (0, 0), (0, dist), '-b', linewidth=2)


def plotCamera3D(K, rvec, tvec, dist, axes=None):
    if axes is None:
        axes = plt.axes(projection='3d')

    tvec = tvec.ravel()
    R, _ = cv2.Rodrigues(rvec)

    # Centro optico de la camara en coordenadas de la escena (world).
    # C = -inv(R)*t = -R.T * t --> R.T = inv(R) porque es R es ortonormal.
    C_opt_esc = -np.linalg.inv(R) @ tvec  # Otra forma: C_opt_esc = -R.T @ tvec
    # print("Centro optico camara en coordenadas de la escena:\n", C_opt_esc)
    axes.scatter3D(C_opt_esc[0], C_opt_esc[1], C_opt_esc[2])

    # Centro optico de la camara en coordenadas de cámara. Deberia ser (0,0,0).
    C_opt_cam = R @ C_opt_esc + tvec

    pt1 = np.array([dist, 0, 0]).T
    pt2 = np.array([0, dist, 0]).T
    pt3 = np.array([0, 0, dist]).T
    pt1_esc = R.T @ (pt1 - tvec)
    pt2_esc = R.T @ (pt2 - tvec)
    pt3_esc = R.T @ (pt3 - tvec)

    axes.plot3D([C_opt_esc[0], pt1_esc[0]], [C_opt_esc[1], pt1_esc[1]], [C_opt_esc[2], pt1_esc[2]], '-r', linewidth=2)
    axes.plot3D((C_opt_esc[0], pt2_esc[0]), (C_opt_esc[1], pt2_esc[1]), (C_opt_esc[2], pt2_esc[2]), '-g', linewidth=2)
    axes.plot3D((C_opt_esc[0], pt3_esc[0]), (C_opt_esc[1], pt3_esc[1]), (C_opt_esc[2], pt3_esc[2]), '-b', linewidth=2)


def calibration(path, params):

    chessboard_shape = params[0]
    chessboard_size = params[1]
    save_result = params[2]
    plot3D_cameras = params[3]

    # Load all images in folder.
    all_images, names = load_images(path)
    # Corner detection.
    corners, imgs = corners_detection(all_images, names, chessboard_shape, verbose=True)
    # Refine corner detection. Create a deepcopy to avoid erase problems.
    corners2 = copy.deepcopy(corners)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cornersRefined = [refine_corners_detection(imgs[i], corners2[i], criteria=criteria) for i in range(len(corners2))]
    # Create chessboard points
    cb_points = get_chessboard_points(chessboard_shape, chessboard_size, chessboard_size)

    num_imgs = len(imgs)
    num_points = corners[0].shape

    ################  Prepare input data  ################
    # object_points: numpy array with dimensions (number_of_images, number_of_points, 3)
    object_points = np.repeat(np.expand_dims(cb_points, axis=0), num_imgs, axis=0).astype('float32')
    #print(object_points.shape)

    # image_points: numpy array with dimensions (number_of_images, number_of_points, 2)
    image_points = np.array(cornersRefined).astype('float32')
    #print(image_points.shape)

    # image size
    img_size = imgs[0].shape[:2]
    #print("Tamaño de imagen: ", img_size)

    #####################################################
    # Calibrate for square pixels corners standard
    rms, intrinsics, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, img_size, None, None,
                                                                     flags=cv2.CALIB_FIX_ASPECT_RATIO)

    time.sleep(1)
    print("------------------------- RESULTADOS CALIBRACION -----------------------------")
    print("Parámetros intrinsecos (K):\n", intrinsics)
    print("Coeficientes de distorsion:\n", dist_coeffs)
    print("Error RMS:\n", rms)

    if save_result is not None:
        if not os.path.exists(save_result):
            os.makedirs(save_result)

        now = datetime.now()
        dt_string = now.strftime("%Y%m%d_%H%M%S")
        name_file = "parameters_" + dt_string + ".npy"
        save_path = os.path.join(save_result, name_file)
        with open(save_path, 'wb') as f:
            np.save(f, intrinsics)
            np.save(f, dist_coeffs)
            np.save(f, rvecs)
            np.save(f, tvecs)

    if plot3D_cameras:
        plt.figure()
        axes = plt.axes(projection='3d')
        axes.set_xlabel('X')
        axes.set_ylabel('Y')
        axes.set_zlabel('Z')

        dist = chessboard_size * 3
        plot3DPoints(cb_points, axes, dist)  # pintar esquinas del "ajedrez" en 3D

        for i in range(len(rvecs)):
            plotCamera3D(intrinsics, rvecs[i], tvecs[i], dist, axes)

        scaling = np.array([getattr(axes, 'get_{}lim'.format(dim))() for dim in 'xyz']);
        axes.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]] * 3)
        plt.show()


################################################################################
# MAIN

if __name__ == '__main__':
    args = parse_arguments()
    path_imgs = args['folder']

    if os.path.isdir(path_imgs) and len(os.listdir(path_imgs)) != 0:
        params = [args['chess_shape'], args['chess_size'], args['path_save'], args['plot_cameras']]
        calibration(path_imgs, params)
    else:
        print("Folder not exist or is empty.")
