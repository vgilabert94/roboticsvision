################################################################################
# ROBOTIC VISION - EXERCISE 2. 3D RECONSTRUTION.
# MASTER DEGREE IN COMPUTER VISION.
# UNIVERSIDAD REY JUAN CARLOS (URJC).
# 
# Autor:
# VICENTE GILABERT MAÑÓ
#
#################################################################################
#################################################################################
#IMPORTS

from GUI import GUI
from HAL import HAL
import numpy as np
import cv2
from math import floor

################################################################################
#FUNCTIONS

def get_edges(img, sigma=0.33):
    '''
    Preprocess image to get the edges using canny filter.
    img -> img to filter.
    sigma -> value to get best parameters in canny filter.
    edges -> already filtered image of edges.
    '''
    
    img = cv2.bilateralFilter(img, 10, 80, 80)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Obtain lower and upper values for Canny Filter.
    v = np.median(img)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edges = cv2.Canny(img, lower, upper)
    # Apply threshold to canny image. Image result only values 0 or 255.
    _, edges = cv2.threshold(edges, 1, 255, cv2.THRESH_BINARY)
    return edges


def get_pixels_to_match(img, N=1):
    '''
    Given an already filtered image (grayscale or binary) obtain the pixels different from 0.
    Randomly filter indices and return it depending on percentage N.
    img     -> img (grayscale or binary) to select points to match.
    N       -> percentage of all points selected. (0-1)
    selected_points -> return selected points after apply randomly selection using percentage (N).
    '''
    
    if N>1 and N<=0:
        N = 1
    
    # Get y and x coordinates of pixel different to 0.
    y, x = np.where(img != 0)
    all_points = np.column_stack((y,x))
    dim_all_points = all_points.shape
    
    # We keep the percentage (N) of all the points. 
    N_points = int(dim_all_points[0]*N)
    
    # Randomly take points.
    index = np.arange(0, dim_all_points[0], dtype=int)
    selected_idx = np.sort(np.random.choice(index, N_points, replace=False))
    selected_points = np.take(all_points, selected_idx, axis=0)
    
    if debug:
        print("------------------- GET PIXELS TO MATCH ---------------------")
        print(dim_all_points)
        print(selected_points.shape)
    
    return selected_points

def get_ray_backproject(cam, point):
    '''
    Get a back project ray given a point a center og the camera.
    cam     -> cam to get the position of camera. ('left' or  'right')
    point   -> point 2D (pixel) of image.
    return (pos_cam-point),(pos_cam) -> result in homogeneous.
    '''
    
    y, x = point
    pos_cam = HAL.getCameraPosition(cam)
    px_cam = HAL.graficToOptical(cam, [x, y, 1])
    px_cam = HAL.backproject(cam, px_cam)
    
    if debug:
        print("------------------- GET RAY BACKPROJECT --------------------")
        print(np.append(px_cam[:3] - pos_cam, [1]))
        print(np.append(pos_cam, [1]))
    
    return np.append(px_cam[:3] - pos_cam, [1]), np.append(pos_cam, [1])
    

def get_epipolar(cam, ray, imgSize, ksize=9):
    '''
    Get a projection of ray to other camera. 
    cam     -> cam to get the position of camera. ('left' or  'right')
    ray     -> ray to project in the other camera. 
    return p0, p1 -> point in 2D of start and end of epipolar line. 
    '''
    
    vd0 = ray[0] + ray[1]
    vd0_projected = HAL.project(cam, vd0)

    vd1 = (10 * ray[0]) + ray[1]
    vd1_projected = HAL.project(cam, vd1)
    
    # Convert projections to image coordinates.
    p0 = HAL.opticalToGrafic(cam, vd0_projected)
    p1 = HAL.opticalToGrafic(cam, vd1_projected)
    vect = p1 - p0
    
    end_rect_p0 = (vect[1] * (0 - p0[0]) / vect[0]) + p0[1]
    end_rect_p1 = (vect[1] * (imgSize[1] - p0[0]) / vect[0]) + p0[1]

    #p = (y,x) -> To draw we need to flip. (x,y)
    p0 = np.flip(np.array([0, end_rect_p0]).astype(np.int))
    p1 = np.flip(np.array([imgSize[1], end_rect_p1]).astype(np.int))

    if debug:
        print("------------------- FIND EPIPOLAR ---------------------------")
        print(vd0, vd1)
        print(vd0_projected, vd1_projected)
        print(end_rect_p0, end_rect_p1)
        print(p0, p1)

    return p0, p1


def find_best_similar(point, imgLH, imgRH, epi_line, ksize=9):
    '''
    Find point homologous to point recieved in epipolar line. 
    point       -> point in left camera to use as template.
    imgLH       -> image of LH camera in HSV.
    imgRH       -> image of RH camera in HSV.
    epi_line    -> epipolar  line to search the homologous point.
    return match_point, coeff -> point in 2D of best match. coeff -> result of matching algorithm. Close to 1 = very similar. 
    '''
    
    pad = floor(ksize / 2)
    y, x = point
    p0, p1 = epi_line
    imgSize = imgLH.shape
    margen = 2
    window_LH = imgLH[y-pad:y+pad+1, x-pad:x+pad+1, :]
    
    # Create mask with only epipolar line in 1.
    mask = np.zeros(imgSize, dtype=np.uint8)
    cv2.line(mask, (p0[1], p0[0]), (p1[1], p1[0]), (1,1,1), ksize)
    # Multiple mask with imgRH.
    img_to_match = cv2.multiply(imgRH, mask)
    
    # If p0 == p1 -> straight line in Y axis. This is usually the case when the camera pair is canonical.
    if p0[0] == p1[0]:
        min_y = max(p0[0]-pad-margen, 0)
        max_y = min(p0[0]+pad+margen, imgSize[0])
    else: # When is different we have a line with slope.
        min_y = max(min(p0[0]-pad-margen, p1[0]-pad-margen), 0)
        max_y = min(max(p0[0]+pad+margen, p1[0]+pad+margen), imgSize[0])
    
    # As we know the size of the line in the image, we will crop it to speed up the calculation.
    crop_img = img_to_match[min_y:max_y+1, :, :]
    res = cv2.matchTemplate(crop_img, window_LH, cv2.TM_CCOEFF_NORMED)
    _, coeff, _, maxLoc = cv2.minMaxLoc(res)
    maxLoc = np.array(maxLoc)
    match_point = np.flip(maxLoc) + pad
    match_point[0] = match_point[0] + min_y
    
    if debug:
        print("----------------- FIND SIMILAR ---------------------------")
        print(match_point)
        print(coeff)
    
    return match_point, coeff

def compute_3Dpoint(point, ray_LH):
    '''
    Compute 3D (triangulation) using least-squares.
    point       -> point (match) in right camera to triangulate.
    ray_LH      -> rayLH to use in the triangulation.
    return pt3D -> point in 3D in world coordinates.
    '''

    #Compute back-projection right ray
    ray_RH = get_ray_backproject('right', point)
    
    #Create the system Ax=b for least-square solve.
    n = np.cross(ray_LH[0][:3], ray_RH[0][:3])
    A = np.array([ray_LH[0][:3], n, -ray_RH[0][:3]]).T
    b = HAL.getCameraPosition('right') - HAL.getCameraPosition('left')
    
    # Solve system equation Ax = b.
    alpha, beta, _ = np.linalg.lstsq(A, b, rcond=None)[0]

    #Compute the 3D points using least-square solutions
    pt3D = HAL.getCameraPosition('left') + (alpha * ray_LH[0][:3]) + ((beta / 2) * n)
    #pt3D = (alpha * ray_LH[0][:3]) + ((beta / 2) * n)

    return pt3D


################################################################################
# VARIABLES
N = 0.7             # Percentage of points to project.
debug = False       # Show results of all steps in algorithm.
verbose = True      # Show information of actual point. 
only_one = True     # Only execute algorithm one time.
kernel_size = 21    # Kernel size used in find point homologous.

GUI.ClearAllPoints()

while True:
    if only_one:
        imageLH = HAL.getImage('left')
        imageRH = HAL.getImage('right')
        dim = imageLH.shape
        
        GUI.showImages(imageLH, imageRH, True)
        
        edgesLH = get_edges(imageLH)
        
        hsvLH = cv2.cvtColor(imageLH, cv2.COLOR_BGR2HSV)
        hsvRH = cv2.cvtColor(imageRH, cv2.COLOR_BGR2HSV)
        
        points2D = get_pixels_to_match(edgesLH, N)
        total_points2D = len(points2D)
        points3D_world = []
    
        for i in range(total_points2D):
        
            if verbose: print("Point: " + str(i + 1) + " of " + str(total_points2D))
            
            point = points2D[i]
            ray_LH = get_ray_backproject('left', point)
            epipolar_line = get_epipolar('right', ray_LH, dim, kernel_size)
            match, coeff = find_best_similar(point, hsvLH, hsvRH, epipolar_line, kernel_size)
            
            if coeff > 0.8:
                
                point3D = compute_3Dpoint(match, ray_LH)
                
                color = (imageLH[point[0], point[1]] + imageRH[match[0], match[1]]) // 2
                color = color.tolist()[::-1]
                draw_point3D = [point3D + color]
                GUI.ShowNewPoints(draw_point3D)
                
                points3D_world.append(draw_point3D)
                
                if verbose: print("2D: " + str(point) + " / 3D: " + str(np.round_(point3D,2)))
            
            else:
                if verbose: print("2D: " + str(point) + " / 3D: " + str([None, None, None]))

        
        GUI.ShowAllPoints(points3D_world)
        only_one = False
        print("FIN")
