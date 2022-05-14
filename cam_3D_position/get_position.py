import os
import cv2
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
from pupil_apriltags import Detector
import argparse


def parse_arguments():
    '''
    use argparse to get aguments.
    '''
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--camera_params", type=str, required=True, help="Path to file with camera calibration parameters (NPY).")
    ap.add_argument("-s", "--showResult", default=False, action='store_true', help= "If flag is sent, the display of detection is activated.")
    ap.add_argument("-p", "--plot3D", default=False, action='store_true', help= "If flag is sent, the display of 3D space with camera position is activated.")
    ap.add_argument("-id", "--camera_id", type=str, default="0", help="Camera id (0, 1, ..., N) or url to IP camera (http://192.168.100.74:4747/video) or path to video.")
    return vars(ap.parse_args())

def get_tag_3Dposition(id):
    '''
    Get 3D position of tag depeding on the id tag.
    '''
    if id == "0":
        return np.array([[485, 0, 0], [685, 0, 0], [685, 200, 0], [485, 200, 0]], dtype=np.float64).reshape(-1, 3)
    elif id == "2":
        return np.array([[0, 0, 0], [200, 0, 0], [200, 200, 0], [0, 200, 0]], dtype=np.float64).reshape(-1, 3)
    elif id == "3":
        return np.array([[-550, 200, 120], [-750, 200, 120], [-750, 0, 120], [-550, 0, 120]], dtype=np.float64).reshape(-1, 3)
    else:
        return -1

def load_camera_parameters(path):
    '''
    Load camera parameters using .npy file.
    '''
    K = None
    d = None
    print(K, d)
    with open(path, 'rb') as f:
        K = np.load(f)
        d = np.load(f)
        #rvecs = np.load(f)
        #tvecs = np.load(f)
    return K, d

def resize_img(img, scale_percent=0.5):
    '''
    resize image using scale_percent.
    '''
    width = int(img.shape[1] * scale_percent)
    height = int(img.shape[0] * scale_percent)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    return resized


def main(parameters):
    # Read parameters and declare variables.
    K = parameters[0]
    fx = K[0][0]
    fy = K[1][1]
    cx = K[0][2]
    cy = K[1][2]
    distorsion = parameters[1]
    id_cam = parameters[2]
    showResult = parameters[3]
    plot3D = parameters[4]
    num_frame = 0
    Xpoints = []
    Ypoints = []
    Zpoints = []
    Npoints = 15
    dist = 50

    # Define a video capture object
    vid = cv2.VideoCapture(id_cam)

    # Define Tag Detector object (april-tag)
    at_detector = Detector(families='tag36h11', nthreads=1, quad_decimate=2.0, quad_sigma=0.0, refine_edges=1,
                           decode_sharpening=0.25, debug=0)

    # Create figure to 3D plot
    if plot3D:
        plt.figure()
        axes = plt.axes(projection='3d')


    # Start reading frames from camera.
    while (vid.isOpened):
        rvecs = []
        tvecs = []
        cam_centers = []
        frame_ok, img = vid.read()
        if frame_ok:
            draw = np.copy(img)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Tag Detection
            Detection = at_detector.detect(gray, estimate_tag_pose=False)

            if len(Detection) != 0:
                print("----------------------------------------------")
                print("Frame: " + str(num_frame) + " // Tag Detected.")

                if plot3D: axes.clear()

                for det in Detection:
                    print("Tag ID: ", det.tag_id)
                    objectPoint3D = get_tag_3Dposition(str(det.tag_id)) # Fijamos la deteccion al centro de referencia. Solo 1 baliza.
                    _, rvec, tvec, _ = cv2.solvePnPRansac(objectPoint3D, det.corners, K, distorsion)
                    rvec, tvec = cv2.solvePnPRefineLM(objectPoint3D, det.corners, K, distorsion, rvec, tvec)
                    R = cv2.Rodrigues(rvec)[0]
                    # Get position of camera center is world coordinates:
                    cameraCenter_esc = -np.linalg.inv(R) @ tvec
                    cam_centers.append(cameraCenter_esc)

                    if showResult:
                        pts = np.array(det.corners, np.int32).reshape((-1, 1, 2))
                        cent = np.array(det.center, np.int32)
                        draw = cv2.polylines(draw, [pts], True, (255, 0, 255), 3)
                        draw = cv2.circle(draw, cent, 1, (0, 0, 255), 8)

                        # project 3D axis reference points to image plane.
                        pt_cent = objectPoint3D[0] # Central point of plot
                        axis = np.array([[pt_cent[0], pt_cent[1], pt_cent[2]],
                                         [pt_cent[0]+dist, pt_cent[1], pt_cent[2]],
                                         [pt_cent[0], pt_cent[1]+dist, pt_cent[2]],
                                         [pt_cent[0], pt_cent[1], pt_cent[2]+dist]], dtype=np.float64).reshape(-1, 3)

                        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, K, distorsion)
                        img_axis_ref = imgpts.astype(int)
                        draw = cv2.line(draw, tuple(img_axis_ref[0].ravel()), tuple(img_axis_ref[1].ravel()), (255, 0, 0), 5) # X img coordinates
                        draw = cv2.line(draw, tuple(img_axis_ref[0].ravel()), tuple(img_axis_ref[2].ravel()), (0, 255, 0), 5) # Y img coordinates
                        draw = cv2.line(draw, tuple(img_axis_ref[0].ravel()), tuple(img_axis_ref[3].ravel()), (0, 0, 255), 5) # Z
                        draw = cv2.putText(draw, " DETECTED", (50, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 255, 0), 2, 1)

                    if plot3D:
                        # Get position of camera center:
                        pt = objectPoint3D[0]  # Punto de referencia de la baliza (0,0)
                        # Se pintan los ejes del sistema de referencia del plano
                        axes.scatter3D(pt[0], pt[1], pt[2], color="yellow", edgecolor="black", s=20)
                        axes.plot3D((pt[0], pt[0]+dist), (pt[1], pt[1]), (pt[2], pt[2]), '-b', linewidth=2)
                        axes.plot3D((pt[0], pt[0]), (pt[1], pt[1]+dist), (pt[2], pt[2]), '-g', linewidth=2)
                        axes.plot3D((pt[0], pt[0]), (pt[1], pt[1]), (pt[2], pt[2]+dist), '-r', linewidth=2)

                if plot3D:
                    if len(Xpoints) >= Npoints:
                        Xpoints.pop(0)
                        Ypoints.pop(0)
                        Zpoints.pop(0)

                    cam_pt = np.mean(np.array(cam_centers),axis=0)
                    margin = 500
                    #Detect if new camera position is far from last position -> OUTLIER.
                    if len(Xpoints)>1 and (cam_pt[0]>Xpoints[-1]+margin or cam_pt[0]<Xpoints[-1]-margin \
                        or cam_pt[1]>Ypoints[-1]+margin or cam_pt[1]<Ypoints[-1]-margin \
                        or cam_pt[2]>Zpoints[-1]+margin or cam_pt[2]<Zpoints[-1]-margin):
                        print("OUTLIER CAMERA POSITION. ", [cam_pt])
                    else:
                        print("MEAN CAMERA POSITION: ", list(cam_pt.ravel()))
                        Xpoints.append(cam_pt[0])
                        Ypoints.append(cam_pt[1])
                        Zpoints.append(cam_pt[2])
                    axes.scatter3D(Xpoints[:Npoints], Ypoints[:Npoints], Zpoints[:Npoints], cmap="Greens", s=15)
                    axes.scatter3D(Xpoints[-1], Ypoints[-1], Zpoints[-1], color="magenta", edgecolor="black", s=20)

            else:
                print("----------------------------------------------")
                print("Frame: " + str(num_frame) + " // Tag NOT detected.")
                draw = cv2.putText(draw, " NOT DETECTED", (50,100), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 2, 1)

            if showResult:
                draw = resize_img(draw, 0.5)
                cv2.imshow('frame', draw)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if plot3D:
                # Importante para que los ejes 3D tengan las mismas proporciones en matplotlib.
                scaling = np.array([getattr(axes, 'get_{}lim'.format(dim))() for dim in 'xyz'])
                axes.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]] * 3)
                axes.set_xlabel('X (mm)')
                axes.set_ylabel('Y (mm)')
                axes.set_zlabel('Z (mm)')
                plt.draw()
                plt.pause(0.001)
        else:
            break
        num_frame += 1

    vid.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    args = parse_arguments()
    path_K = args['camera_params']
    cam_id = args['camera_id']

    if cam_id == "0": cam_id = 0

    if os.path.isfile(path_K):
        K, distorsion = load_camera_parameters(path_K)
        if K is not None:
            print("Camera parameters (K) loaded.")
            params = [K, distorsion, cam_id, args['showResult'], args['plot3D']]
            print(params)
            print("Input parameters loaded. Starting detection.")
            main(params)
        else:
            print("K is None. Error loading camera parameters!")
    else:
        print("Intrinsic camera parameters (K) not found.")


# 'http://192.168.100.74:4747/mjpegfeed'
# vid = cv2.VideoCapture('http://192.168.100.74:4747/video') #Small (480, 640, 3)
# vid = cv2.VideoCapture('http://192.168.100.74:8080/video') #Full size (2160, 3840,3)
# Plot 4 corners points with diferent color:
# draw = cv2.circle(draw, pts[0].ravel(), 1, (255, 0, 0), 5)
# draw = cv2.circle(draw, pts[1].ravel(), 1, (0, 255, ), 5)
# draw = cv2.circle(draw, pts[2].ravel(), 1, (0, 0, 255), 5)
# draw = cv2.circle(draw, pts[3].ravel(), 1, (0, 255, 255), 5)
# axis = np.array([[0, 0, 0],
#                  [dist, 0, 0],
#                  [0, dist,0],
#                  [0,0, dist]], dtype=np.float64)
# axes.scatter3D(0, 0, 0, color="yellow", edgecolor="black", s=20)
# axes.plot3D((0, dist), (0, 0), (0, 0), '-b', linewidth=2)
# axes.plot3D((0, 0), (0, dist), (0, 0), '-g', linewidth=2)
# axes.plot3D((0, 0), (0, 0), (0, dist), '-r', linewidth=2)
# Se pinta la posicion 3D de la camara.