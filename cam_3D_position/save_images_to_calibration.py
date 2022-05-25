import os
from cv2 import cv2
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
from pupil_apriltags import Detector
import argparse


def resize_img(img, scale_percent=0.5):
    '''
    resize image using scale_percent.
    '''
    width = int(img.shape[1] * scale_percent)
    height = int(img.shape[0] * scale_percent)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    return resized


def main():
    num_frame = 0
    path_save = "images_calibration"

    if not os.path.exists(path_save):
        os.makedirs(path_save)

    vid = cv2.VideoCapture("https://192.168.247.137:8080/video")
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #cv2.VideoWriter_fourcc(*"MJPG")
    out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (1280,960))

    # Start reading frames from camera.
    while (num_frame<150):

        ret, img = vid.read()
        draw = np.copy(img)
        print(img.shape)


        # if num_frame%5 == 0:
        #     name_img = "image" + str(num_frame) + ".png"
        #     save_img = os.path.join(path_save, name_img)
        #     cv2.imwrite(save_img, img)
        #     cv2.putText(draw, "SAVED", (50,150), 0, 4, (0,255,0), 3, 2)

        out.write(img)

        draw = resize_img(draw, 0.5)
        cv2.imshow('frame', draw)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

        print("Frame: " + str(num_frame))
        num_frame += 1

    out.release()
    vid.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()

