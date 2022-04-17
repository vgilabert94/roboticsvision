################################################################################
# ROBOTIC VISION - EXERCISE 1. FOLLOW LINE.
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
import cv2
import numpy as np

################################################################################
#FUNCTIONS

def hsv_filter(img):
    '''
    HSV FILTER. Filter image using HSV space color. 
    IN: img -> image to filter.
    OUT: mask -> image filtered (mask).
    '''
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_mask = cv2.inRange(hsv, lower1, upper1)
    upper_mask = cv2.inRange(hsv, lower2, upper2)
    mask = lower_mask + upper_mask;
    _, mask = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)
    mask = cv2.erode(mask, (5,5))
    mask = cv2.erode(mask, (5,5))
    mask = cv2.dilate(mask, (5,5))
    return mask
    
    
def get_max_contours(img):
    '''
    GET_MAX_CONTOURS. Find contours of image. 
    IN: img -> image to find contours.
    OUT: centerTop -> returns the topmost point of the biggest contour.
    OUT: img -> image with contours drawed.
    '''
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    centerTop = ()

    if len(contours) != 0:
        c = max(contours, key=cv2.contourArea)
        img = cv2.drawContours(img, c, -1, color=(255), thickness=cv2.FILLED)
        if cv2.contourArea(c) > minArea:
            centerTop = tuple(c[c[:, :, 1].argmin()][0])

    return centerTop, img


def draw_info_img(img, point1, point2, error, vel, giro):
    '''
    DRAW_INFO_IMG. Draw points, text, and lines in img. 
    IN: img -> image to draw.
    OUT: draw -> image drawed.
    '''
    x, y = point1
    centerY = int(y/2)
    centerX = int(x/2)
    act_center  = point2
    draw = cv2.circle(img, (centerX, centerY), radius=10, color=(255, 0, 0), thickness=-1)
    draw = cv2.circle(draw, (int(centerX), int(centerY+(centerY/2))), radius=10, color=(255, 0, 0), thickness=-1)
    draw = cv2.circle(draw, act_center, radius=10, color=(0, 255, 0), thickness=-1)
    draw = cv2.line(draw, (centerX, 0), (centerX, y), color=(255,0,0), thickness=2)
    #draw = cv2.line(draw, (centerX, centerY), act_center, color=(255,255,0), thickness=2)
    #draw = cv2.line(draw, (centerX, centerY), (int(centerX), int(centerY+(centerY/2))), color=(255,255,0), thickness=2)
    #draw = cv2.line(draw, act_center, (int(centerX), int(centerY+(centerY/2))), color=(255,255,0), thickness=2)
    #draw = cv2.line(draw, act_center, (centerX, y), color=(255,255,0), thickness=2)
    putError = "Error: " + str(error)
    draw = cv2.putText(draw, putError, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    putVel = "Velocidad: " + str(round(vel, 2))
    draw = cv2.putText(draw, putVel, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    putGiro = "Giro: " + str(round(giro, 2))
    draw = cv2.putText(draw, putGiro, (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    
    return draw


def straight_state(vel):
    '''
    straight_state. The current status is to continue in a straight line.
    IN: vel -> actual speed.
    OUT: new_speed, new_spin.
    '''
    global act_speed
    
    new_speed = vel + vel*0.02
    act_speed = new_speed

    if new_speed > speed_max:
        HAL.setV(speed_max)
        print("Velocidad actual MAX: ", speed_max)
        return speed_max, 0
    else:
        HAL.setV(new_speed)
        print("Velocidad actual: ", new_speed)
        return new_speed, 0


def curve_state(error, vel):
    '''
    curve_state. The current status is to turn the car.
    IN: vel -> actual speed.
    OUT: new_speed, new_spin.
    '''
    global act_speed, act_spin
    
    diff_error = error - error_list[-1]
    sum_error = error + error_list[-1]
    new_spin = (kp_W*error) + (kd_W*diff_error) + (ki_W*sum_error)
    HAL.setW(new_spin)
    print("Giro actual: ", new_spin)
    new_speed = adjust_vel(error, vel)
    HAL.setV(new_speed)
    print("Velocidad actual: ", new_speed)
    act_speed = new_speed
    act_spin = new_spin
    
    return  new_speed, new_spin
    
    
def find_line_state():
    '''
    find_line_state. Line is not detected. The current status is to turn slowly and find a red line.
    OUT: new_speed, new_spin.
    '''
    HAL.setV(0.5)
    HAL.setW(0.5)
    return 0.5, 0.5


def adjust_vel(error, vel):
    '''
    adjust_vel. This function is to adjust the speed depending on the error.
    IN: error, vel. Actual error and speed.
    OUT: new_speed, new_spin.
    '''
    
    # Convertimos a  valor absoluto para que el giro a ambos aldos sea igual.
    error = abs(error)
    
    #Reducir la velocidad dependiendo del error.
    if error > min_th and error < min_th+20:
        new_speed = vel - vel*0.001
    elif error >= min_th+20 and error < min_th+40:
        new_speed = vel - vel*0.002
    elif error >= min_th+40 and error < min_th+60:
        new_speed = vel - vel*0.005
    elif error >= min_th+60:
        new_speed = vel - vel*0.008
    
    if new_speed < speed_min:
        new_speed = speed_min
    
    return new_speed



################################################################################
########################## DECLARACION DE VARIABLES ############################
################################################################################

lower1 = np.array([0, 50, 50])
upper1 = np.array([15, 255, 255])
lower2 = np.array([165, 50, 50])
upper2 = np.array([180, 255, 255])

error_list = []     # Lista para guarda los errores en cada frame.
minArea = 250       # Minimo tamaño de area para considerar un contorno valido.
offset = 10         # Offset del coche, ya que esta desplazado hacia la derecha.
min_th = 20         # Minimo umbral para considerar que estamos en una recta.
first_frame = True  # Para saber cual es el primer frame.

# Constantes para el manejo del coche (giro y velocidad).
act_spin = 0    # Giro actual del coche.
speed_max = 7.5   # Velocidad maxima para el coche.
act_speed = 6   # Velocidad actual del coche.
speed_min = 5.5   # Velocidad minima para el coche.

# Constantes del controlador PID. 
kp_W = 0.009       # Proporcional.
kd_W = 0.0006       # Derivativo.
ki_W = 0.00008       # Integral.


while True:
    
    img = HAL.getImage() #Get img.
    y, x, c = img.shape 
    centerY = int(y/2)
    centerX = int(x/2)
    # 480, 640, 3 -> center x = 320
    
    mask = hsv_filter(img)
    act_center, mask = get_max_contours(mask)

    if len(act_center) != 0:
        act_error = (centerX+offset) - act_center[0]
        #act_error = centerX - act_center[0]
        
        if first_frame:
            first_frame = False
        else:
            if act_error < 0 and abs(act_error) > min_th:
                print("Giro derecha. / Error actual: ", act_error)
                act_speed, act_spin = curve_state(act_error, act_speed)
            
            elif act_error > 0 and act_error > min_th:
                print("Giro izquierda. / Error actual: ", act_error)
                act_speed, act_spin = curve_state(act_error, act_speed)

            else:
                print("Seguimos recto. / Error actual: ", act_error)
                act_speed, act_spin = straight_state(act_speed)
        
        error_list.append(act_error)
        draw = draw_info_img(img, (x, y), act_center, act_error, act_speed, act_spin)
        GUI.showImage(draw)
        
    else:
        print("Linea no detectada. Buscar linea...")
        act_speed, act_spin = find_line_state()
        GUI.showImage(img)