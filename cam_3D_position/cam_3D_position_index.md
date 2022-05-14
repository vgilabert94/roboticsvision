# ROBOTICS VISION 
## [Vicente Gilabert Maño](https://www.linkedin.com/in/vgilabert/)
### Master degree in Computer Vision (URJC)

---

## Exercise 3 - Tag Detection and 3D representation of the camera position.


### 1. Introduction

El objetivo del siguiente trabajo es obtener una representación tridimensional en tiempo real de la posición de la cámara en una escena realizando la detección de balizas para autolocalizarnos.

Se va a utilizar la aplicación Webcam IP, donde obtendremos los frames de nuestro movil. La resuloción de la imágenes obtenidas es de (960,1280) con un frame rate de 10fps.
En estas imagenes se realizará la detección de balizas y el cálculo de la posicion 3D de la camara utilizando la matriz de rotacion (R) y el vector de traslacion (tvec).
Para obtener la R y tvec, se utiliza la funcion solvePnP, con las posiciones 3D conocidas de la baliza y los pixeles de las esquinas.
Los pixeles de las esquinas de las balizas son obtenidos mediante la libreria apriltags, donde nos develve los cuatro puntos (x,y) correspondientes a las esquinas.

Todo el proceso anterior, es dependiente de la calibracion de la cámara. Debemos de obtener una calibracion correcta para obtener la matriz de intrinsecos y la posible distorsion de la lente.

---
### 2. How to use?

Para ejecutar el programa se describen a continuacion los archivos y las posibles configuraciones:


* Se ha utlizado un repositorio en python con bindings a la libreria apriltags (C). Solo funciona con Python 3.6 y 3.7. 
  ````
  python -m pip install --upgrade pip
  pip install pupil-apriltags 
  ````

* Para realizar la calibración de la cámara se ha implentado el script camera_calibration.py:
  ```
  python camera_calibration.py --folder images_Calibration/ --chess_shape (9,6) --chess_size 24
  ```

  Los parámetros para script son los siguientes: 
  
  |     |                 | Description                                                                     |
  |:---:|:---------------:|:-------------------------------------------------------------------------------:|
  | -f  | --folder        | Path to folder with images to calibrate camera.                                 |
  | -sh | --chess_shape   | Shape. Number of rows and columns to detect in chessboard.  Default=(9,6)       |
  | -sz | --chess_size    | Size of every rectangle of chessboards (mm).  Default=24                        |
  | -sv | --path_save     | Path to save calibration parameters in file .npy.  Default=results_calibration/ |
  | -p  | --plot_cameras  | Activate flag to 3D plot of all cameras positions.                              |


* Para ejecutar la aplicación principal para la detección de las balizas y el calculo de la posición es get_position.py:
  ```
  python .\get_position.py -c results_calibration/parameters_20220514_123408.npy -id http://192.168.100.74:4747/video -s -p
  ```
  
  Los parámetros para script son los siguientes:
  
  |     |       flag      |                                                 Description                                                 |
  |:---:|:---------------:|:-----------------------------------------------------------------------------------------------------------:|
  | -c  | --camera_params |                           Path to file with camera calibration parameters (NPY).                            |
  | -s  | --showResult    |                    If flag is sent, the display of detection is activated. Default=False                    |
  | -p  | --plot3D        |          If flag is sent, the display of 3D space with camera position is activated. Default=False          |
  | -id | --camera_id     | Camera id (0, 1, ..., N) or url to IP camera (http://192.168.100.74:4747/video) or path to video. Default=0 |
  


---
### 4. Camera calibration

Para el objetivo de este trabajo es necesario realizar una calibracion de la camara para obtener los parametros intrinsecos (K) de la camara.

Se ha utilizado una imagen de ejedrez de un tamaño (9,6) esquinas y un tamaño de cuadrado de 24mm. El total de imagenes utilizadas para  la calibracion es de 48. A continuacion se pueden ver algunas de estas imagenes:

<p align="center">
	<img src="images/imgs_calib.png" alt="imgs_calib" width="80%"/>
</p>

Con el script explicado en punto anterior, se ha realizado nuestra calibracion. El proceso implementado ha sido:

1. Cargar las imagenes para la calibracion.
2. Para cada imagen realizar una deteccion de esquinas. $\rightarrow$ ```cv2.findChessboardCorners```
3. Refinar el resultado obtenido para aumentar la precision. $\rightarrow$ ```cv2.cornerSubPix```
4. Definir los puntos 3D de cada esquima del patron de ajedrez. $\rightarrow$ ```get_chessboard_points((9,6), 24, 24)```
5. Con los puntos 3D definidos y los puntos 2D encontrados en cada imagen, se realiza la calibracion de la camara. $\rightarrow$ ```cv2.calibrateCamera```
6. Guardar resultados de la calibracion a un archivo .npy y representacion del resultado.


A continuacion se muestra la posicion de la camara para cada imagen que hemos utlizado en la calibracion:

<p align="center">
	<img src="images/calib_pos.png" alt="pos_calib" width="80%"/>
</p>



---
### 5. Tag detector


---
### 6. Obtain camera position


---
### 8. Real-time 3D representation.


---
### 7. Results and conclusions.

