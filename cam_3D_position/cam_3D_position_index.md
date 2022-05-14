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


* Se ha utlizado un repositorio en python con bindings a la libreria apriltags (C). Solo funciona con Python 3.6 y 3.7. Instalamos todas las librerias necesiaras que vienen en el archivo de requerimeintos.
  ````
  python -m pip install --upgrade pip
  pip install -r requirements.txt
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

> python camera_calibration.py --folder images_Calibration/ --chess_shape (9,6) --chess_size 24

Se ha utilizado una imagen de ejedrez de un tamaño (9,6) esquinas y un tamaño de cuadrado de 24mm. El total de imagenes utilizadas para  la calibracion es de 48. A continuacion se pueden ver algunas de estas imagenes:

<p align="center">
	<img src="images/imgs_calib.png" alt="imgs_calib" width="80%"/>
</p>

Con el script explicado en punto anterior, se ha realizado nuestra calibracion. El proceso implementado ha sido:

1. Cargar las imagenes para la calibracion.
2. Para cada imagen realizar una deteccion de esquinas. ```cv2.findChessboardCorners```
3. Refinar el resultado obtenido para aumentar la precision. ```cv2.cornerSubPix```
4. Definir los puntos 3D de cada esquima del patron de ajedrez. ```get_chessboard_points((9,6), 24, 24)```
5. Con los puntos 3D definidos y los puntos 2D encontrados en cada imagen, se realiza la calibracion de la camara. ```cv2.calibrateCamera```
6. Guardar resultados de la calibracion a un archivo .npy y representacion del resultado.


A continuacion se muestra la posicion de la camara para cada imagen que hemos utlizado en la calibracion:

<p align="center">
	<img src="images/calib_pos.png" alt="pos_calib" width="80%"/>
</p>



---
### 5. Tag detector and get camera position.

Una vez tenemos la camara calibrada, ya podemos empezar con la deteccion de las balizas y el cálculo de la posicion de la camara. 

Los pasos seguidos son los siguientes:
1. Cargar parametros intrinsecos de la camara (K y distorsion).
2. Establecer conexion con la camara movil mediante la direccion web. 
3. Convertir la imagen a grayscale.
4. Aplicar el detector de balizas. ```Detection = at_detector.detect(gray, estimate_tag_pose=False)```
5. Para cada Deteccion obtenida (puede detectar mas de una baliza en la imagen) mediante el solvePnP se obtiene rvec y tvec. ```cv2.solvePnPRansac```
6. Se realiza un refinamiento de rvec y tvec mediante un proceso de optimizacion iterativo usando Levenberg-Marquardt. ```cv2.solvePnPRefineLM```
6. Utilizamos ```cv2.Rodrigues``` para convertir rvec en R (matriz). Con la siguiente formula obtenemos el centro opctico de la camara en coordenadas mundo:
<p align="center">
	<img src="images/cam_center.png" alt="pos_calib" width="20%"/>
</p>
7. Se realiza la representacion 3D. Para la representacion 2D se muestran la baliza rodeada y los ejes encontrado mediante una proyeccion desde el espacio 3D al 2D. ```cv2.projectPoints```

El resultado que se puede observar al ejecutar la aplicacion es el siguiente:
<p align="center">
	<img src="images/results.png" alt="youtube" width="80%"/>
</p>

---
### 6. Results and conclusions.

La configuracion de las balizas uttilizada ha sido:

<p align="center">
	<img src="images/config_tags.png" alt="youtube" width="80%"/>
</p>

>  Todas las medidas anteriores estan en milimetros.


Se ha realizado tres videos demostrativos donde se ven ejemplos para 1, 2 y 3 balizas en la deteccionn:

* Una baliza:
	<p align="center">
		<a href="https://www.youtube.com/watch?v=ACiy273RN0g" target="_blank">
		<img src="images/youtube_play.png" alt="youtube" width="20%"/>
		</a>
	</p>

* Dos balizas:
	<p align="center">
		<a href="https://www.youtube.com/watch?v=XjnPv5EeaSE" target="_blank">
		<img src="images/youtube_play.png" alt="youtube" width="20%"/>
		</a>
	</p>

* Dos balizas:
	<p align="center">
		<a href="https://www.youtube.com/watch?v=Z6aoJRJHjtY" target="_blank">
		<img src="images/youtube_play.png" alt="youtube" width="20%"/>
		</a>
	</p>

