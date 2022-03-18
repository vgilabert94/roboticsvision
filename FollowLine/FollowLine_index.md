# ROBOTICS VISION 
## Vicente Gilabert Maño
### Master degree in Computer Vision (URJC)

---

## Exercise 1 - Follow red line.



### 1. Introduccion

El objetivo de esta practica es desarrolar un sistema para tratar de controlar un coche sobre una linea roja en la carretera.

<img src="images/road_red_line.png" alt="img_ori" width="500"/>

Hay que tratar de ir el maximo tiempo posible sobre la linea roja y evitar el balanceo descontrolado del coche. Esto se va a poder proobar con diferentes enfoques de  controles PID.



### 2. Controles
Para ello se va a utilizar la plataforma de simulacion robotica [Unibotics](https://unibotics.org/), que nos va a permitir la programacion en Python para el control del robot (coche en nuestro caso).
Esta plataforma nos brinda algunas librerias y metodos para el control del robot. A continuacion se explican:

* __from HAL import HAL__ $\rightarrow$ -> Hardware Abstraction Layer. Contiene funciones para enviar y recibir informacion al hardware del robot (Gazebo).
* __from GUI import GUI__ $\rightarrow$ ->  Graphical User Interface. Contiene las funciones para la visualizacion de imagenes.

* Para recibir la imagen de la camara del robot:
```python
 HAL.getImage()
```
* Para asignar la velocidad al robot:
```python
 HAL.setV()
```

* Para asignar el giro al robot:
```python
 HAL.setW()
```

* Para visualizar las imagenes en una ventana de la plataforma:
```python
 GUI.showImage()
```



### 3. Sistema perceptivo y preprocesado
Los humanos estamos acostumbrado a las imágenes y saber interpretar la escena, detectando los objetos o cualquier detalle. 
En robótica uno de los sensores más utilizados son las cámaras debido a la gran cantidad información que contienen.
Aunque no siempre es una tarea sencilla, ya que tenemos una matriz de números donde hay que extraer la información, por lo que en algunos casos suele ser una tarea difícil.

Para nuestro problema, al ser un caso bastante controlado se va a detectar la línea roja con un filtrado de color en el espacio HSV. También se aplicarán algunas operaciones morfológicas para eliminar posible ruido que nos aparezca en la imagen.
El resultado de esta función para cada *frame*, será un resultado similar al siguiente:
<img src="images/hsv_filter.png" alt="mask" width="500"/>

Una vez tenemos la máscara que contiene la información del color rojo, se han obtenido los contornos de la imagen. De los contornos obtenidos se han filtrado por tamaño, para evitar posibles pequeños contornos que nos afecten y así robustecer el sistema. Utilizando el contorno de mayor área, se ha obtenido el punto más alto, es decir el de menor coordenada y. 
En la imagen siguiente imagen se puede ver en verde el punto más alto (menor y):
<img src="images/centros.png" alt="center" width="500"/>

Los otros dos puntos en azul son los centros de la imagen, es decir nuestros puntos de referencia.



### 4. Control proporcional (P)

Una vez tenemos el preprocesado terminado, donde nos devuelve un punto (el mas alto del contorno) se obtendra la desviacion, donde será:
```math
error = pto_ref - pto_actual
```
error = pto_ref - pto_actual

Donde pto_ref es la coordenada en x del punto central del la imagen y pto_actual es la coordenada x del punto a del *frame* actual.

La primera aproximacion para resolver el problema es el control propocional, donde simplemente tendremos una variable Kp que ajustar de forma experimental para obtener el giro en cada *frame*.
giro = kp * error
HAL.setW(giro)

Con este control estamos modificando el giro de nuestro coche, dependiendo del error obtenido en cada *frame*.
En este caso la velocidad permanece constante durante toda la vuelta al circuito. Los parametros obtenidos para un correcto funcionamiento son:

vel = 2 ; kp_W = 0.005 -> bien, con balanceo.
vel = 3 ; kp_W = 0.005 -> bien, con balanceo.
vel = 4 ; kp_W = 0.005 -> bien, con balanceo.
vel = 5 ; kp_W = 0.001 -> se estrella el coche.
vel = 5 ; kp_W = 0.01 -> conduccion brusca.
vel = 5 ; kp_W = 0.008 -> puede ser el mejor
vel = 6 ; kp_W = 0.008 -> probar?

Se adjunta un video del mejor resultado obtenido para esta primera aproximacion (control P):
Parametros vel = 6 ; kp_W = 0.008

INSERTAR VIDEO


### 5. Control derivativo (PD)
