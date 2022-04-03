# ROBOTICS VISION 
## [Vicente Gilabert Maño](https://www.linkedin.com/in/vgilabert/)
### Master degree in Computer Vision (URJC)

---

## Exercise 2 - 3D Reconstruction


### 1. Introduction
En este trabajo se va a realizar una reconstrucción 3D utilizando un robot (kobuki) con visión estereoscópica. La reconstrucción 3D es el proceso de determinar el perfil 3D de un objeto, así como conocer la coordenada 3D de cualquier punto del perfil. 

En nuestro caso la reconstrucción se va a realizar basada en software.  Este enfoque se basa en la capacidad de cálculo del ordenador para determinar las propiedades 3D del objeto mediante dos imágenes de la escena obtenidas cámaras en diferente posición. Utilizaremos la geometría epipolar para con las propiedades y restricciones que nos aporta, poder realizar la triangulación de las dos imágenes en un punto y obtener la profundidad del punto. 

A continuación, se muestra una imagen de la escena a reconstruir con los diversos objetos en ella: 

<p align="center">
	<img src="images/introduccion.png" alt="img_ori" width="70%"/>
</p>

La siguiente imagen muestra la vista de cada camara del robot, que es la informacion que tenemos para realizar la reconstruccion 3D.

<p align="center">
	<img src="images/camaras.png" alt="img_ori" width="70%"/>
</p>


### 2. 



### 3. 



### 4. Conclusions


### Prueba


```python
def adjust_vel(error, vel):
    error = abs(error)
    
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
```

<p align="center">
	<img src="images/road_red_line.png" alt="img_ori" width="500"/>
</p>

<p align="center">
	<a href="https://www.youtube.com/watch?v=Bxno-UfDaz0" target="_blank">
	<img src="images/youtube_play.png" alt="youtube" width="30%"/>
	</a>
</p>


> **_NOTE:_** *Python file used in this exercise is PID_CONTROL_SPEED_CASE.py.*