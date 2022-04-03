# ROBOTICS VISION 
## [Vicente Gilabert Maño](https://www.linkedin.com/in/vgilabert/)
### Master degree in Computer Vision (URJC)

---

## Exercise 2 - 3D Reconstruction


### 1. Introduction



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