from arena import *
import time
import numpy as np

# want to move the box around while printing "hello" in main loop, arena xr stuff needs to be async

# setup library
scene = Scene(host="mqtt.arenaxr.org", scene="crazyflie", namespace="emwang2")
# make a box
box = Box(object_id="my_box", position=Position(0,4,-2), scale=Scale(2,2,2))

x = 0
c = [(0,0,0),(1,0,0),(2,0,0),(3,0,0),(4,0,0),(5,0,0),(6,0,0),(7,0,0),(8,0,0),(9,0,0)]
@scene.async
def periodic():
    global x    # non allocated variables need to be global
    global c
    scene.update_object(box, position=Position(c[x][0],c[x][2],c[x][1]))
    print(box.data.position)
    x += 1
    if x == 9:
        x = 0