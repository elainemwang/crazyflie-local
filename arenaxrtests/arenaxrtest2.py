from arena import *
import time
import numpy as np

# setup library
scene = Scene(host="mqtt.arenaxr.org", scene="crazyflie", namespace="emwang2")
box = Box(object_id="my_box", position=Position(0,4,-2), scale=Scale(2,2,2))
box2 = Box(object_id="my_box2", position=Position(0,6,-2), scale=Scale(2,2,2))
f = open("up_down.txt", "r")
coord = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
for line in f:
    x,y,z = line.split(" ")
    add = np.array([[float(x), float(y), float(z)]])
    coord = np.concatenate((coord, add))

@scene.run_once
def main():
    scene.add_object(box)

    scene.update_object(box, position=Position(2,4,-2))
    

    text = Text(object_id="my_text", text="Welcome to arena-py!", position=Position(0,2,0), parent=box)
    scene.add_object(text)

    scene.add_object(box2)

    scene.update_object(box2, position=Position(2,6,-2))
    

    text2 = Text(object_id="my_text2", text="Welcome to arena-py!", position=Position(0,4,0), parent=box2)
    scene.add_object(text2)

# x = 0
# @scene.run_async
# async def periodic():
#     global x    # non allocated variables need to be global
#     global coord
#     scene.update_object(box, position=Position(coord[x][0]*10,coord[x][2]*10,coord[x][1]*10))
#     print(box.data.position)
#     await scene.sleep(500)
#     x += 1
#     if x > 1273:
#         x = 0

x = 0
c = [(0,0,0),(1,0,0),(2,0,0),(3,0,0),(4,0,0),(5,0,0),(6,0,0),(7,0,0),(8,0,0),(9,0,0)]
@scene.run_forever(interval_ms=500)
def periodic():
    global x    # non allocated variables need to be global
    global c
    scene.update_object(box, position=Position(c[x][0],c[x][2],c[x][1]))
    print(box.data.position)
    x += 1
    if x == 9:
        x = 0

x1 = 0
c1 = [(0,2,0),(1,2,0),(2,2,0),(3,2,0),(4,2,0),(5,2,0),(6,2,0),(7,2,0),(8,2,0),(9,2,0)]
@scene.run_forever(interval_ms=500)
def periodic2():
    global x1    # non allocated variables need to be global
    global c1
    scene.update_object(box2, position=Position(c1[x1][0],c1[x1][2],c1[x1][1]))
    print(box2.data.position)
    x1 += 1
    if x1 == 9:
        x1 = 0


scene.run_tasks()




