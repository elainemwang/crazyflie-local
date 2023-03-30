from arena import *
import time
import numpy as np

# setup library
scene = Scene(host="mqtt.arenaxr.org", scene="crazyflie", namespace="emwang2")
# make a box
box = Box(object_id="my_box", position=Position(0,4,-2), scale=Scale(2,0.3,4))

def main():
    # add the box
    scene.add_object(box)

    scene.update_object(box, position=Position(2,10,-2))
    # box1 = Box(object_id="my_box1", position=Position(0,4,-2), scale=Scale(2,2,2))
    # scene.add_object(box1)

    f = open("textfiles/position.txt", "r")
    coord = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
    for line in f:
        x,y,z = line.split(" ")
        add = np.array([[float(x), float(y), float(z)]])
        coord = np.concatenate((coord, add))
    
    # i = 0
    # while(i < len(coord)):
    #     #print(i)
    #     time.sleep(0.05)
    #     xc = coord[i,0]*10 
    #     yc = coord[i,1]*10 #0.5 is the size of object
    #     zc = coord[i,2]*10
    #     #print(xc, " ", yc, " ", zc)
    #     scene.update_object(box, position=Position(xc,zc,yc))
    #     i += 7 #7 is a little smoother/faster

# add and start tasks
scene.run_once(main)
scene.run_tasks()