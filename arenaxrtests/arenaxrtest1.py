from arena import *
import time
import numpy as np

# setup library
scene = Scene(host="mqtt.arenaxr.org", scene="crazyflie", namespace="emwang2")
# make a box
drone = GLTF(object_id="drone", position=Position(0,0,0), scale=Scale(2,2,2), url="store/models/Drone.glb")

def main():
    # add the box
    scene.add_object(drone)

    scene.update_object(drone, position=Position(0,0,0))

    f = open("textfiles/position.txt", "r")
    coord = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
    for line in f:
        x,y,z = line.split(" ")
        add = np.array([[float(x), float(y), float(z)]])
        coord = np.concatenate((coord, add))

# add and start tasks
scene.run_once(main)
scene.run_tasks()