from arena import *
import time
import numpy as np

# setup library
scene = Scene(host="mqtt.arenaxr.org", scene="crazyflie", namespace="emwang2")
# make a box
drone = GLTF(object_id="drone", position=Position(0,0,0), scale=Scale(2,2,2), url="store/models/Drone.glb")

i = 0
@scene.run_forever(interval_ms=100)
def periodic():
    # add the box
    global i
    scene.update_object(drone, position=Position(0,i,0))
    print("drone position: ", drone.data.position)
    time.sleep(0.1)
    i+=1

    
# add and start tasks
scene.run_tasks()