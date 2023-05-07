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
    print("main updating")
    scene.update_object(drone, position=Position(0,0,0))


@scene.run_async
async def f():
    print("here")
    await scene.sleep(5000) # must use scene.sleep or asyncio.sleep. DO NOT use time.sleep!
    print("here, but after 5 seconds")


# add and start tasks
scene.run_async(f)

scene.run_once(main)
scene.run_tasks()
scene.run_async(f)

# scene.run_async()