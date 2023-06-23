from arena import *
import math

pc = 0
# setup library
scene = Scene(host="mqtt.arenaxr.org", scene="crazyflie", namespace="emwang2")
# make a box
# box = Box(object_id="my_box", position=Position(0,4,-2), scale=Scale(2,0.3,4))
drone = GLTF(object_id="drone", position=Position(0,0,0), scale=Scale(2,2,2), url="store/models/Drone.glb")

x,y,z = 0,0,0
@scene.run_forever(interval_ms=1)
def periodic():
    global x, y, z    # non allocated variables need to be global
    global pc
    stab_or_pos, x,y,z = pc.recv()
    if stab_or_pos == "stab":
        # box.update_attributes(rotation=Rotation(math.radians(-y),math.radians(x),math.radians(z)))
        drone.update_attributes(rotation=Rotation(math.radians(-y),math.radians(x),math.radians(z)))

        # scene.update_object(box)
        scene.update_object(drone)

        print("from m11 rotation: ", drone.data.rotation)

    else:
        # box.update_attributes(position=Position(y*10,z*10,x*10))
        drone.update_attributes(position=Position(y*10 + 10,z*10,x*10))

        # scene.update_object(box)
        scene.update_object(drone)

        print("from m11 position: ", drone.data.position)

def f(parent_conn):
   global pc
   pc = parent_conn
   scene.run_tasks()

def test(parent_conn):
    while 1:
        print("from m1: ", parent_conn.recv())