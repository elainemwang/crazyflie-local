from multiprocessing import Process,Pipe
from arena import *
import time
import numpy as np
pc = 0
# setup library
scene = Scene(host="mqtt.arenaxr.org", scene="crazyflie", namespace="emwang2")
# make a box
box = Box(object_id="my_box", position=Position(0,4,-2), scale=Scale(2,2,2))

x,y,z = 0,0,0
@scene.run_forever(interval_ms=50)
def periodic():
    global x, y, z    # non allocated variables need to be global
    global pc
    box.update_attributes(position=Position(x*10,z*10,-y*10))
    scene.update_object(box)
    print("from m11: ", box.data.position)
    x,y,z = pc.recv()

def f(parent_conn):
   global pc
   pc = parent_conn
   scene.run_tasks()


def test(parent_conn):
    while 1:
        print("from m1: ", parent_conn.recv())