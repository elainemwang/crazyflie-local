from multiprocessing import Process,Queue,Pipe
from m1 import f
import time

if __name__ == '__main__':
    parent_conn,child_conn = Pipe()
    p = Process(target=f, args=(parent_conn,))
    p.start()

    i = 0
    while 1:
        child_conn.send((i,i,i))
        time.sleep(0.1)
        i+=0.01
