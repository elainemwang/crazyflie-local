from multiprocessing import Process,Queue,Pipe
from arenaxr_rtpos import f
import time

if __name__ == '__main__':
    parent_conn,child_conn = Pipe()
    p = Process(target=f, args=(parent_conn,))
    p.start()

    pos_file = open("textfiles/up_down.txt", "r")
    ypr_file = open("textfiles/my_ypr_file.txt", "r")
    
    # i = 0
    while 1:
        pos_line = pos_file.readline()
        ypr_line = ypr_file.readline()
        if not pos_line or not ypr_line:
            break
        # print(ypr_line.split(" "))
        pos_x,pos_y,pos_z = pos_line.split(" ")
        ypr_y,ypr_p,ypr_r,_ = ypr_line.split(" ")
        child_conn.send(("pos",float(pos_x),float(pos_y),float(pos_z)))
        child_conn.send(("stab",float(ypr_y),float(ypr_p),float(ypr_r)))
        # child_conn.send((i,i,i))
        time.sleep(0.1)
        # i+=1
