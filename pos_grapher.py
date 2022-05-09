# Helper functions for graphing lines in 3d for the position graphs and the 3d orientation matrix.

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

# update the position of the graph
def update_line(hl, new_data):
	xdata, ydata, zdata = hl._verts3d
	hl.set_xdata(np.append(xdata, new_data[0]))
	hl.set_ydata(np.append(ydata, new_data[1]))
	hl.set_3d_properties(np.append(zdata, new_data[2]))
	plt.draw()


def graph_orient(ax, pos, orient):
	x,y,z = orient
	# print("------------------")
	# print(pos)
	# print(orient)
	ax.quiver(pos[0], pos[1], pos[2], x[0], x[1], x[2],  color = "blue", length = 0.3)
	ax.quiver(pos[0], pos[1], pos[2], y[0], y[1], y[2], color = "red", length = 0.3)
	ax.quiver(pos[0], pos[1], pos[2], z[0], z[1], z[2], color = "green", length = 0.3)