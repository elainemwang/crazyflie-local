# Helper functions for graphing lines in 3d for the position graphs and the 3d orientation matrix.

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

def plotting():
	# plotting
	map1 = plt.figure(1)
	ax1 = map1.add_subplot(2, 2, 1, projection='3d') # my position plot only
	ax1.set_title('Locally Calculated Position')
	ax1.set_xlabel('Meters')
	ax1.set_ylabel('Meters')
	ax1.set_zlabel('Meters')
	ax2 = map1.add_subplot(2, 2, 2, projection='3d') # their position plot only
	ax2.set_title('Onboard Calculated Position')
	ax2.set_xlabel('Meters')
	ax2.set_ylabel('Meters')
	ax2.set_zlabel('Meters')
	map_ax = map1.add_subplot(2, 2, (3,4), projection='3d') # both our positions
	map_ax.set_title('Both Positions')
	map_ax.set_xlabel('Meters')
	map_ax.set_ylabel('Meters')
	map_ax.set_zlabel('Meters')

	map2 = plt.figure(2)
	orient_pos = map2.add_subplot(111, projection='3d')  # my orientation
	orient_pos.set_title('Local Orientation')
	orient_pos.set_xlabel('Meters')
	orient_pos.set_ylabel('Meters')
	orient_pos.set_zlabel('Meters')

	map3 = plt.figure(3)
	ypr_plot = map3.add_subplot(111) # our ypr positions
	ypr_plot.set_title('Yaw Pitch Roll Measurements')
	ypr_plot.set_xlabel('Timestamp (ms)')
	ypr_plot.set_ylabel('Degrees')

	map_ax.autoscale(enable=True, axis='both', tight=True)
	ax1.autoscale(enable=True, axis='both', tight=True)
	ax2.autoscale(enable=True, axis='both', tight=True)

	# # # Setting the axes properties
	map_ax.set_xlim3d([-0.5, 1])
	map_ax.set_ylim3d([-0.5, 1])
	map_ax.set_zlim3d([-0.5, 1])

	ax1.set_xlim3d([-0.5, 1])
	ax1.set_ylim3d([-0.5, 1])
	ax1.set_zlim3d([-0.5, 1])

	ax2.set_xlim3d([-0.5, 1])
	ax2.set_ylim3d([-0.5, 1])
	ax2.set_zlim3d([-0.5, 1])

	orient_pos.set_xlim3d([-0.5, 1])
	orient_pos.set_ylim3d([-0.5, 1])
	orient_pos.set_zlim3d([-0.5, 1])

	# Setting up the lines
	hl, = map_ax.plot3D([0], [0], [0], label = "Local Position") # for the combined plot
	hb, = map_ax.plot3D([0], [0], [0], label = "On Board Position")

	h1, = ax1.plot3D([0], [0], [0]) # for the individual plots
	h2, = ax2.plot3D([0], [0], [0], color = "orange")

	return h1, h2, hl, hb, ypr_plot, orient_pos

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