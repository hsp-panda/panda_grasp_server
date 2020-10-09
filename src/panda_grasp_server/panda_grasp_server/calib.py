import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

theta=np.random.random_sample((1, 100))*2*np.pi
phi=np.reshapenp.random.random_sample((1, 100))*np.pi/2
rad=0.5

center = [0.5, 0, 0]

x = rad*np.cos(theta)*np.sin(phi) + center[0]

y = rad*np.sin(theta)*np.sin(phi) + center[1]

z = rad*np.cos(phi) + center[2]

# find the x axis by looking at center of sphere
# find the y axis by finding the derivative of x and y wrt theta
# find the z axis by outer product 

# rosrun tf static_transform_publisher 0.061251 -0.0397 -0.592961 0.002414 -1.6 3.1249 /panda_EE /camera_link 50

points = np.hstack((x,y,z))

x_ax = np.reshape(center, (1,3)) - points

#y_ax = 

#z_ax = 

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x,y,z)

plt.show()




