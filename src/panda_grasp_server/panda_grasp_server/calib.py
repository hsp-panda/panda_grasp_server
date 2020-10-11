import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import Pose

def normalize_rows(mat_to_normalize):
    sum_of_rows = np.sum(mat_to_normalize, axis=1)
    return mat_to_normalize / sum_of_rows[:, np.newaxis]

# Define ranges of theta and phi
theta = np.linspace(np.pi/2, 3/2*np.pi, 10)
phi = np.linspace(0, np.pi/2, 10)
rad = 0.5

# Organize the parameter grid
theta_space = np.empty((0,))
phi_space = np.empty((0,))
for idx in range(phi.size):
    theta_space = np.append(theta_space, theta)
    phi_space = np.append(phi_space, phi[idx]*np.ones_like(theta))

theta_space = np.reshape(theta_space, (1, theta_space.size))
phi_space = np.reshape(phi_space, (1, phi_space.size))

# Define sphere center
center = [2, 0, 0]

# Obtain pose center points
x = rad*np.cos(theta_space)*np.sin(phi_space) + center[0]
y = rad*np.sin(theta_space)*np.sin(phi_space) + center[1]
z = rad*np.cos(phi_space) + center[2]

# find the x axis by looking at center of sphere
# find the y axis by finding the derivative of x and y wrt theta
# find the z axis by cross product

points = np.hstack((np.transpose(x),np.transpose(y),np.transpose(z)))

x_ax = np.reshape(center, (1,3)) - points
x_ax = normalize_rows(x_ax)

y_ax_x = -rad*np.sin(theta_space)*np.sin(phi_space)
y_ax_y = rad*np.cos(theta_space)*np.sin(phi_space)
y_ax_z = np.zeros_like(y_ax_x)

y_ax = np.hstack((np.transpose(y_ax_x),np.transpose(y_ax_y),np.transpose(y_ax_z)))
y_ax = normalize_rows(y_ax)

z_ax = np.cross(x_ax, y_ax)
z_ax = normalize_rows(z_ax)

# Obtain poses as position + quaternion
poses = []
for idx in range(x_ax.shape[0]):
    rot_matrix = np.identity(4)
    rot_matrix[:3, :3] = np.transpose(np.stack((
                                            x_ax[idx,:],
                                            y_ax[idx,:],
                                            z_ax[idx,:]
                                        )))

    pose_orient = quaternion_from_matrix(rot_matrix)
    pose_origin = points[idx, :]
    pose = Pose(
                position = pose_origin,
                orientation = pose_orient)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x,y,z)

plt.show()



