import numpy as np
import matplotlib.pyplot as plt
import vectorgen
import pathgen
import geometry_msgs.msg
def conversions(force, x, y):
    p = geometry_msgs.msg.Point()
    p.x = x
    p.y = y
    ftot = force(p)
    return ftot.x, ftot.y
X, Y  = np.meshgrid(np.linspace(-25,25, 50),np.linspace(-25,25,50))
U = np.zeros(X.shape)
V = np.zeros(Y.shape)

for i in range(X.shape[0]):
	for j in range(Y.shape[0]):
            U[i,j], V[i,j] = conversions(vectorgen.netForce, X[i,j], Y[i,j])
start = geometry_msgs.msg.Point()
start.x = 15
start.y = -3
path = pathgen.genpath(start, vectorgen.netForce, 4.0, horizon=100.0, dt=0.01)


start2 = geometry_msgs.msg.Point()
start2.x = -4
start2.y = 15
path2 = pathgen.genpath(start, vectorgen.netForce, 2.0, horizon=100.0, dt=0.1)


start3 = geometry_msgs.msg.Point()
start3.x = -14
start3.y = -8
path3 = pathgen.genpath(start, vectorgen.netForce, 1.0, horizon=100.0, dt=0.1)

plt.quiver(X,Y,U,V, units='xy',scale=1.2, color='black')

for pose in path.poses:
    plt.plot(pose.position.x, pose.position.y, 'b.')
#for pose in path2.poses:
#    plt.plot(pose.position.x, pose.position.y, 'g.')
#for pose in path3.poses:
#    plt.plot(pose.position.x, pose.position.y, 'm.')

plt.title('Paths generated from different starting positions')
plt.xlabel('X Position (Meters)')
plt.ylabel('Y Position (Meters)')
plt.show()
