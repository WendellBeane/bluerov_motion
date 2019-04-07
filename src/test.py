from controller import *
import numpy as np
import time

c = Controller()

goal_xyz = np.array([0.03261, 0, 0.6380])
goal_orient = np.array([-2.9737, -0.0463, -0.0282])

c.move_to((goal_xyz, goal_orient))
# array([-0.30020567, -0.2529854 ,  0.86268569]), array([-2.93745714, -0.47285402, -0.13006019])
c.serialize_poses()
