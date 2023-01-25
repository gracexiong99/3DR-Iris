import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, Vector3, Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from std_msgs.msg import Header
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler

# K = rospy.get_param("~K",  [[1,0,0,0.4515,0,0],
#                             [0,-1,0,0,-0.4515,0],
#                             [0,0,1,0,0,1.6125]])
# _K = np.asarray(K)
# new = np.matmul(_K, [1,1,1,1,1,1])
# print(new)

# K = np.array([[1,0,0,0.4515,0,0],[0,-1,0,0,-0.4515,0],[0,0,1,0,0,1.6125]])
# position_error = [1,2,3]
# input = K@[position_error[0],position_error[1],position_error[2],0,0,0] #K:3x6  e:6x1
# print(input)

temp = quaternion_from_euler(2.16e-3, -1.59e-2, 0)
print(temp)
