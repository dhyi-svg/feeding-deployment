
from feeding_deployment.ros2_utils import node_handle
from feeding_deployment.ros2_utils import rospy_compat
from geometry_msgs.msg import WrenchStamped
import numpy as np

def ft_callback(msg):

    ft_reading = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
    down_torque = ft_reading[3]
    # mag = np.linalg.norm(ft_reading)
    # print(f"FT reading: {ft_reading}, magnitude: {mag}")
    if np.abs(down_torque) > 0.05:
        print("Bite detected with down torque: ", down_torque)

if __name__ == '__main__':
    node_handle.init_node('check_ft_readings')

    np.set_printoptions(precision=2, suppress=True)
    ft_sensor_sub = node_handle.get_node().create_subscription(WrenchStamped, '/forque/forqueSensor', ft_callback, 10)
    rospy_compat.spin()