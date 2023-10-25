
import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist

import tf_transformations as tf

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class PBVS(Node):


    def __init__(self):

        super().__init__('pbvs')

        # Init TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare publisher of vel comands
        self.publisher_ = self.create_publisher(Twist, '/jackal_velocity_controller/cmd_vel_unstamped', 1000)
        
        # Set update timers for perception and control
        self.timer_perception = self.create_timer(0.1, self.update_pose)
        self.timer_control = self.create_timer(0.5, self.send_command)

        # TF variables to be used later
        self.cam_H_camDesired = None

        # Declare and acquire lambda parameter
        self.lam = self.declare_parameter(
        'lam', 0.01).get_parameter_value().double_value
    


    def update_pose(self):
        #Update relative transform
        try:
            self.cam_H_camDesired = self.tf_buffer.lookup_transform(
                "desired_camera",
                "front_camera",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform "Desired cam position" to current cam position: {ex}')
            return
        

    def send_command(self):
        # Don't give command if aruco was lost or odometry fails
        if (self.cam_H_camDesired == None):
            return
        #Finding error
        rot_diff = np.array([self.cam_H_camDesired.transform.rotation.x,
                        self.cam_H_camDesired.transform.rotation.y,
                        self.cam_H_camDesired.transform.rotation.z,
                        self.cam_H_camDesired.transform.rotation.w])
        rot_diff_mat = tf.quaternion_matrix(rot_diff)

        
        err_pos = np.array([self.cam_H_camDesired.transform.translation.x,
                            self.cam_H_camDesired.transform.translation.y,
                            self.cam_H_camDesired.transform.translation.z])
        err_rpy = tf.euler_from_matrix(rot_diff_mat)
        
        error = np.concatenate((err_pos,err_rpy))


        # Compute Jacobian
        Jp = rot_diff_mat[0:3,0:3]
        
        Jth = 0.5*(np.trace(np.transpose(Jp))*np.identity(3)-Jp)
        
        J = np.zeros((6,6))
        J[0:3,0:3] = Jp
        J[3:6,3:6] = Jth

        # Compute control action
        v = -self.lam*np.matmul(np.linalg.pinv(J),error)

        #Send the command with an adjustment made due to differential drive properties
        msg = Twist()
        msg.linear.x = v[0]
        msg.angular.z = v[5] + math.atan2(v[2],v[1]) # Add rotation bc robot not omnidirectional
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "vx: %f, wz: %f"' % (msg.linear.x, msg.angular.z))





def main():

    rclpy.init()

    node = PBVS()
    rclpy.spin(node)

    rclpy.shutdown()