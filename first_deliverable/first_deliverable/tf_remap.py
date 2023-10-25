
import rclpy
from rclpy.node import Node

import math
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped



from tf_transformations import quaternion_multiply
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


class FrameRepublisher(Node):


    def __init__(self):

        super().__init__('jackal_aruco_remapper')

        # Initialize the TF objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to get Aruco pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.aruco_callback,
            100)
        self.subscription  # prevent unused variable warning

        # Instantiate TF with static header variable
        self.cam_H_aruco = TransformStamped() 
        self.cam_H_aruco.header.frame_id = "front_camera_optical"
        self.cam_H_aruco.child_frame_id = "aruco_marker_frame"
        
    # Returns the concatenation of two TransformStamped messages
    def concatenate_transforms(self, transform1, transform2):
        # Combine the headers
        combined_transform = TransformStamped()
        combined_transform.header.stamp = self.get_clock().now().to_msg()
        combined_transform.header.frame_id = transform1.header.frame_id
        combined_transform.child_frame_id = transform2.child_frame_id

        # Concatenate translation
        combined_transform.transform.translation.x = transform1.transform.translation.x + transform2.transform.translation.x
        combined_transform.transform.translation.y = transform1.transform.translation.y + transform2.transform.translation.y
        combined_transform.transform.translation.z = transform1.transform.translation.z + transform2.transform.translation.z

        # Concatenate rotation (quaternion multiplication)
        concatQ = quaternion_multiply(
            [transform1.transform.rotation.x, transform1.transform.rotation.y, transform1.transform.rotation.z, transform1.transform.rotation.w],
            [transform2.transform.rotation.x, transform2.transform.rotation.y, transform2.transform.rotation.z, transform2.transform.rotation.w]
        )
        combined_transform.transform.rotation.x = concatQ[0]
        combined_transform.transform.rotation.y = concatQ[1]
        combined_transform.transform.rotation.z = concatQ[2]
        combined_transform.transform.rotation.w = concatQ[3]

        return combined_transform
    
    # Returns the inverse of a TransformStamped message
    def transform_inverse(self, t):
        t2 = TransformStamped()

        # Read message content and swap frames
        t2.header.stamp = self.get_clock().now().to_msg()
        tmp_frame = t.header.frame_id
        t2.header.frame_id = t.child_frame_id
        t2.child_frame_id = t.header.frame_id

        # Invert the transform coords
        t2.transform.translation.x = -t.transform.translation.x
        t2.transform.translation.y = -t.transform.translation.y
        t2.transform.translation.z = -t.transform.translation.z


        # To invert quaternion, just change sign of w
        t2.transform.rotation.x = t.transform.rotation.x
        t2.transform.rotation.y = t.transform.rotation.y
        t2.transform.rotation.z = t.transform.rotation.z
        t2.transform.rotation.w = -t.transform.rotation.w

        return t2

    # Returns the transformation between target_frame and from_frame,
    # or returns None otherwise
    def get_from_H_to(self, from_frame_rel, to_frame_rel):
        try:
            # Check TF tree for requested transform
            return self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        
    # Plots the TF of the aruco corresponding to the received PoseStamped
    def aruco_callback(self, msg):
        self.cam_H_aruco.header.stamp = self.get_clock().now().to_msg()
        self.cam_H_aruco.transform.translation.x = msg.pose.position.x
        self.cam_H_aruco.transform.translation.y = msg.pose.position.y
        self.cam_H_aruco.transform.translation.z = msg.pose.position.z
        self.cam_H_aruco.transform.rotation.x = msg.pose.orientation.x
        self.cam_H_aruco.transform.rotation.y = msg.pose.orientation.y
        self.cam_H_aruco.transform.rotation.z = msg.pose.orientation.z
        self.cam_H_aruco.transform.rotation.w = msg.pose.orientation.w
        self.tf_broadcaster.sendTransform(self.cam_H_aruco)
        return


def main():

    rclpy.init()

    node = FrameRepublisher()
    rclpy.spin(node)
    
    rclpy.shutdown()