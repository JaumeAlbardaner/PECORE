
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to get Aruco pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.aruco_callback,
            100)
        self.subscription  # prevent unused variable warning

        self.cam_H_aruco = TransformStamped() 
        self.cam_H_aruco.header.frame_id = "front_camera_optical"
        self.cam_H_aruco.child_frame_id = "aruco_marker_frame"
        

    def concatenate_transforms(self, transform1, transform2):
        # Combine the transformations
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

    def get_from_H_to(self, from_frame_rel, to_frame_rel):

        # Return the transformation between target_frame and from_frame
        # or return None otherwise
        try:
            return self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        

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

    # def pub_desCam_H_rob(self):
    #     robot_H_cam = self.get_from_H_to('base_link','front_camera')
    #     # cam_H_robot = self.transform_inverse(robot_H_cam)
    #     # map_H_descam = self.get_from_H_to('map','desired_camera')
    #     robot_H_cam.header.frame_id = 'desired_camera'
    #     robot_H_cam.child_frame_id = 'desired_robot'
    #     self.tf_broadcaster.sendTransform(robot_H_cam)
    #     return



def main():

    rclpy.init()

    node = FrameRepublisher()
    # node.pub_desCam_H_rob()
    rclpy.spin(node)

    # try:
    #     while rclpy.ok():            
    #         node.get_map_H_baselink()

    #         rclpy.spin_once(node)
            
    # except KeyboardInterrupt:
    #     pass


    rclpy.shutdown()