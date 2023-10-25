import rclpy

import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped


from rclpy.node import Node

from tf_transformations import quaternion_multiply
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster

from gazebo_msgs.srv import GetEntityState



class FrameListener(Node):


    def __init__(self):

        super().__init__('jackal_tf2_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
        'target_frame', 'base_link').get_parameter_value().string_value


        # Declare transforms
        self.odom_H_baselink = TransformStamped()
        self.map_H_baselink = TransformStamped()

        self.map_H_baselink.header.frame_id = "map"
        self.map_H_baselink.child_frame_id = "baselink"

        self.odom_H_baselink.header.frame_id = "odom"
        self.odom_H_baselink.child_frame_id = "base_link"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_entity service not available, waiting again...')


        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)

    

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

        # Read message content and assign it to
        # corresponding tf variables

        t2.header.stamp = self.get_clock().now().to_msg()
        tmp_frame = t.header.frame_id
        t2.header.frame_id = t.child_frame_id
        t2.child_frame_id = t.header.frame_id

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t2.transform.translation.x = -t.transform.translation.x
        t2.transform.translation.y = -t.transform.translation.y
        t2.transform.translation.z = -t.transform.translation.z


        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
    
        t2.transform.rotation.x = t.transform.rotation.x
        t2.transform.rotation.y = t.transform.rotation.y
        t2.transform.rotation.z = t.transform.rotation.z
        t2.transform.rotation.w = -t.transform.rotation.w

        return t2

    
    def get_map_H_baselink(self):
        self.req = GetEntityState.Request()
        self.req.name = "jackal"
        self.req.reference_frame= "world"   
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        self.map_H_baselink.transform.translation.x = self.future.result().state.pose.position.x
        self.map_H_baselink.transform.translation.y = self.future.result().state.pose.position.y
        self.map_H_baselink.transform.translation.z = self.future.result().state.pose.position.z
        self.map_H_baselink.transform.rotation = self.future.result().state.pose.orientation

    def get_odom_H_baselink(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'odom'

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            self.odom_H_baselink = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return



    def on_timer(self):
        # self.get_map_H_baselink()
        self.get_odom_H_baselink()

        baselink_H_odom = self.transform_inverse(self.odom_H_baselink)

        map_H_odom = self.concatenate_transforms(self.map_H_baselink, baselink_H_odom)

        # Send the transformation
        self.tf_broadcaster.sendTransform(map_H_odom)



def main():

    rclpy.init()

    node = FrameListener()

    try:
        while rclpy.ok():            
            node.get_map_H_baselink()

            rclpy.spin_once(node)
            
    except KeyboardInterrupt:
        pass


    rclpy.shutdown()