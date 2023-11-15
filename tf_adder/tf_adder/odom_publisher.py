# Required libraries and modules are imported.
import rclpy
import tf2_ros
import numpy as np
#
import ros2_numpy

from tf2_ros import TransformListener, TransformBroadcaster, Buffer

from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry


class OdomPublisher(Node):

    def __init__(self):

    	# Publisher setup
        super().__init__('odom_publisher')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service client setup for querying entity states from Gazebo
        self.client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.publisher_ = self.create_publisher(Twist, '/odometry/ground_truth', 1000)

    def publish_odom(self):

        # Prepare a request to obtain the state of the entity named 'jackal'
        req = GetEntityState.Request()
        req.name = 'jackal'
        req.reference_frame = 'world'
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # When the service responds, the pose of the 'jackal' is obtained
        if (future.done()):
            response = future.result().state.pose

        # Prepare a TransformStamped message for the map -> base_link transform
        map_H_baselink = TransformStamped()
        map_H_baselink.header.stamp = self.get_clock().now().to_msg()
        map_H_baselink.header.frame_id = 'map'
        map_H_baselink.child_frame_id = 'base_link'
        map_H_baselink.transform.translation.x = response.position.x
        map_H_baselink.transform.translation.y = response.position.y
        map_H_baselink.transform.translation.z = response.position.z
        map_H_baselink.transform.rotation = response.orientation

        # [FLAG 1] - map_H_baselink transform is obtained from gazebo
        #self.get_logger().info('[FLAG 1]')

        # Try to get the transform between the odom and base_link frames
        try:
            odom_H_baselink = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Lookup transform failed: {str(e)}")
            return

        # [FLAG 2] - odom_H_baselink transform is obtained
        #self.get_logger().info('[FLAG 2]')

        # Convert the transforms to numpy arrays and then compute the inverse and resulting transforms
        map_H_baselink_mat = ros2_numpy.geometry.transform_to_numpy(map_H_baselink.transform)
        odom_H_baselink_mat = ros2_numpy.geometry.transform_to_numpy(odom_H_baselink.transform)

        # Inverse of the odom_H_baselink to obtain baselink_H_odom
        baselink_H_odom_mat = np.linalg.inv(odom_H_baselink_mat)
        # baselink_H_odom_mat = t.inverse_matrix(odom_H_baselink_mat)

        # Concatenate the two transforms to obtain map_H_odom
        # map_H_odom_mat = np.dot(map_H_baselink_mat,baselink_H_odom_mat)
        map_H_odom_mat = np.dot(map_H_baselink_mat,baselink_H_odom_mat)

        # map_H_odom_mat = t.concatenate_matrices(map_H_baselink_mat,baselink_H_odom_mat)
        #self.get_logger().info('Matrix: %s' % str(map_H_odom_mat))

        # Convert the resulting numpy matrix back to a TransformStamped message
        map_H_odom = TransformStamped()
        map_H_odom.transform = ros2_numpy.geometry.numpy_to_transform(map_H_odom_mat)
        map_H_odom.header.stamp = self.get_clock().now().to_msg()
        map_H_odom.header.frame_id = 'map'
        map_H_odom.child_frame_id = 'odom'

        # [FLAG 3] - map_H_odom transform is computed
        #self.get_logger().info('[FLAG 3]')

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'odom'
        # Extracting translation
        position = map_H_odom.transform.translation
        msg.pose.pose.position.x = position.x
        msg.pose.pose.position.y = position.y
        msg.pose.pose.position.z = position.z
        # Extracting rotation and converting to quaternion
        rotation = ros2_numpy.geometry.transform_to_numpy(map_H_odom.transform.rotation)  # Convert rotation to numpy matrix
        quaternion = ros2_numpy.geometry.numpy_to_quaternion(rotation)  # Convert to quaternion
        msg.pose.pose.orientation.x = quaternion.x
        msg.pose.pose.orientation.y = quaternion.y
        msg.pose.pose.orientation.z = quaternion.z
        msg.pose.pose.orientation.w = quaternion.w

        # Publish the odom message
        self.publisher_.publish(msg)


        # [FLAG 4] - map_H_odom transform is broadcasted
        #self.get_logger().info('[FLAG 4]')
        return False

def main(args=None):

    # ROS 2 node initialization and execution
    rclpy.init(args = args)

    odom_publisher = OdomPublisher()

    while not odom_publisher.publish_odom():
        pass

    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()