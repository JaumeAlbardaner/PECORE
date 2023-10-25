import rclpy
import random
import math
import tf2_ros

from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, Pose
from gazebo_msgs.srv import GetEntityState

class VelPublisher(Node):

    def __init__(self):
        super().__init__('publish_velocity')
        self.updated = False
        self.publisher_ = self.create_publisher(Twist, 'jackal_velocity_controller/cmd_vel_unstamped', 1000)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.vel_publisher)
        self.robotState = Pose()

        self.cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_entity service not available, waiting again...')


    def send_request(self):
        self.req = GetEntityState.Request()
        self.req.name = "jackal"
        self.req.reference_frame= "world"   
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.robotState = self.future.result().state.pose
        self.updated = True

    def vel_publisher(self):
        if not(self.updated):
            return

        #Goal position
        self.goal = Point()
        self.goal.x = 1.0
        self.goal.y = -1.0

        diff_x = self.goal.x - self.robotState.position.x
        diff_y = self.goal.y - self.robotState.position.y
        
        self.directional = Point() # Normalized 
        self.directional.x = diff_x / (diff_x**2+diff_y**2)**0.5
        self.directional.y = diff_y / (diff_x**2+diff_y**2)**0.5

        #Convert to linear x and angular z velocities
        x_vel = (diff_x**2 + diff_y**2)
        z_vel = math.atan2(diff_y,diff_x)
        # This doesn't actually compute the difference between angles, just the angle
        # TODO: perform current angle - desired angle
        msg = Twist()
        msg.linear.x = x_vel
        msg.angular.z = z_vel
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "vx: %f, wz: %f"' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)

    publish_velocity = VelPublisher()   

    try:
        while rclpy.ok():            
            publish_velocity.send_request()

            rclpy.spin_once(publish_velocity)
            
    except KeyboardInterrupt:
        pass


    publish_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
