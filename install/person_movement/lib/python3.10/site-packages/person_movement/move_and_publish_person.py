import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, PointStamped

class MoveAndPublishPersonNode(Node):
    def __init__(self):
        super().__init__('move_and_publish_person_node')
        # Client to set model state in Gazebo
        self.cli = self.create_client(SetModelState, '/set_model_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Publisher for /person_position
        self.publisher_ = self.create_publisher(PointStamped, '/person_position', 10)
        
        # Define waypoints: (time, position)
        self.waypoints = [
            (0.0, Point(x=4.0, y=1.5, z=0.0)),
            (5.0, Point(x=4.0, y=5.0, z=0.0)),
            (10.0, Point(x=4.0, y=1.5, z=0.0))
        ]
        self.current_time = 0.0
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # Fixed orientation (no rotation)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Update at 10 Hz

    def timer_callback(self):
        self.current_time += 0.1  # Increment time by timer period (0.1s)
        
        # Find current and next waypoint for interpolation
        for i in range(len(self.waypoints) - 1):
            t0, pos0 = self.waypoints[i]
            t1, pos1 = self.waypoints[i + 1]
            if self.current_time >= t0 and self.current_time < t1:
                fraction = (self.current_time - t0) / (t1 - t0)
                x = pos0.x + (pos1.x - pos0.x) * fraction
                y = pos0.y + (pos1.y - pos0.y) * fraction
                z = pos0.z + (pos1.z - pos0.z) * fraction
                
                # Call /set_model_state to update position in Gazebo
                req = SetModelState.Request()
                req.model_state.model_name = 'Person 2 - Walking'
                req.model_state.pose.position.x = x
                req.model_state.pose.position.y = y
                req.model_state.pose.position.z = z
                req.model_state.pose.orientation.x = self.orientation[0]
                req.model_state.pose.orientation.y = self.orientation[1]
                req.model_state.pose.orientation.z = self.orientation[2]
                req.model_state.pose.orientation.w = self.orientation[3]
                self.cli.call_async(req)
                
                # Publish the position to /person_position
                msg = PointStamped()
                msg.header.frame_id = 'world'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x = x
                msg.point.y = y
                msg.point.z = z
                self.publisher_.publish(msg)
                
                self.get_logger().info(f'Moving to ({x}, {y}, {z}) at time {self.current_time}')
                break
        
        # Loop back to start
        if self.current_time >= self.waypoints[-1][0]:
            self.current_time = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = MoveAndPublishPersonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()