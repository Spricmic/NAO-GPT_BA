import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TopicLogger(Node):
    def __init__(self):
        super().__init__('topic_logger')
        self.subscription = self.create_subscription(
            String,
            'prompt_gpt',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.start_time = None

    def listener_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        self.get_logger().info(f'Received: "{msg.data}" | Elapsed Time: {elapsed_time:.2f} seconds')

def main(args=None):
    rclpy.init(args=args)
    node = TopicLogger()
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
