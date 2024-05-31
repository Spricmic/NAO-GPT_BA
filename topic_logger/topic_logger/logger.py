import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import csv

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
        self.csv_file = open('prompt_gpt_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Elapsed Time (s)', 'Message'])

    def listener_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        self.get_logger().info(f'Received: "{msg.data}" | Elapsed Time: {elapsed_time:.2f} seconds')

        # Write to CSV file
        self.csv_writer.writerow([elapsed_time, msg.data])

    def __del__(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = TopicLogger()
    
    # Countdown before starting the node
    for i in range(3, 0, -1):
        print(i)
        time.sleep(1)
    
    print("Starting GPTListener node...")
    
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
