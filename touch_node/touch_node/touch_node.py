import rclpy
import time
from std_msgs.msg import String
from rclpy.node import Node
from naoqi_bridge_msgs.msg import Bumper, HandTouch, HeadTouch


class TouchNode(Node):
    def __init__(self):
        super().__init__('touch_node')
        self.get_logger().info('Starting Touch_Node')
        # create publisher
        self.gpt_publisher = self.create_publisher(String, '/ask_gpt', 10)
        # create Subscribers
        self.bumper_subscriber = self.create_subscription(Bumper, '/bumper', self.bumper_callback, 10)
        self.hand_touch_subscriber = self.create_subscription(HandTouch, '/hand_touch', self.hand_touch_callback, 10)
        self.head_touch_subscrciber = self.create_subscription(HeadTouch, '/head_touch', self.head_touch_callback, 10)

        self.delay_time = 3  # delay until new msg will be registerd and published
        self.last_published_time = time.time()
        self.get_logger().info('Touch_Node initalized')

    def bumper_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_published_time < self.delay_time:
            return
        
        if msg.bumper == 1:
            if msg.state == 1:
                self.publish_once("[something touched your left foot]")
            elif msg.state == 0:
                self.publish_once("[something let go of your left foot]")
            else:
                pass
        elif msg.bumper == 0:
            if msg.state == 1:
                self.publish_once("[something touched your right foot]")
            elif msg.state == 0:
                self.publish_once("[something let go of your right foot]")
            else:
                pass
        else:
            pass
        self.last_published_time = current_time

    def hand_touch_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_published_time < self.delay_time:
            return

        if msg.hand in [0, 1, 2]:
            if msg.state == 1:
                self.publish_once("[something touched your right hand]")
            elif msg.state == 0:
                self.publish_once("[something let go of your right hand]")
            else:
                pass
        elif msg.hand == [3, 4, 5]:
            if msg.state == 1:
                self.publish_once("[something touched your left hand]")
            elif msg.state == 0:
                self.publish_once("[something let go of your left hand]")
            else:
                pass
        else:
            pass
        self.last_published_time = current_time

    def head_touch_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_published_time < self.delay_time:
            return
        
        if msg.button in [0, 1, 2]:
            if msg.state == 1:
                self.publish_once("[something touched your head]")
            elif msg.state == 0:
                self.publish_once("[something let go of your head]")
            else:
                pass
        else:
            pass
        self.last_published_time = current_time

    def publish_once(self, response):
        response_msg = String()
        response_msg.data = response
        self.gpt_publisher.publish(response_msg)
        self.get_logger().info(f"published: {response}")

def main(args=None):
    rclpy.init(args=args)
    touch_node = TouchNode()
    rclpy.spin(touch_node)

    touch_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
