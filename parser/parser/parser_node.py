import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
import re

class ParserNode(Node):
    def __init__(self):
        super().__init__('parser_node')
        self.get_logger().info("starting parser node.")
        self.subscription = self.create_subscription(String, '/gpt_response', self.response_callback, 10)
        self.pose_publisher = self.create_publisher(String, '/pose', 10)
        self.speech_publisher = self.create_publisher(String, '/speech', 10)
        self.get_logger().info("parser node initalized succesfully.")
        

    def response_callback(self, msg):
        response = msg.data
        self.get_logger().info(f"Parser_Node: Recived the following msg on /gpt_response: {response}")

        pattern = r'\[(.*?)\]'  # pattern used to find content in [] brackets
        matches = re.findall(pattern, response)  # search in response for the pattern

        if matches:
            bracket_text = matches[0]  # only takes the first pose given in brackets
            remaining_text = re.sub(pattern, '', response)
            remaining_text = remaining_text.strip()
            self.get_logger().info(f"Parser_Node: Publishing to /pose: {bracket_text}")
            self.get_logger().info(f"Parser_Node: Publishing to /speech: {remaining_text}")

            # publish the messages on the respective topic
            pose_msg = String()
            speech_msg = String()
            pose_msg.data = bracket_text
            speech_msg.data = remaining_text
            self.pose_publisher.publish(pose_msg)
            self.speech_publisher.publish(speech_msg)

        else:
            self.get_logger().error(f"No pose was found in the response of /gpt_response msg.\nThe pose was:{bracket_text}")


def main(args=None):
    rclpy.init(args=args)
    parser_node = ParserNode()
    rclpy.spin(parser_node)
    parser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
