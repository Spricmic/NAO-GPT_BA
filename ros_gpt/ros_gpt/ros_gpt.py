import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from .gpt_interface import GPT_Interface

class GptPublisherNode(Node):
    def __init__(self):
        super().__init__('gpt_node')
        self.get_logger().info("starting gpt node.")
        # create publisher and subscriber nodes
        self.stt_subscription = self.create_subscription(String, '/ask_gpt', self.question_callback, 10)
        self.text_publisher = self.create_publisher(String, '/gpt_response', 10)
        self.gpt = GPT_Interface()  # create GPT_Interface instance
        self.get_logger().info("gpt node initalized succesfully.")
        

    def publish_once(self, response):
        #create the message 
        response_msg = String()
        response_msg.data = response

        # publish the message once
        self.text_publisher.publish(response_msg)
        


    def question_callback(self, msg):
        question = msg.data
        self.get_logger().info(f"gpt_Node: Recived the following msg on /ask_gpt: {question}")
        response = self.gpt.ask_gpt(question)
        self.get_logger().info(f"gpt_Node: The model responded with: {response}")

        #publish the msg to the /gpt_response topic
        self.publish_once(response)



def main(args=None):  
    rclpy.init(args=args)
    gpt_node = GptPublisherNode()  # create the publisher class an populate the text
    rclpy.spin(gpt_node)

    #cleanup
    gpt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
