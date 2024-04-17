import rclpy
from std_msgs.msg import String
from gpt_interface import GPT_Interface

class GptPublisher:
    def __init__(self):
        super().__init__('ros_gpt_node')
        self.get_logger().info("starting ros_gpt node.")
        self.node = rclpy.create_node('gpt_response')
        # create publisher nodes
        self.text_publisher = self.node.create_publisher(String, '/gpt_response', 10)
        #create subscriber node
        self.subscription = self.create_subscription(String, '/ask_gpt', self.question_callback, 10)
        self.gpt = GPT_Interface()  # create GPT_Interface instance
        self.get_logger().info("ros_gpt node initalized succesfully.")
        

    def publish_once(self, response):
        #create the message 
        response_msg = String()
        response_msg.data = response

        # publish the message once
        self.text_publisher.publish(response_msg)
        


    def question_callback(self, msg):
        question = msg.data
        self.get_logger().info(f"ros_gpt_Node: Recived the following msg on /ask_gpt: {question}")
        response = self.gpt.ask_gpt(question)
        self.get_logger().info(f"ros_gpt_Node: The model responded with: {response}")

        #publish the msg to the /gpt_response topic
        self.publish_once(response)



def main(args=None):  # initial function for testing
    rclpy.init()
    publisher_node = GptPublisher()  # create the publisher class an populate the text

    rclpy.spin(publisher_node.node)

    #cleanup
    publisher_node.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
