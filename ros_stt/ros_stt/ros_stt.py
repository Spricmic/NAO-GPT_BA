import rclpy
from std_msgs.msg import String
from naoqi_bridge_msgs.msg import AudioBuffer
from rclpy.node import Node
from .stt_azure_interface import SttAzureInterface
from .stt_presampler import SttPresampler

class SttPublisherNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.get_logger().info("starting stt node.")
        # create publisher and subscriber nodes
        self.audio_subscription = self.create_subscription(AudioBuffer, '/audio', self.audio_callback, 10)
        self.text_publisher = self.create_publisher(String, '/ask_gpt', 10)
        #self.gpt = GPT_Interface()  # create GPT_Interface instance
        self.get_logger().info("stt node initalized succesfully.")
        

    def publish_once(self, response):
        #create the message 
        response_msg = String()
        response_msg.data = response

        # publish the message once
        self.text_publisher.publish(response_msg)
        


    def audio_callback(self, msg):
        pass



def main(args=None):  
    rclpy.init(args=args)
    gpt_node = SttPublisherNode()  # create the publisher class an populate the text
    rclpy.spin(gpt_node)

    #cleanup
    gpt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
