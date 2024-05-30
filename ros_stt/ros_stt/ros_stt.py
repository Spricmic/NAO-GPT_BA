import rclpy
import os
import time
from threading import Timer, Thread
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
        self.text_publisher = self.create_publisher(String, '/prompt_gpt', 10)
        self.stt_presampler = SttPresampler()  # create a presampler instance.
        self.stt_modul = SttAzureInterface()  # creat a STT Interface instance.
        self.recording_time = 5  # debugging time to record a wave fiel
        self.get_logger().info("stt node initalized succesfully.")
        self.start_time = time.time()
        self.wav_was_saved = False
        self.start_recording()
        

    def publish_once(self, response):
        #create the message 
        response_msg = String()
        response_msg.data = response

        # publish the message once
        self.text_publisher.publish(response_msg)
        

    def audio_callback(self, msg):
        try:
            if (time.time() - self.start_time) <= 5:
                audio_sample = msg.data
                self.stt_presampler.extend_audio(audio_sample)
                print(f"{audio_sample}")
            else:
                if not self.wav_was_saved:
                    self.stop_recording()
                    self.wav_was_saved = True
                else:
                    pass

        except Exception as e:
            self.get_logger().error(f"Error in audio_callback: {e}")


    def start_recording(self):
        self.get_logger().info("start recording")
        self.stt_presampler.start_recording()
        

    def stop_recording(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        wav_file_path = os.path.join(script_dir, 'output.wav')
        print("saving file MOTHERFUCKER!!!")
        self.stt_presampler.stop_recording(wav_file_path)
        print(f"The FUCKER was saved at: {wav_file_path}")
        self.stt_modul.evaluate_wav(wav_file_path)
    	


def main(args=None):  
    print("hey this is a test")
    rclpy.init(args=args)
    stt_node = SttPublisherNode()  # create the publisher class and populate the text
    
    rclpy.spin(stt_node)
    



    #cleanup
    stt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()