import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
import json
import os

class PoseConverterNode(Node):
    def __init__(self):
        super().__init__('pose_converter_node')
        self.get_logger().info("starting Nao_poser node.")
        self.subscription = self.create_subscription(String, '/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(JointAnglesWithSpeed, '/joint_angles', 10)
        self.pose_data = self.load_pose_data()
        self.get_logger().info("Nao_poser node initalized succesfully.")


    def load_pose_data(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        poses_file_path = os.path.join(script_dir, 'poses.json')
        with open(poses_file_path) as pose_file:
            return json.load(pose_file)
        

    def pose_callback(self, msg):
        pose_name = msg.data
        self.get_logger().info(f"Poser_Node: Recived the following on /pose: {pose_name}")
        for pose in self.pose_data:

            if pose['pose_name'] == pose_name:
            
                self.get_logger().info(f"Found pose: {pose['pose_name']}")

                joint_angles_msg = JointAnglesWithSpeed()
                joint_angles_msg.header.stamp = self.get_clock().now().to_msg()
                joint_angles_msg.header.frame_id = ''
                joint_angles_msg.joint_names = pose['parts']
                joint_angles_msg.joint_angles = pose['angles']
                joint_angles_msg.speed = 0.3
                joint_angles_msg.relative = False

                self.publisher.publish(joint_angles_msg)

                break

            else:
                self.get_logger().debug(f"compared to: {pose['pose_name']}")


def main(args=None):
    rclpy.init(args=args)
    pose_converter_node = PoseConverterNode()
    rclpy.spin(pose_converter_node)
    pose_converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
