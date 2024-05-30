#!/bin/bash

SSH_PASS='nao' # password to be used to ssh into NAO with SSHPASS
NAO_IP='192.168.10.100' # current IP-address of NAO on this network

# SSH into NAO (when on NAO_NET) and execute the remote commands
sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no nao@$NAO_IP << 'EOF'
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
exit
EOF

# source the necessary setup files in the active terminal
source /opt/ros/humble/setup.bash
source ~/NAO_WS/install/setup.bash

# start the ros driver nodes on NAO
gnome-terminal --tab -- /bin/bash -c 'source /opt/ros/humble/setup.bash && source ~/NAO_WS/install/setup.bash && ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:='$NAO_IP'; exec bash'

# start the stt node which converts /audio topics into /ask_gpt topic
gnome-terminal --tab -- /bin/bash -c 'source /opt/ros/humble/setup.bash && source ~/NAO_WS/install/setup.bash && ros2 run ros_stt start_stt_node; exec bash'

# start the touch_node which converts sensor topics into /ask_gpt topic
gnome-terminal --tab -- /bin/bash -c 'source /opt/ros/humble/setup.bash && source ~/NAO_WS/install/setup.bash && ros2 run touch_node start_touch_node; exec bash'

# start the ros_gpt node which converts /prompt_gpt topics into a /gpt_response topic
gnome-terminal --tab -- /bin/bash -c 'source /opt/ros/humble/setup.bash && source ~/NAO_WS/install/setup.bash && ros2 run ros_gpt start_gpt_node; exec bash'

# start the parser node which converts /gpt_response topics into a /pose and /speech topic
gnome-terminal --tab -- /bin/bash -c 'source /opt/ros/humble/setup.bash && source ~/NAO_WS/install/setup.bash && ros2 run parser parser_node; exec bash'

# start the nao_poser node which converts /pose topics into physicle poses on nao
gnome-terminal --tab -- /bin/bash -c 'source /opt/ros/humble/setup.bash && source ~/NAO_WS/install/setup.bash && ros2 run nao_poser start_poser_node; exec bash'



