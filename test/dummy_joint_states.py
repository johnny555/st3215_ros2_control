# write a python node to publish a joint_states message on a topic

import  rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


rclpy.init()

node = Node("pubber")

pub = node.create_publisher(JointState, '/desired_joint_state', 10)

jst = JointState()
# set the joint names and positions in the message
jst.name = ['joint1']
jst.position = [2800.0]

# publish the message on the topic
pub.publish(jst)

while True:
    time.sleep(2)
    jst.position = [1500.0]
    pub.publish(jst)
    time.sleep(2)
    jst.position = [2800.0]
    pub.publish(jst)






