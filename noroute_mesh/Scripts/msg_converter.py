# NOT USABLE YET

import rospy
import json
from rospy_message_converter import json_message_converter
from std_msgs.msg import String

def ros2string(req):
    message = req.data
    json_str = json_message_converter.convert_ros_message_to_json(message)

def string2ros(req):
    json_str = '{"data": "Hello"}'
    message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str) 

if __name__ == "__main__":
    rospy.init_node('message_converter')
    s = rospy.Service('ros_to_string', AddTwoInts, ros2string)
    print("Ready to add two ints.")
    rospy.spin()