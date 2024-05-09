#!/usr/bin/env python
import rosbag
from geometry_msgs.msg import Twist

def process_bag(input_bag_file, output_bag_file):
    # Open the original bag file for reading
    with rosbag.Bag(input_bag_file, 'r') as bag:
        # Create a new bag file to write the transformed data
        with rosbag.Bag(output_bag_file, 'w') as outbag:
            for topic, msg, t in bag.read_messages():
                # Write the original message to the output bag
                outbag.write(topic, msg, t)

                if topic == "/turtle1/cmd_vel":
                    # Create a new Twist message for turtle2 with opposite values
                    new_msg = Twist()
                    new_msg.linear.x = -msg.linear.x
                    new_msg.linear.y = -msg.linear.y
                    new_msg.linear.z = -msg.linear.z
                    new_msg.angular.x = -msg.angular.x
                    new_msg.angular.y = -msg.angular.y
                    new_msg.angular.z = -msg.angular.z
                    
                    # Write the new message to the new topic in the output bag
                    outbag.write("/turtle2/cmd_vel", new_msg, t)

if __name__ == "__main__":
    input_bag_file = 'test.bag'  # Update this path
    output_bag_file = 'out.bag'  # Update this path
    process_bag(input_bag_file, output_bag_file)

