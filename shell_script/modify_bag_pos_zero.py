import rosbag
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

# Define the rotation matrix R and the offset

input_bag_path = "../rio_output_seq3_2025-02-11-14-08-42-0--30.bag"
output_bag_path = "../rio_output_seq3_2025-02-11-14-08-42-0--30.bag_modified_altitude0.bag"
gt_topic = '/lidar_ground_truth'
odom_topic = '/estimated_pose'

num_topic = 0
# Open input bag for reading
with rosbag.Bag(input_bag_path, 'r') as input_bag:
    # Open output bag for writing
    with rosbag.Bag(output_bag_path, 'w') as output_bag:
        for topic, msg, t in input_bag.read_messages():
            # Check if the topic is the odometry topic
            if topic == gt_topic:
                num_topic = num_topic + 1
                if (num_topic % 50 == 0):
                    print('obtained ', num_topic, ' topics')
                # Save the original message as a new topic
                output_bag.write(topic + "_original", msg, t)

                # Extract position from the Odometry message
                position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     0])

                # Update the odometry message with the transformed position
                msg.pose.pose.position.x = position[0]
                msg.pose.pose.position.y = position[1]
                msg.pose.pose.position.z = position[2]

                # Write the modified message back to the bag
                output_bag.write(topic, msg, t)
            elif topic == odom_topic:
                output_bag.write(topic + "_original", msg, t)
                position = np.array([msg.pose.position.x,
                                     msg.pose.position.y,
                                     0])
                msg.pose.position.x = position[0]
                msg.pose.position.y = position[1]
                msg.pose.position.z = position[2]
                output_bag.write(topic, msg, t)       
            else:
                # Copy all other topics unchanged
                output_bag.write(topic, msg, t)

print("Processing complete. Modified bag saved as", output_bag_path)
