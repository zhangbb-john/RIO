import rosbag
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

# Define the rotation matrix R and the offset
R = np.array([[0.99102897, 0.13260436, -0.0166632],  
[-0.13334472, 0.98945426, -0.05656376],
[0.00898688, 0.05827827, 0.99825993]])


offset = np.array([-4.6082068, -1.15361245, 0.04214228])  # Example offset vector

# Input and output bag files
input_bag_path = "../dataset/exp/Sequence_3.bag"
output_bag_path = "../dataset/exp/Sequence_3_modified_gt2.bag"
num_topic = 0
# Open input bag for reading
with rosbag.Bag(input_bag_path, 'r') as input_bag:
    # Open output bag for writing
    with rosbag.Bag(output_bag_path, 'w') as output_bag:
        for topic, msg, t in input_bag.read_messages():
            # Check if the topic is the odometry topic
            if topic == "/pos_vel_mocap/odom_TA":
                num_topic = num_topic + 1
                if (num_topic % 50 == 0):
                    print('obtained ', num_topic, ' topics')
                # Save the original message as a new topic
                output_bag.write(topic + "_original", msg, t)

                # Extract position from the Odometry message
                position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])

                # Transform the position using R * (position - offset)
                transformed_position = np.dot(R.transpose(), position - offset)

                # Update the odometry message with the transformed position
                msg.pose.pose.position.x = transformed_position[0]
                msg.pose.pose.position.y = transformed_position[1]
                msg.pose.pose.position.z = transformed_position[2]

                # Write the modified message back to the bag
                output_bag.write(topic, msg, t)
            else:
                # Copy all other topics unchanged
                output_bag.write(topic, msg, t)

print("Processing complete. Modified bag saved as", output_bag_path)
