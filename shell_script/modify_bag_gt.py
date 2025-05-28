import rosbag
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

# Define the rotation matrix R and the offset

seq = 'seq3'
if seq == 'seq3':
    R = np.array([[0.99985743, -0.01688535, 0],
            [0.01688535, 0.99985743, 0],
            [0, 0, 1]])
    offset = np.array([-4.68683812, -1.47742145, 0])
    # R = np.array([[0.99102897, 0.13260436, -0.0166632],  
    # [-0.13334472, 0.98945426, -0.05656376],
    # [0.00898688, 0.05827827, 0.99825993]])

    # offset = np.array([-4.6082068, -1.15361245, 0.04214228])  # Example offset vector

    # Input and output bag files
    input_bag_path = "../dataset/exp/Sequence_3.bag"
    output_bag_path = "../dataset/exp/Sequence_3_modified_gt_horizontal.bag"
    gt_topic = '/pos_vel_mocap/odom_TA'
elif seq == 'coloradar2':
    R = np.array([[0.07624376, -0.99457773, 0.07072496],
        [0.99478025, 0.07105121, -0.07323915], 
        [0.06781694, 0.07593982, 0.9948035]]) 
        
    #     [[-0.14924933, 0.98480914, 0.08874454],
    # [-0.98429545, -0.13941323, -0.10828855],
    # [-0.09427139, -0.10351284, 0.99015049]])
    # offset = np.array([-0.98996716, -1.04210591, 4.17299483])
    offset = np.array([-0.7422675, 0.58759571, 0.21097775 ])
    input_bag_path = "../dataset/coloradar_trim/colo_trim_outdoors_run_0.bag"
    output_bag_path = "../dataset/coloradar_trim/colo_trim_outdoors_run_0_modified_gt2.bag"
    gt_topic = '/lidar_ground_truth'
elif seq == 'classroom3':
    # R = np.array([[0.1590413, -0.98476778, -0.07027277],
    #               [0.94111195, 0.12971072, 0.31222337],
    #               [-0.29835238, -0.11579098, 0.94740609]])
    
    # offset = np.array([-0.10162363, 0.36543084, 0.03399292])
    R = np.array([[0.13469154, -0.99088758, 0 ],
                  [0.99088758, 0.13469154, 0],
                  [0, 0, 1]])
    offset = np.array([-0.13236015, 0.79985266, 0])
    input_bag_path = "../dataset/coloradar_trim/classroom3.bag"
    output_bag_path = "../dataset/coloradar_trim/classroom3_modified_gt2.bag"
    gt_topic = '/lidar_ground_truth'

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
