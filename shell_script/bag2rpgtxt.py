# ...existing code...
#!/usr/bin/env python
import sys
import os

import rospy
import rosbag
from nav_msgs.msg import Odometry
# ...existing code...
from geometry_msgs.msg import PoseStamped

def write_posestamped_to_txt(input_bag, pose_topic, output_txt, output_txt_3d):
    """
    Extract PoseStamped messages from a specific topic and write them to a txt file.
    """
    with rosbag.Bag(input_bag, 'r') as inbag, open(output_txt, 'w') as txtfile, open(output_txt_3d, 'w') as txtfile_3d:
        txtfile.write("# timestamp tx ty tz qx qy qz qw\n")
        for topic, msg, t in inbag.read_messages(topics=[pose_topic]):
            timestamp = msg.header.stamp.to_sec()
            tx = msg.pose.position.x
            ty = msg.pose.position.y
            tz0 = 0#
            tz = msg.pose.position.z
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            # txtfile.write("{:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e}\n".format(
            #     timestamp, tx, ty, tz, qx, qy, qz, qw))
# ...existing code...
            txtfile.write("{:.2f} {:.2f} {:.2f} {:.2f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
                timestamp, tx, ty, tz0, qx, qy, qz, qw))
            txtfile_3d.write("{:.2f} {:.2f} {:.2f} {:.2f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(timestamp, tx, ty, tz, qx, qy, qz, qw))
# ...existing code...
def write_odometry_to_txt(input_bag, odom_topic, output_txt, ouptput_txt_3d=None):
    """
    Extract Odometry messages from a specific topic and write them to a txt file.
    """
    with rosbag.Bag(input_bag, 'r') as inbag, open(output_txt, 'w') as txtfile, open(ouptput_txt_3d, 'w') as txtfile_3d:
        txtfile.write("# timestamp tx ty tz qx qy qz qw\n")
        for topic, msg, t in inbag.read_messages(topics=[odom_topic]):
            timestamp = msg.header.stamp.to_sec()
            tx = msg.pose.pose.position.x
            ty = msg.pose.pose.position.y
            tz0 = 0#
            tz = msg.pose.pose.position.z
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
#             txtfile.write("{:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e}\n".format(
#                 timestamp, tx, ty, tz, qx, qy, qz, qw))
# # ...existing code...
            txtfile.write("{:.2f} {:.2f} {:.2f} {:.2f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
                timestamp, tx, ty, tz0, qx, qy, qz, qw))
            
            txtfile_3d.write("{:.2f} {:.2f} {:.2f} {:.2f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(timestamp, tx, ty, tz, qx, qy, qz, qw))
# ...existing code...
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python x.py <bagfile.bag>")
        sys.exit(1)
    input_bag = sys.argv[1]

    # Write PoseStamped from /estimated_pose to txt
    pose_topic_to_txt = '/estimated_pose'
    output_txt_est = "stamped_traj_estimate.txt"  
    if not os.path.exists("3d"):
        os.mkdir("3d")  
    output_txt_est_3d = "3d/stamped_traj_estimate.txt"

    # Create a single directory
    
    write_posestamped_to_txt(input_bag, pose_topic_to_txt, output_txt_est, output_txt_est_3d)
    print(f"PoseStamped data written to {output_txt_est}")
    a = input("are you use coloradar? (1 for yes, 0 for no): ")
    output_txt_gt = "stamped_groundtruth.txt"
    output_txt_gt_3d = "3d/stamped_groundtruth.txt"
    if a == "1":

        pose_topic_to_txt = '/gt_pose'

        write_posestamped_to_txt(input_bag, pose_topic_to_txt, output_txt_gt, output_txt_gt_3d)
        print(f"PoseStamped data written to {output_txt_gt}")
    else:
        # Write Odometry from /lidar_ground_truth to txt
        odom_topic_to_txt = '/pos_vel_mocap/odom_TA'
        write_odometry_to_txt(input_bag, odom_topic_to_txt, output_txt_gt, output_txt_gt_3d)
        print(f"Odometry ground truth written to {output_txt_gt}")
# ...existing code...