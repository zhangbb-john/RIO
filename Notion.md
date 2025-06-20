- [1. Steps to reproduce the results](#1-steps-to-reproduce-the-results)
  - [1.1. Need docker?](#11-need-docker)
    - [1.1.1. Download the docker image by](#111-download-the-docker-image-by)
    - [1.1.2. Run the docker](#112-run-the-docker)
  - [1.2. ROS](#12-ros)
    - [1.2.1. coloradar](#121-coloradar)
    - [1.2.2. reproduce the groundtruth](#122-reproduce-the-groundtruth)
    - [1.2.3. modify\_bag\_gt](#123-modify_bag_gt)
    - [1.2.4. demo](#124-demo)
    - [1.2.5. rqt\_graph](#125-rqt_graph)
- [2. Code interpretation](#2-code-interpretation)
  - [2.1. what is frame](#21-what-is-frame)
- [3. inherit from vins-mono](#3-inherit-from-vins-mono)
  - [3.1. reducevector](#31-reducevector)
  - [3.2.](#32)
- [4. Sensor Charactoristic/Front end](#4-sensor-charactoristicfront-end)
  - [4.1. Radar](#41-radar)
    - [4.1.1. /radar\_frame](#411-radar_frame)
  - [4.2. what is row\_step](#42-what-is-row_step)
  - [4.3.](#43)
  - [4.4. IMU](#44-imu)
- [5. Back end/Optimization](#5-back-endoptimization)
- [6. Output Data Extraction](#6-output-data-extraction)
  - [6.1. map](#61-map)
    - [6.1.1. /sub\_map\_frame](#611-sub_map_frame)
    - [6.1.2. /tracking\_frame (trackingPubTopic)](#612-tracking_frame-trackingpubtopic)
      - [6.1.2.1. Is /tracking\_frame the overlap of the current frame with sub\_map?](#6121-is-tracking_frame-the-overlap-of-the-current-frame-with-sub_map)
      - [6.1.2.2. Is radarFeatureFactor.pointRelation contains each point from the current frame, where the point without match turns out to be zeros-match?](#6122-is-radarfeaturefactorpointrelation-contains-each-point-from-the-current-frame-where-the-point-without-match-turns-out-to-be-zeros-match)
      - [6.1.2.3. tracking\_frame is world frame right?](#6123-tracking_frame-is-world-frame-right)
  - [6.2. Pose](#62-pose)
    - [6.2.1. /estimated\_pose (world to inertial frame)](#621-estimated_pose-world-to-inertial-frame)
    - [6.2.2. rio](#622-rio)
- [7. RIO frame management](#7-rio-frame-management)
- [8. Issue](#8-issue)
  - [8.1. ERROR: Unable to start XML-RPC server, port 11311 is already in use (just occasionally occurs)](#81-error-unable-to-start-xml-rpc-server-port-11311-is-already-in-use-just-occasionally-occurs)
  - [8.2. z error is very large](#82-z-error-is-very-large)
  - [8.3. Bug](#83-bug)
    - [8.3.1. the rio msg is strange,  always being jumping](#831-the-rio-msg-is-strange--always-being-jumping)
    - [8.3.2. the header timestamp of radar\_frame is wrong](#832-the-header-timestamp-of-radar_frame-is-wrong)
    - [8.3.3. the rviz is dark](#833-the-rviz-is-dark)
- [9. Xu Yang questions](#9-xu-yang-questions)

# 1. Steps to reproduce the results
## 1.1. Need docker?
###  1.1.1. Download the docker image by 
```
bash docker.sh -b
```
(then, it start download the file and build
```
function build() {
    docker build \
    -t rio \
    -f $SCRIPT_DIR/Dockerfile \
    $SCRIPT_DIR/..
}
```
)
![alt text](notion/docker.png)


![alt text](notion/image_location.png)

### 1.1.2. Run the docker 
```
bash docker.sh -r
```

*This starts the Docker container and attaches your terminal to it. Once the container starts successfully, the terminal will switch to being inside the Docker container.*

(mapping the current folder to docker 
```
function run() {
    docker run -it --rm \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v $SCRIPT_DIR/../:/ws/src \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    rio \
    /bin/bash
}
```
)

## 1.2. ROS
bash 1

```
roscore &
rviz -d /ws/src/rio/config/RIO.rviz
```

bash 2
```
python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/ars548.yaml -d /ws/src/dataset/exp/Sequence_1.bag -r 1 -p 1
```

```
python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/ars548.yaml -d /ws/src/dataset/exp/Sequence_3_modified_gt_horizontal2.bag -r 1 -p 1
```

### 1.2.1. coloradar
python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d dataset/coloradar_trim/colo_trim_outdoors_run_0_modified_gt3.bag -r 1 -p 1

python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d dataset/coloradar_trim/aspen_run0_compressed_modified_gt3.bag -r 1 -p 1

python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d dataset/coloradar_trim/edgar_classroom_run0_compressed.bag -r 1 -p 1

python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d dataset/coloradar_trim/ec_hallways_run0_compressed.bag -r 1 -p 1

python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d dataset/coloradar_trim/arpg_lab_run0_compressed.bag -r 1 -p 1

arpg_lab_run0_compressed
python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d dataset/coloradar_trim/aspen_run0_compressed.bag -r 1 -p 1

python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d dataset/coloradar_trim/colo_trim_outdoors_run_0_modified_gt2.bag -r 1 -p 1

python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d ./dataset/coloradar_trim/ec_hallways_run0_compressed_modified_gt6.bag  -r 1 -p 1

python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d dataset/coloradar_trim/aspen_run0_compressed.bag -r 1 -p 1


### 1.2.2. reproduce the groundtruth
evo_ape bag ./rio_output_seq3_2025-02-09-13-43-21_0.2toend.bag /estimated_pose /lidar_ground_truth -va --plot_mode xy --plot  --t_max_diff 0.05
evo_ape bag ./rio_output_seq3_2025-06-10-03-01-05.bag /estimated_pose /lidar_ground_truth -va --plot_mode xy --plot  --t_max_diff 0.05

### 1.2.3. modify_bag_gt
python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d ./dataset/coloradar_trim/classroom3_modified_gt_horizontal.bag  -r 1 -p 1

python3 modify_bag_pos_zero.py   ../rio_output_seq3_2025-06-10-04-55-52.bag

evo_ape bag ../rio_output_seq3_2025-06-10-06-31-13_modified_altitude0.bag /lidar_ground_truth  /estimated_pose  -va --plot_mode xy --plot  --t_max_diff 0.05

python3 modify_bag_gt.py

python3 /ws/src/docker/run.py -a -n rio -c /ws/src/rio/config/coloradar.yaml -d ./dataset/coloradar_trim/classroom3_modified_gt_horizontal.bag  -r 1 -p 1


root@ubuntu-Legion-Y9000P-IRX9:/ws/src/dataset/coloradar_trim# rostopic echo -b ./ec_hallways_run0_compressed_modified_gt6.bag -p /lidar_ground_truth >> gt.csv

### 1.2.4. demo
![alt text](notion/rio_seq1.gif)
![alt text](notion/seq1.png)
![alt text](notion/seq3_result.png)
### 1.2.5. rqt_graph 

![alt text](notion/rqtgraph.png)


# 2. Code interpretation
## 2.1. what is frame 
```
  frame = scan2scanTracker.trackPoints(frameRadarData, timeStamp);

  radarData.data.emplace_back(frame);
```

# 3. inherit from vins-mono
## 3.1. reducevector
![alt text](notion/vins_rio.png)

## 3.2. 

# 4. Sensor Charactoristic/Front end
## 4.1. Radar

![alt text](notion/data_radar.png)

### 4.1.1. /radar_frame
![alt text](notion/radar_frame.png)

![alt text](notion/radar_frame_csv.png)

angle seems to be [-60, 60]x[-20, 20]

## 4.2. what is row_step
it is how many points there are in a single frame.

## 4.3. 

## 4.4. IMU

# 5. Back end/Optimization

# 6. Output Data Extraction
## 6.1. map 
### 6.1.1. /sub_map_frame

![alt text](notion/sub_map.png)

### 6.1.2. /tracking_frame (trackingPubTopic)
![alt text](notion/tracking_frame.png)
#### 6.1.2.1. Is /tracking_frame the overlap of the current frame with sub_map?
Yes
#### 6.1.2.2. Is radarFeatureFactor.pointRelation contains each point from the current frame, where the point without match turns out to be zeros-match?
Likely
#### 6.1.2.3. tracking_frame is world frame right? 

## 6.2. Pose
### 6.2.1. /estimated_pose (world to inertial frame)


### 6.2.2. rio
seems to be groundtruth, the same name as node name


# 7. RIO frame management
all of the states uses inertial frames as body frames


# 8. Issue
## 8.1. ERROR: Unable to start XML-RPC server, port 11311 is already in use (just occasionally occurs)

```
roscore -p 11312
```

```
export ROS_MASTER_URI=http://localhost:11312
```

## 8.2. z error is very large
![alt text](notion/seq3_z_err.png)
![alt text](notion/seq1_z_small_err.png)
## 8.3. Bug
### 8.3.1. the rio msg is strange,  always being jumping
![alt text](notion/rio_bug.gif)

### 8.3.2. the header timestamp of radar_frame is wrong
now it is zero, not very good. 
also the frame is world, which is not appropriate for visualization.

### 8.3.3. the rviz is dark
we can adjust window size in the rviz file, makeing it smaller.

sometimes retry many times can help alleviate the problem.

other times waiting and retrying are okay.

# 9. Xu Yang questions
