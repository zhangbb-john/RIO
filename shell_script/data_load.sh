# this script is used to load the dataset of coloradar for viusalization
echo 'data loader'
python utils/ColoRadar_tools-master/python/bag_to_dataset_fmt.py -i ../../../dataset/classroom -o ../../../dataset/classroom --groundtruth /lidar_ground_truth