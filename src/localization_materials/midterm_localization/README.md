This version is created for IMU data fusion, but is still developing state. (not good)
The algorithom is the same as version 2 code.

Put \*.pcd files to test_data/[MAP_NAME] folder like this:
```bash
test_data
│   itri_map
│   └── map.pcd
├── itri_map.zip
├── nctu_map
│   ├── first-0.pcd
│   ├── first-10.pcd
│        ...
├── nuscene_map
│   ├── map_200_600.pcd
│   ├── map_200_700.pcd
│        ...
```

After compiling, run the code by:
```bash
$ roslaunch midterm_localization solution3.launch map_name:=[itri | nuscene] use_rviz:=true
----- After showing the map in rviz, then you can play the rosbag -----
$ rosbag play [ROS_BAG] --clock -r [0.05~0.1]
```
