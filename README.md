# carmen_player_ros

carmen_player_ros is a player of CARMEN log files to use them in ROS. You can easily test 2D LiDAR-based SLAM and localization on ROS with this player.



CARMEN log files are datasets for 2D laser scanner. See for more details [Pre-2014 Robotics 2D-Laser Datasets](https://www.ipb.uni-bonn.de/datasets/).





# How to use

```
$ git clone https://github.com/NaokiAkai/carmen_player_ros.git
$ cd carmen_player_ros
$ catkin_make
$ source devel/setup.bash
$ rosrun carmen_player_ros carmen_player_ros [carmen_logfile]
```

For example to use MIT CSAIL logfile,

```
$ wget http://www2.informatik.uni-freiburg.de/~stachnis/datasets/datasets/csail/csail-newcarmen.log.gz
$ gzip -d csail-newcarmen.log.gz
$ rosrun carmen_player_ros carmen_player_ros csail-newcarmen.log
```

In addition, a launch file is provided in the carmen_player_ros package and it can be used as

```
$ roslaunch carmen_player_ros carmen_player_ros.launch
```

Please edit the launch file for your purpose.

