# trajectory_test
https://github.com/ethz-asl/mav_trajectory_generation

按照过程下载安装package
过程3  wstool init 后不成功，git clone 绕过

新建package进行测试，按教程写一个cpp 在trajectory_test/src

可视化过程中：// From Trajectory class:
mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
打开rviz没反应，写一个发布器将markersArray发布出来即可

注意Cmakelist和package.xml的写法
