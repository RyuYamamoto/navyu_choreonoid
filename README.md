# navyu_choreonoid

## 1. Build
Build choreonoid and ros plugin.
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/choreonoid/choreonoid
git clone https://github.com/choreonoid/choreonoid_ros
./src/choreonoid/misc/script/install-requisites-ubuntu-22.04.sh # install depend packages for choreonoid
cd ../
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build Navigation Packages and Choreonoid Controller.
```bash
cd ~/ros2_ws/src
git clone https://github.com/RyuYamamoto/navyu_choreonoid
git clone https://github.com/RyuYamamoto/navyu
git clone https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If there was no willow garage model, download it manually.
```bash
mkdir ~/.gazebo/models && cd ~/.gazebo/models # If there is no directory
wget http://models.gazebosim.org/willowgarage/model.tar.gz
tar -zxvf model.tar.gz
```

## 2. Run
Launch choreonoid.
```bash
ros2 launch navyu_choreonoid navyu_choreonoid.launch.py use_rviz:=false
```

Launch Navigation.
```bash
ros2 launch navyu_choreonoid navyu_bringup.launch.py
```
