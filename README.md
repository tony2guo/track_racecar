# track_racecar
The racecar can stay on track, do obstacle avoidance, and obey traffic sign.

## Dependencies
[YOLO ROS](https://github.com/leggedrobotics/darknet_ros)
```bash
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
```

## Installation
```bash
roscd track_racecar/install
bash install.sh
```
Download [traffic_light_and_sign.weights](https://drive.google.com/file/d/1DtZ6CmAAV_JgOK7YKwRsDAlO1Nl6GIi1/view?usp=sharing) to track_racecar/config/yolo/

## Usage
startup
```bash
cd
. startup.sh
```
kill roslaunch
```bash
. killall.sh
```