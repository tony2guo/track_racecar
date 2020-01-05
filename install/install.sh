sudo cp -a rules/. /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
rosdep install track_racecar -y