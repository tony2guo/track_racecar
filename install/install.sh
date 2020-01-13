echo copy desktop
cp -a desktops/. ~/Desktop
echo copy scripts
cp -a scripts/. ~/
echo copy rules
sudo cp -a rules/. /etc/udev/rules.d/
echo reload rules
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
echo rosdep install
rosdep install track_racecar -y
echo done