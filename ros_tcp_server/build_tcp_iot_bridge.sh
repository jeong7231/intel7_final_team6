cd ~/ros2_ws

echo "colcon build only tcp_iot_bridge"
colcon build --symlink-install --packages-select tcp_iot_bridge

source install/setup.bash





