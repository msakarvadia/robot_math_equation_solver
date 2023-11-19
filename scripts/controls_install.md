# Install wstool
python3 -m pip install --upgrade pip
pip install -U wstool
sed -i 's/yaml.load/yaml.safe_load/g' ~/.local/lib/python3.8/site-packages/wstool/config_yaml.py

# Download ros_control Package and Dependencies
cd ~/catkin_ws/src
~/.local/bin/wstool init
~/.local/bin/wstool merge https://raw.githubusercontent.com/ros-controls/ros_control/noetic-devel/ros_control.rosinstall
~/.local/bin/wstool update
git clone git@github.com:ros-drivers/four_wheel_steering_msgs.git
git clone git@github.com:ros-controls/urdf_geometry_parser.git

# Install package
cd ~/catkin_ws
catkin_make
source devel/setup.bash