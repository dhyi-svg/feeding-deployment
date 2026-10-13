# Source in every terminal that runs sheppy or the button press (hybrid workflow):
#   source ~/feeding-deployment/rammp_button_press/sheppy/env.sh
# The venv goes FIRST: activating it while it is already active restores the PATH it saved at
# its first activation, which dropped /opt/ros/humble/bin when ROS came after it ("ros2: command
# not found" in a terminal that already had (.venv), 2026-10-12).
source ~/feeding-deployment/.venv/bin/activate   # --system-site-packages venv: pybullet, scipy, opencv; rclpy from /opt/ros
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash          # rammp-interfaces-ros2 v1.1.0 + rammp_button_press
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export BUTTON_CALIB=~/.ros2/easy_handeye2/calibrations/<this rig>.calib   # hand-eye file for camera_calib_tf
