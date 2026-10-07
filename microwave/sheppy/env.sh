# Source in every terminal that runs sheppy or the microwave scripts (hybrid workflow):
#   source ~/feeding-deployment/microwave/sheppy/env.sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash                 # rammp-interfaces-ros2 v1.1.0
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
_FD=$HOME/feeding-deployment
source $_FD/.venv/bin/activate                      # venv sees the system site-packages (rclpy, cv_bridge)
export PYTHONPATH=$_FD/src:$PYTHONPATH
# Jetson torch (jetson-ai-lab jp6/cu126) needs cuDSS + CUPTI from pip and cuBLAS/cuDNN from JetPack
_NV=$_FD/.venv/lib/python3.10/site-packages/nvidia
export LD_LIBRARY_PATH=$_NV/cu12/lib:$_NV/cuda_cupti/lib:/usr/local/cuda/lib64:$LD_LIBRARY_PATH
cd $_FD
unset _FD _NV
