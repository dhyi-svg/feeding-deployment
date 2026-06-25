# Setup & Installation

## Requirements

- Python 3.10+
- Tested on Ubuntu 20.04
- ROS 1 (with `rospy`)

## Pre-installation

1. Install ROS and `rospy`.
2. Install [PyAudio](https://pypi.org/project/PyAudio/).

## Install the package

1. Recommended: create and source a virtualenv or conda environment.
2. Install:
   ```bash
   pip install -e ".[robot, develop]"   # full install (robot + dev tools)
   pip install -e .                     # preference-learning setup only
   ```

## Check the installation

```bash
./run_ci_checks.sh
```
Should complete all-green in 5–10 seconds.

---

# Navigation-stack dependencies

The navigation stack (Cartographer multi-lidar SLAM + `move_base`) needs a separate
dependencies workspace.

1. Create a dependencies workspace in your home directory:
   ```bash
   mkdir vention_dependencies_ws
   ```
2. Build **Cartographer** for ROS into it, following the official guide:
   <https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html>
3. Source `vention_dependencies_ws` before building this package.

## URDF meshes

Download the URDF mesh bundle and extract it into `urdf/meshes`:

<https://drive.google.com/file/d/1OZAdcuAua0Nr7p6ITxTQDwUMFzZjeR8F/view?usp=sharing>

## Build the workspace

```bash
catkin build
```

Then source your workspace in every new terminal:

```bash
source devel/setup.bash
```
