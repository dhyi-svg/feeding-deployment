# `rammp_button_detect` ament package template

Wraps the button detector as a standalone ament_python package so sheppy can launch it
(`sheppy_node_entry.yaml`). The in-repo way to run the same node is
`python3 -u -m feeding_deployment.button_press.detector_node` (see
`docs/button_press_runbook.md`); use this template only for the sheppy/RAMMP workflow.

```bash
ros2 pkg create rammp_button_detect --build-type ament_python \
  --dependencies rclpy sensor_msgs geometry_msgs cv_bridge
P=$WS/src/rammp_button_detect
cp <repo>/src/feeding_deployment/button_press/detector_node.py  $P/rammp_button_detect/button_detector_node.py
cp <repo>/src/feeding_deployment/perception/appliance_perception/reference_button_detector.py $P/rammp_button_detect/
cp mock_button_detector.py $P/rammp_button_detect/
mkdir -p $P/launch && cp launch/button_detect.launch.py $P/launch/
```
In the copied `button_detector_node.py`, replace the `feeding_deployment...reference_button_detector`
import with `from .reference_button_detector import ReferenceButtonDetector`, apply
`setup_py_snippet.txt`, then `colcon build --packages-select rammp_button_detect`.
