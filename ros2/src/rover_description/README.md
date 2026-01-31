After building the package with `colcon build` and sourcing the workspace, you can generate the URDF by running:

```bash
ros2 run xacro xacro \
  $(ros2 pkg prefix rover_description)/share/rover_description/urdf/rover.urdf.xacro
```

This command expands the Xacro file and outputs the generated URDF.

The image below shows the URDF visualized in **rerun**:

![Rover URDF visualized in rerun](images/rover_in_rerun.png)