# Mission Planner 2

## About
BumblebeeAS mission planner stack using py_trees.

Useful references:

- [py_trees repositories](https://github.com/splintered-reality/py_trees_ros)
- [py_trees module API](https://py-trees.readthedocs.io/en/release-2.2.x/modules.html#)
- [py_trees_ros module API](https://py-trees.readthedocs.io/en/release-2.2.x/modules.html#)
- [py_trees_ros tutorial](https://py-trees-ros-tutorials.readthedocs.io/en/release-2.1.x/)

## Quickstart

### Installation
To install py_trees:
```
sudo apt-get install \
  ros-humble-py-trees \
  ros-humble-py-trees-ros-interfaces \
  ros-humble-py-trees-ros
```

The GUI debugging utility doesn’t exist for humble as of writing this. However, there exists a workaround to get a working installation as follows.

First install the dependencies:
```
sudo apt-get install \
  libqt5webengine5 \
  libqt5webenginewidgets5 \
  python3-pyqt5.qtwebengine
```

Then clone the following ROS packages and colcon build.
```
git clone https://github.com/splintered-reality/py_trees_js.git
git clone https://github.com/splintered-reality/py_trees_ros_viewer.git
```

### Launch the Mission Planner
```bash
ros2 launch mission_planner_2 main.launch.py
```

### Run Mission Scripts
```bash
# Run main mission
ros2 run mission_planner_2 main.py

# Run test mission
ros2 run mission_planner_2 test.py
```

### Generate Behavior Tree Diagram

See [py-trees-render](https://py-trees.readthedocs.io/en/devel/programs.html#py-trees-render).

## Development guide

See [Mission Planner conventions](docs/behavior_trees.md) for our conventions.

## Organization
- Convenience API is located in `mission_planner_2/commons`
- Trees are written under `mission_planner_2/trees/<vehicle>/<mission>`
- Main executables are written under `scripts`
- Static transform configuration is defined in `cfg/cfg.yaml`

## Static Transform Configuration

### Overview
The static transform configuration defines coordinate frame relationships for AUV4 competition tasks. This config is loaded by `main.launch.py` which launches a `static_transform_publisher` node for each transform defined.

### Coordinate Conventions

#### Frame Definition Format
- **parent_frame_id**: Reference frame
- **child_frame_id**: Target frame  
- **Translation**: x, y, z in meters
- **Rotation**: roll, pitch, yaw in degrees

#### Camera Frame Conventions

**Front Camera**
- While FACING the object:
  - Origin is at the top left corner of the object
  - +x is to the right
  - +y is downwards  
  - +z is pointing into the object plane away from the camera (right hand rule)
- To align child frames with `base_link_ned` of the AUV: apply rotation of `roll = -90, pitch = -90, yaw = 0` degrees

**Bottom Camera**
- While bottom camera is facing the object, and AUV is aligned towards the front of the object:
  - Origin is at the top left corner of the object
  - +x is to the right
  - +y is downwards
  - +z is pointing into the object plane away from the camera, into the pool floor (right hand rule)
- To align child frames with `base_link_ned` of the AUV: apply rotation of `roll = 0, pitch = 0, yaw = -90` degrees

### Transform Mathematics

**Important Note on Transform Direction**: For a transform from frame A to frame B defined in the configuration, the inputted rotation $R$ and translation vector $t$ define the **INVERSE transform** $f^{-1}: B \to A$.

Given a point $x$ in frame A, to get the corresponding point in frame B:
- Translate back to origin by $-t$
- Reverse the rotation by $R^T$
- Thus $f: A \to B$ is defined by $f(x) = R^T(x - t)$
- Inverting gives $f^{-1}: B \to A$ defined by $f^{-1}(x) = Rx + t$

**Key Takeaway**: To retrieve transform data from this configuration, query the transform from the parent frame to the child frame, not the reverse.

### Configuration Usage

1. Create a new comment header for each task in `cfg/cfg.yaml`
2. Define the transforms using the coordinate conventions above
3. Visually verify the frames in Foxglove (handle extrinsic/intrinsic rotations carefully)
