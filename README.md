# Mission Planner 2

## About
New BumblebeeAS mission planner stack using py_trees for AUV4 competition tasks.
Refer to the [notion page](https://www.notion.so/nusbbas/Mission-Planner-2-1683cacaefa180c28574f8291d6e2977) for documentation to get started. The development guide is at the bottom of the documentation.

## Quickstart

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
```bash
python scripts/render.py
```

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
