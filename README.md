# m3t_ros2

`m3t_ros2` is the unified ROS 2 interface for M3T.

The tracker is independent of its image source:

```text
camera/image publisher                       m3t_tracker_node
  RGB Image -------------------------------> color camera
  RGB CameraInfo --------------------------> color intrinsics
  depth Image (when depth is enabled) -----> depth camera
  depth CameraInfo ------------------------> depth intrinsics
```

- Built-in objects: `triangle`, `box`, `cylinder`, `mustard`, `cracker_box`, etc.
- Supported modalities: combination of `region`, `depth`, and `texture`.

The tracker never reads camera intrinsics from an object or tracker config. It waits until the external camera has published both `Image` and `CameraInfo` before it initializes M3T.

The optional synthetic and sequence nodes in this package are camera publishers for development only, so their own intrinsics are configured in their ROS parameter sections.

## Build

From the ROS 2 workspace:

```bash
cd <workspace_dir>
NUM_JOBS=2 && \
export CMAKE_BUILD_PARALLEL_LEVEL=${NUM_JOBS} && \
export MAKEFLAGS=-j${NUM_JOBS} && \
export NINJAFLAGS=-j${NUM_JOBS} && \
colcon build --symlink-install \
  --packages-up-to m3t_ros2 \
  --executor sequential --parallel-workers ${NUM_JOBS} \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
  . install/setup.bash
```

## Package layout

```text
m3t_ros2/
  assets/<object>/                   package-local object assets
  assets/ycb/<object>/               YCB mesh, material, and texture assets
  config/m3t.yaml                    common node parameters
  config/objects/<object>.yaml       package-local object parameters
  config/objects/ycb/<object>.yaml   YCB object parameters
  launch/m3t.launch.py               one launch interface
```

## Run with the online synthetic source

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic object:=box rviz:=true
# object:=box | triangle | cylinder | mustard | cracker_box |etc.
```

Per-frame refinement is configured under `m3t_tracker_node.ros__parameters` in `config/m3t.yaml`. The tracker runs at least `min_corr_iterations` and at most `max_corr_iterations`, with `n_update_iterations` pose updates per correspondence round. It stops early after both pose-change thresholds remain satisfied for `convergence_required_rounds` consecutive rounds. Set `adaptive_iterations: false` to always run `max_corr_iterations`.

```yaml
adaptive_iterations: true
min_corr_iterations: 2
max_corr_iterations: 7
n_update_iterations: 2
convergence_translation_threshold: 0.0001  # meter
convergence_rotation_threshold_deg: 0.05
convergence_required_rounds: 2
```

## Run only the tracker with an external camera

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=topics \
  object:=box \
  modalities:=region,depth \
  init_mode:=tf \
  color_topic:=/camera/color/image_raw \
  color_info_topic:=/camera/color/camera_info \
  depth_topic:=/camera/depth/image_raw \
  depth_info_topic:=/camera/depth/camera_info
```

`source:=topics` launches only `m3t_tracker_node`. No camera process is launched and no camera calibration is loaded by the tracker.

Initialization modes:

- `init_mode:=tf`: wait for `world_frame -> gt_frame`; in practice the named frame can be supplied by a detector, mocap system, or another ROS node.
- `init_mode:=static`: use `initial_pose` from the object ROS parameter YAML.
- `init_mode:=gt`: use the same TF mechanism, named explicitly for synthetic/recorded development sources. GT initializes the tracker once and is not fed back into normal tracking.

The camera driver must publish valid dimensions and the pinhole matrix `K` in `sensor_msgs/msg/CameraInfo`. With the depth modality enabled, RGB and depth timestamps must fall within `sync_tolerance` (default 0.02 seconds).

## Run a recorded sequence

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=sequence \
  sequence_config:=/data/box_sequence/sequence.yaml \
  object:=box modalities:=region,depth
```

The sequence node is read-only. File patterns, frame count, camera intrinsics, and optional `gt_poses` are ROS parameters. It never creates files in the dataset or package tree.

## Custom object

Adding an object requires:

1. A metric 3D mesh.
2. An object ROS parameter YAML.
3. An initial pose from an external initializer, TF, or the object YAML.

M3T is a local pose tracker, so the initial pose must be reasonably close to the actual object pose.

A [`download_ycb_obj.sh`](./download_ycb_obj.sh) script to download multiple objects in YCB dataset is provided for quick guidance.

```bash
cd <workspace_dir>/src/.../m3t_ros2
chmod +x download_ycb_obj.sh

./download_ycb_obj.sh \
  002_master_chef_can \
  003_cracker_box \
  004_sugar_box \
  005_tomato_soup_can \
  006_mustard_bottle
```

The script downloads each object into `assets/ycb/<object>/` and creates its parameter file at `config/objects/ycb/<object>.yaml`.


### 1. Prepare the mesh

For an object stored inside this package, create:

```text
m3t_ros2/assets/my_object/
  model.obj
```

Use a triangulated mesh with the correct scale, coordinate frame, and surface orientation. The mesh does not have to be stored in meters, but `geometry_unit_in_meter` must convert one mesh unit to meters:

```yaml
geometry_unit_in_meter: 1.0    # mesh coordinates are meters
# geometry_unit_in_meter: 0.001  # mesh coordinates are millimeters
```

For synthetic RGB rendering with a real texture, the OBJ must contain valid UV coordinates. Place the texture beside the mesh, for example:

```text
m3t_ros2/assets/my_object/
  model.obj
  model.mtl
  texture_map.png
```

`texture_path` is optional and is used by the synthetic RGB renderer. The runtime texture tracking modality itself builds image keyframes and does not require a pre-textured CAD model when tracking a real camera.

### 2. Create the object YAML

Create:

```text
m3t_ros2/config/objects/my_object.yaml
```

Use this template:

```yaml
/**:
  ros__parameters:
    object_name: my_object
    geometry_path: "../../assets/my_object/model.obj"

    # Optional synthetic-rendering texture:
    # texture_path: "../../assets/my_object/texture_map.png"

    # Used when launch is called with modalities:=auto.
    modalities: "region,depth,texture"

    geometry_unit_in_meter: 1.0
    geometry_counterclockwise: true
    geometry_enable_culling: false

    # Row-major transform from the mesh geometry frame to the tracking
    # body frame. This remains a 4x4 matrix; it is not an initial pose.
    geometry2body_pose: [
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
    ]

    # IDs must be integers in [1, 255]. They may be equal for a
    # single-object tracker.
    body_id: 42
    region_id: 42

    # Used by init_mode:=static.
    # [x, y, z, roll, pitch, yaw], meters and radians.
    initial_pose: [0.0, 0.0, 0.60, 0.0, 0.0, 0.0]

    # Used to place the object in source:=synthetic.
    # It accepts the same 6D RPY or 7D quaternion format.
    gt_initial_pose: [0.0, 0.0, 0.60, 0.0, 0.0, 0.0]
```

The quaternion alternative is:

```yaml
initial_pose: [0.0, 0.0, 0.60, 0.0, 0.0, 0.0, 1.0]
gt_initial_pose: [0.0, 0.0, 0.60, 0.0, 0.0, 0.0, 1.0]
```

Relative `geometry_path` and `texture_path` values are resolved relative to the object YAML file. Absolute paths are also supported.

If the object has discrete rotational symmetries, add the non-identity equivalent rotations as flattened row-major 3x3 matrices. For example, a 180-degree symmetry around body Z is:

```yaml
rotation_symmetries: [
  -1.0,  0.0,  0.0,
   0.0, -1.0,  0.0,
   0.0,  0.0,  1.0
]
```

### 3. Test it without changing launch code

An external object YAML overrides `object:=...`, so no C++ or launch-file modification is required:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic \
  object_config:=/absolute/path/to/my_object.yaml \
  modalities:=auto \
  init_mode:=gt \
  rviz:=true
```

For a real camera:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=topics \
  object_config:=/absolute/path/to/my_object.yaml \
  modalities:=auto \
  init_mode:=static \
  color_topic:=/camera/color/image_raw \
  color_info_topic:=/camera/color/camera_info \
  depth_topic:=/camera/depth/image_raw \
  depth_info_topic:=/camera/depth/camera_info \
  rviz:=true
```

Use `init_mode:=tf` instead of `static` when another node provides `world_frame -> gt_frame`.

### 4. Optionally register it as a built-in object

To launch the object with `object:=my_object`, add it to `BUILTIN_OBJECTS` in `launch/m3t.launch.py`. Then rebuild and source the workspace.

```python
BUILTIN_OBJECTS = {
    "triangle": "triangle.yaml",
    "box": "box.yaml",
    "cylinder": "cylinder.yaml",
    "master_chef_can": os.path.join("ycb", "master_chef_can.yaml"),
    "cracker_box": os.path.join("ycb", "cracker_box.yaml"),
    "mustard": os.path.join("ycb", "mustard.yaml"),
    "my_object": "my_object.yaml",
}
```

Then launch:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic \
  object:=my_object \
  modalities:=auto \
  init_mode:=gt \
  rviz:=true
```

### 5. Generated tracking models

The region and depth modalities automatically generate their view models on the first run:

```text
auto_generated/m3t/my_object/
  region_model.bin
  depth_model.bin
```

The first startup can therefore take longer. These files are generated caches, not source assets.

If the mesh scale, geometry, body-frame transform, or winding settings change, use a new `model_cache_dir` or remove only that object's old cache before running again:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic \
  object:=my_object \
  model_cache_dir:=/tmp/m3t_my_object_cache \
  rviz:=true
```

The cache directory must be writable.

### 6. Verify the loaded parameters

After launching, confirm that ROS loaded the intended mesh and poses:

```bash
ros2 param get /m3t_tracker_node geometry_path
ros2 param get /m3t_tracker_node initial_pose

# For source:=synthetic:
ros2 param get /m3t_synthetic_source gt_initial_pose
```

If an edited YAML appears to have no effect, confirm that the launch uses the intended `object_config`. For a built-in object, rebuild and source `install/setup.bash` so the installed package contains the new YAML and assets.


## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
