# m3t_ros2

`m3t_ros2` is the unified ROS 2 interface for M3T.

The tracker is independent of its image source:

```text
camera/image publisher                       m3t_tracker_node
  RGB Image -------------------------------> color camera
  RGB CameraInfo --------------------------> color intrinsics
  depth Image -----------------------------> depth camera
  depth CameraInfo ------------------------> depth intrinsics
```

- Built-in objects: `triangle`, `box`, `cylinder`, `002_master_chef_can`, `003_cracker_box`, and `006_mustard_bottle`.
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
  assets/primitives/<object>/        primitive mesh assets
  assets/ycb/<ycb_object_id>/        YCB mesh, material, and texture assets
  config/m3t.yaml                    common parameters and launch substitutions
  config/objects/primitives/<object>.yaml  primitive object parameters
  config/objects/ycb/<ycb_object_id>.yaml  YCB object parameters
  launch/m3t.launch.py               one launch interface
```

The launch file loads the selected object YAML, optional sequence YAML, and then `m3t.yaml` as substitution-enabled ROS parameter files. Launch arguments map directly into `m3t.yaml` with entries such as `publish_rate: $(var publish_rate)`; the launch file does not construct node parameter dictionaries.

## Run with the online synthetic source

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic object:=006_mustard_bottle rviz:=true
# object:=box | triangle | cylinder | 002_master_chef_can |
#         003_cracker_box | 006_mustard_bottle
```

Per-frame refinement is configured under `m3t_tracker_node.ros__parameters` in `config/m3t.yaml`. The tracker runs at least `min_corr_iterations` and at most `max_corr_iterations`, with `n_update_iterations` pose updates per correspondence round. It stops early after both pose-change thresholds remain satisfied for `convergence_required_rounds` consecutive rounds. Set `adaptive_iterations: false` to always run `max_corr_iterations`.

Overlay and keypoint image topics are enabled by default for RViz. Use `image_outputs:=none` when those debug images are not needed.

```yaml
adaptive_iterations: true
min_corr_iterations: 2
max_corr_iterations: 10
n_update_iterations: 5
convergence_translation_threshold: 0.0001  # meter
convergence_rotation_threshold_deg: 0.05
convergence_required_rounds: 5
```

## Run only the tracker with an external camera

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=topics \
  object:=box \
  modalities:=region,depth,texture \
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
  object:=box modalities:=region,depth,texture
```

The sequence node is read-only. File patterns, frame count, camera intrinsics, and optional `gt_poses` are stored in the sequence YAML. Source rate, looping, frame names, and topics use the same launch arguments as the synthetic source. It never creates files in the dataset or package tree. A sequence can set `wait_for_tracker_ready: true` to hold its first frame while M3T loads or generates model caches and performs one-time detection; playback begins when the tracker publishes readiness.

FAST-YCB `.float` depth files are converted to the configured `depth_scale` in memory. The real-mustard configuration plays all 775 RGB-D frames once at the recorded 30 Hz rate and uses the frame-0 DOPE pose for one-time initialization:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=sequence \
  sequence_config:="$(ros2 pkg prefix m3t_ros2)/share/m3t_ros2/config/sequences/fast_ycb/006_mustard_bottle_real.yaml" \
  sequence_dir:=/root/mac_src/object_pose_tracking/dataset/fast-ycb/006_mustard_bottle_real \
  object:=006_mustard_bottle \
  modalities:=region,depth,texture \
  init_mode:=static \
  image_outputs:=overlay,keypoints \
  rviz:=true
```

For this sequence, `init_mode:=static` selects M3T's `StaticDetector`. Its `initial_pose` is the frame-0 entry from FAST-YCB `dope/poses.txt`, converted from axis-angle to quaternion in the sequence YAML. It is applied once and is not fed back during tracking. The mustard object's `geometry2body_pose` maps the raw Google 16k mesh into the centered NVDU frame used by `dope/poses.txt`.

The dataset directory must contain every `rgb/<index>.png` and `depth/<index>.float` from index 0 through 774. FAST-YCB real sequences provide DOPE estimates but no ground truth, so tracker logs report `no-GT` and cannot measure accuracy.

Run the same pipeline without RViz as an automated smoke test:

```bash
M3T_FAST_YCB_DIR=/root/mac_src/object_pose_tracking/dataset/fast-ycb/006_mustard_bottle_real \
ros2 run m3t_ros2 m3t_smoke_test fast-ycb-real
```

For an interactive RViz run with both tracker image outputs, use:

```bash
M3T_FAST_YCB_DIR=/root/mac_src/object_pose_tracking/dataset/fast-ycb/006_mustard_bottle_real \
ros2 run m3t_ros2 m3t_smoke_test fast-ycb-real \
  --rviz --image-outputs overlay,keypoints
```

`--rviz` keeps the launch running after the smoke check; press Ctrl-C to stop it. The headless default remains `image_outputs:=none`. The accepted image selections are `none`, `overlay`, `keypoints`, and `overlay,keypoints`. Use `--keep-running` without `--rviz` when viewing published images in another ROS tool.

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

The script preserves the official ID-prefixed YCB name, downloads each object into `assets/ycb/<ycb_object_id>/`, and creates `config/objects/ycb/<ycb_object_id>.yaml`.

### 1. Prepare the mesh

For an object stored inside this package, choose a dataset group. For example, create:

```text
m3t_ros2/assets/custom/my_object/
  model.obj
```

Use a triangulated mesh with the correct scale, coordinate frame, and surface orientation. The mesh does not have to be stored in meters, but `geometry_unit_in_meter` must convert one mesh unit to meters:

```yaml
geometry_unit_in_meter: 1.0    # mesh coordinates are meters
# geometry_unit_in_meter: 0.001  # mesh coordinates are millimeters
```

For synthetic RGB rendering with a real texture, the OBJ must contain valid UV coordinates. Place the texture beside the mesh, for example:

```text
m3t_ros2/assets/custom/my_object/
  model.obj
  model.mtl
  texture_map.png
```

`texture_path` is optional and is used by the synthetic RGB renderer. The runtime texture tracking modality itself builds image keyframes and does not require a pre-textured CAD model when tracking a real camera.

### 2. Create the object YAML

Create:

```text
m3t_ros2/config/objects/custom/my_object.yaml
```

Use this template:

```yaml
/**:
  ros__parameters:
    object_name: my_object
    geometry_path: "../../../assets/custom/my_object/model.obj"

    # Optional synthetic-rendering texture:
    # texture_path: "../../../assets/custom/my_object/texture_map.png"
    # Set true when the OBJ references an MTL and texture assets.
    mesh_use_embedded_materials: true

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

An external object YAML supplies the object parameters, so no C++ or launch-file modification is required. Set `object` to the object name as well so the default model-cache directory is named correctly:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic \
  object:=my_object \
  object_config:=/absolute/path/to/my_object.yaml \
  modalities:=region,depth,texture \
  init_mode:=gt \
  rviz:=true
```

For a real camera:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=topics \
  object:=my_object \
  object_config:=/absolute/path/to/my_object.yaml \
  modalities:=region,depth,texture \
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
    "triangle": os.path.join("primitives", "triangle.yaml"),
    "box": os.path.join("primitives", "box.yaml"),
    "cylinder": os.path.join("primitives", "cylinder.yaml"),
    "002_master_chef_can": os.path.join("ycb", "002_master_chef_can.yaml"),
    "003_cracker_box": os.path.join("ycb", "003_cracker_box.yaml"),
    "006_mustard_bottle": os.path.join("ycb", "006_mustard_bottle.yaml"),
    "my_object": os.path.join("custom", "my_object.yaml"),
}
```

Then launch:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic \
  object:=my_object \
  modalities:=region,depth,texture \
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
  model_cache_dir:=auto_generated/m3t/my_object_v2 \
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
