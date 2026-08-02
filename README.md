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

- Built-in objects: 
  - YCB dataset: `002_master_chef_can`, `003_cracker_box`, and `006_mustard_bottle`, etc.
  - Geometric primitives: `triangle`, `box`, `cylinder`

- Supported modalities: combination of `region`, `depth`, and `texture`.

- Three typical examples to run:
  1. Synthetic data (for quick test)
  2. Recorded sequence (*e.g.*, benchmark dataset)
  3. Camera (real-world application)

The tracker never reads camera intrinsics from an object or tracker config. It waits until the camera has published both `Image` and `CameraInfo` before it initializes M3T.

The optional synthetic and sequence nodes in this package are camera publishers for development only, so their own intrinsics are configured in their ROS parameter sections.


## Build

From the ROS 2 workspace:

```bash
cd <workspace_dir>/src
git clone https://github.com/wei-hsuan-cheng/m3t_ros2.git

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


## Example 1: Run with online synthetic data

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  object:=006_mustard_bottle \
  modalities:=region,depth,texture \
  source:=synthetic \
  rviz:=true

# object:=002_master_chef_can | 003_cracker_box | 006_mustard_bottle | etc.
```

**Per-frame refinement** params can be configured in [`config/m3t.yaml`](./config/m3t.yaml).

The tracker runs at least `min_corr_iterations` and at most `max_corr_iterations`, with `n_update_iterations` pose updates per correspondence round.

It stops early after both pose-change thresholds remain satisfied for `convergence_required_rounds` consecutive rounds. Set `adaptive_iterations: false` to always run `max_corr_iterations`.

```yaml
adaptive_iterations: true
min_corr_iterations: 2
max_corr_iterations: 10
n_update_iterations: 5
convergence_translation_threshold: 0.0001  # [m]
convergence_rotation_threshold_deg: 0.05   # [deg]
convergence_required_rounds: 5
```


## Example 2: Run with a recorded sequence (*e.g.*, benchmark dataset)

Below shows how to test `m3t` with [FAST-YCB](https://github.com/wei-hsuan-cheng/fast-ycb) dataset.

FAST-YCB `.float` depth files are converted to the configured `depth_scale` in memory. The included real-sequence configurations play every RGB-D frame once at a configurable rate (originally recorded at 30 [Hz]), and use the frame-0 DOPE pose for one-time initialization.

Download the desired sequence, *e.g.*, `003_cracker_box_real`, through:

```bash
cd <dataset_path>
git clone https://github.com/wei-hsuan-cheng/fast-ycb.git

cd <dataset_path>/fast-ycb
bash tools/download/download_dataset.sh 003_cracker_box_real
```


### Required dataset files for FAST-YCB sequence

After extraction, a supported real sequence must have this layout:

```text
<dataset_path>/fast-ycb/003_cracker_box_real/
├── cam_K.json
├── rgb/
│   ├── 0.png
│   ├── ...
│   └── 1681.png
├── depth/
│   ├── 0.float
│   ├── ...
│   └── 1681.float
└── dope/
    └── poses.txt
```

The downloader provides these files in their original FAST-YCB format. No offline conversion, depth rescaling, generated PNG depth images, or temporary preprocessing directory is required.

During playback, `m3t_image_publisher_node` reads each `.float` depth frame in [m] and converts it in memory to a ROS `16UC1` image using `depth_scale: 0.001` from [`config/m3t.yaml`](./config/m3t.yaml).

### Required config files for FAST-YCB sequence

The sequence config YAML files (object-specific) have to be provided. 

For example, this repo already contains [`fast_ycb/003_cracker_box_real.yaml`](./config/sequences/fast_ycb/003_cracker_box_real.yaml) and [`fast_ycb/006_mustard_bottle_real.yaml`](./config/sequences/fast_ycb/006_mustard_bottle_real.yaml).

These config files should contain:
- [`sequence_dir`](./config/sequences/fast_ycb/003_cracker_box_real.yaml) that points to the extracted dataset directory
- [`camera intrinsics`](./config/sequences/fast_ycb/003_cracker_box_real.yaml) matching `cam_K.json`
- [`initial_pose`](./config/sequences/fast_ycb/003_cracker_box_real.yaml) matching the frame-0 pose converted from `dope/poses.txt`. (The initial pose is applied once and is not fed back during tracking)

Launch the example with sequence:

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  object:=003_cracker_box \
  modalities:=region,depth,texture \
  source:=sequence \
  sequence_config:="$(ros2 pkg prefix m3t_ros2)/share/m3t_ros2/config/sequences/fast_ycb/003_cracker_box_real.yaml" \
  sequence_dir:=<dataset_path>/fast-ycb/003_cracker_box_real \
  init_mode:=static \
  image_outputs:=overlay,keypoints \
  rviz:=true
```

The YCB object configs contain the `geometry2body_pose` that maps each raw Google 16k mesh into the centered NVDU frame used by `dope/poses.txt`. These matrices come from [NVIDIA Dataset Utilities](https://github.com/NVIDIA/Dataset_Utilities); they are transposed into M3T's row-major parameter layout and their translations are converted from centimetres to metres.

FAST-YCB real sequences provide DOPE estimates but no ground truth, so tracker logs report `no-GT` and cannot measure accuracy.


## Example 3: Run with an real camera

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  object:=box \
  modalities:=region,depth,texture \
  source:=topics \
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

The script preserves the official ID-prefixed YCB name, downloads each object into `assets/ycb/<ycb_object_id>/`, and creates `config/objects/ycb/<ycb_object_id>.yaml`. Generated configs for master chef, cracker, and mustard include their NVIDIA Dataset Utilities transform from the raw Google 16k mesh to the centered NVDU body frame.


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


### 3. Optionally register it as a built-in object

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

### 4. Launch custom object example

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  object:=my_object \
  modalities:=region,depth,texture \
  source:=synthetic \
  init_mode:=gt \
  rviz:=true
```


## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
