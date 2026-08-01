# SPDX-License-Identifier: MIT
"""Unified ROS 2 launch interface for M3T.

Examples:
  ros2 launch m3t_ros2 m3t.launch.py \
    object:=box \
    modalities:=region,depth

  ros2 launch m3t_ros2 m3t.launch.py \
    source:=sequence sequence_config:=/data/box/sequence.yaml \
    object:=box \
    modalities:=region

  ros2 launch m3t_ros2 m3t.launch.py \
    source:=topics \
    object_config:=/data/object/object.yaml \
    init_mode:=tf
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BUILTIN_OBJECTS = {
    "triangle": "triangle.yaml",
    "box": "box.yaml",
    "cylinder": "cylinder.yaml",
    "master_chef_can": "master_chef_can.yaml",
    "cracker_box": "cracker_box.yaml",
    "mustard": "mustard.yaml",
}


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    return _value(context, name).strip().lower() in ("1", "true", "yes", "on")


def _read_ros_parameters(path, node_name):
    with open(path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    parameters = {}
    parameters.update(document.get("/**", {}).get("ros__parameters", {}))
    for key in (node_name, "/" + node_name):
        parameters.update(document.get(key, {}).get("ros__parameters", {}))
    return parameters


def _xdg_runtime_dir():
    configured = os.environ.get("XDG_RUNTIME_DIR")
    if configured:
        return configured
    ros_home = os.environ.get(
        "ROS_HOME", os.path.join(os.path.expanduser("~"), ".ros")
    )
    runtime_dir = os.path.join(
        ros_home, "m3t", "runtime-" + str(os.getuid())
    )
    os.makedirs(runtime_dir, mode=0o700, exist_ok=True)
    os.chmod(runtime_dir, 0o700)
    return runtime_dir


def _resolve_object(context):
    object_name = _value(context, "object")
    object_config_override = _value(context, "object_config")
    mesh_override = _value(context, "mesh_resource")
    package_share = get_package_share_directory("m3t_ros2")

    if object_config_override:
        object_config = os.path.abspath(
            os.path.expanduser(object_config_override)
        )
    else:
        if object_name not in BUILTIN_OBJECTS:
            supported = ", ".join(sorted(BUILTIN_OBJECTS))
            raise RuntimeError(
                f"unknown built-in object '{object_name}' (choose {supported}) "
                "or pass object_config:=/absolute/path/object.yaml"
            )
        object_config = os.path.join(
            package_share, "config", "objects", BUILTIN_OBJECTS[object_name]
        )

    if not os.path.isfile(object_config):
        raise RuntimeError(f"object config does not exist: {object_config}")
    parameters = _read_ros_parameters(object_config, "m3t_tracker_node")
    geometry_path = parameters.get("geometry_path", "")
    if not geometry_path:
        raise RuntimeError(f"geometry_path missing in {object_config}")
    if not os.path.isabs(geometry_path):
        geometry_path = os.path.abspath(
            os.path.join(os.path.dirname(object_config), geometry_path)
        )
    if not os.path.isfile(geometry_path):
        raise RuntimeError(f"geometry asset does not exist: {geometry_path}")

    object_name = str(parameters.get("object_name", object_name))
    mesh_resource = mesh_override or "file://" + geometry_path
    texture_path = str(parameters.get("texture_path", ""))
    if texture_path and not os.path.isabs(texture_path):
        texture_path = os.path.abspath(
            os.path.join(os.path.dirname(object_config), texture_path)
        )
    if texture_path and not os.path.isfile(texture_path):
        raise RuntimeError(f"texture asset does not exist: {texture_path}")
    default_modalities = str(
        parameters.get("modalities", "region,depth")
    )
    embedded_mode = _value(
        context, "mesh_use_embedded_materials"
    ).strip().lower()
    if embedded_mode == "auto":
        embedded = bool(texture_path)
    elif embedded_mode in ("1", "true", "yes", "on"):
        embedded = True
    elif embedded_mode in ("0", "false", "no", "off"):
        embedded = False
    else:
        raise RuntimeError(
            "mesh_use_embedded_materials must be auto, true, or false"
        )
    return (
        object_name,
        object_config,
        geometry_path,
        texture_path,
        default_modalities,
        mesh_resource,
        embedded,
    )


def launch_setup(context, *args, **kwargs):
    source_mode = _value(context, "source").strip().lower()
    if source_mode not in ("synthetic", "sequence", "topics"):
        raise RuntimeError("source must be one of: synthetic, sequence, topics")

    (
        object_name,
        object_config,
        geometry_path,
        texture_path,
        default_modalities,
        mesh_resource,
        mesh_embedded,
    ) = _resolve_object(context)
    package_share = get_package_share_directory("m3t_ros2")
    config_file = _value(context, "config_file") or os.path.join(
        package_share, "config", "m3t.yaml"
    )
    config_file = os.path.abspath(os.path.expanduser(config_file))
    if not os.path.isfile(config_file):
        raise RuntimeError(f"main config does not exist: {config_file}")
    sequence_config = _value(context, "sequence_config")
    sequence_parameters = {}
    if sequence_config:
        sequence_config = os.path.abspath(os.path.expanduser(sequence_config))
        if not os.path.isfile(sequence_config):
            raise RuntimeError(
                f"sequence config does not exist: {sequence_config}"
            )
        sequence_parameters = _read_ros_parameters(
            sequence_config, "m3t_image_publisher"
        )
    modalities_override = _value(context, "modalities").strip()
    modalities = (
        default_modalities
        if modalities_override in ("", "auto")
        else modalities_override
    )
    sequence_dir = _value(context, "sequence_dir") or str(
        sequence_parameters.get("sequence_dir", "")
    )
    if sequence_dir:
        sequence_dir = os.path.abspath(os.path.expanduser(sequence_dir))

    cache_override = _value(context, "model_cache_dir")
    model_cache_dir = os.path.abspath(
        os.path.expanduser(
            cache_override
            or os.path.join("auto_generated", "m3t", object_name)
        )
    )

    init_mode = _value(context, "init_mode").strip().lower()
    if init_mode not in ("gt", "tf", "static"):
        raise RuntimeError("init_mode must be one of: gt, tf, static")
    use_tf_initial_pose = init_mode in ("gt", "tf")
    topics = {
        "color_topic": _value(context, "color_topic"),
        "depth_topic": _value(context, "depth_topic"),
        "color_info_topic": _value(context, "color_info_topic"),
        "depth_info_topic": _value(context, "depth_info_topic"),
    }
    world_frame = _value(context, "world_frame")
    gt_frame = _value(context, "gt_frame")
    source_rate = float(_value(context, "source_rate"))

    nodes = [
        SetEnvironmentVariable(
            name="XDG_RUNTIME_DIR", value=_xdg_runtime_dir()
        )
    ]
    if source_mode == "synthetic":
        synthetic_overrides = {
            "object_name": object_name,
            "geometry_path": geometry_path,
            "texture_path": texture_path,
            "publish_rate": source_rate,
            "n_frames": int(_value(context, "n_frames")),
            "loop": _bool(context, "loop"),
            "depth_noise": float(_value(context, "depth_noise")),
            "distortion": float(_value(context, "distortion")),
            "depth_scale": float(_value(context, "depth_scale")),
            "world_frame": world_frame,
            "camera_frame": _value(context, "camera_frame"),
            "gt_frame": gt_frame,
            "mesh_resource": mesh_resource,
            "mesh_scale": float(_value(context, "mesh_scale")),
            "mesh_use_embedded_materials": mesh_embedded,
            **topics,
        }
        if _value(context, "motion_mode"):
            synthetic_overrides["motion_mode"] = _value(
                context, "motion_mode"
            )
        if _value(context, "spin_turns"):
            synthetic_overrides["spin_turns"] = float(
                _value(context, "spin_turns")
            )
        if _value(context, "nod_degrees"):
            synthetic_overrides["nod_degrees"] = float(
                _value(context, "nod_degrees")
            )
        nodes.append(
            Node(
                package="m3t_ros2",
                executable="m3t_synthetic_source_node",
                name="m3t_synthetic_source",
                output="screen",
                parameters=[
                    config_file,
                    object_config,
                    synthetic_overrides,
                ],
            )
        )
    elif source_mode == "sequence":
        if not sequence_dir:
            raise RuntimeError(
                "sequence_dir is required when source:=sequence"
            )
        nodes.append(
            Node(
                package="m3t_ros2",
                executable="m3t_image_publisher_node",
                name="m3t_image_publisher",
                output="screen",
                parameters=[
                    config_file,
                    object_config,
                    *([sequence_config] if sequence_config else []),
                    {
                        "sequence_dir": sequence_dir,
                        "object_name": object_name,
                        "geometry_path": geometry_path,
                        "publish_rate": source_rate,
                        "loop": _bool(context, "loop"),
                        "world_frame": world_frame,
                        "publish_gt": init_mode == "gt",
                        "mesh_resource": mesh_resource,
                        "mesh_scale": float(_value(context, "mesh_scale")),
                        "mesh_use_embedded_materials": mesh_embedded,
                        **topics,
                    }
                ],
            )
        )

    nodes.append(
        Node(
            package="m3t_ros2",
            executable="m3t_tracker_node",
            name="m3t_tracker_node",
            output="screen",
            parameters=[
                config_file,
                object_config,
                {
                    "object_name": object_name,
                    "geometry_path": geometry_path,
                    "modalities": modalities,
                    "model_cache_dir": model_cache_dir,
                    "depth_scale": float(_value(context, "depth_scale")),
                    "sync_tolerance": float(_value(context, "sync_tolerance")),
                    "event_driven": _bool(context, "event_driven"),
                    "image_outputs": _value(context, "image_outputs"),
                    "track_rate": float(_value(context, "track_rate")),
                    "publish_rate": float(_value(context, "publish_rate")),
                    "log_period": float(_value(context, "log_period")),
                    "world_frame": world_frame,
                    "mesh_resource": mesh_resource,
                    "mesh_scale": float(_value(context, "mesh_scale")),
                    "mesh_use_embedded_materials": mesh_embedded,
                    "use_gt_initial_pose": use_tf_initial_pose,
                    "gt_frame": gt_frame,
                    **topics,
                }
            ],
        )
    )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                os.path.join(
                    get_package_share_directory("m3t_ros2"),
                    "rviz",
                    "m3t.rviz",
                ),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source",
                default_value="synthetic",
                description="synthetic, sequence, or topics",
            ),
            DeclareLaunchArgument("object", default_value="box"),
            DeclareLaunchArgument(
                "object_config",
                default_value="",
                description="Custom ROS object-parameter YAML; overrides object",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="Main ROS parameter YAML",
            ),
            DeclareLaunchArgument(
                "mesh_resource",
                default_value="",
                description="RViz mesh URI for a custom body",
            ),
            DeclareLaunchArgument(
                "mesh_use_embedded_materials",
                default_value="auto",
                description=(
                    "auto uses OBJ/MTL materials when texture_path is set; "
                    "true or false forces the behavior"
                ),
            ),
            DeclareLaunchArgument("mesh_scale", default_value="1.0"),
            DeclareLaunchArgument(
                "modalities",
                default_value="region,depth,texture",
                description=(
                    "region, depth, texture, any combination, or auto to "
                    "use the object YAML recommendation"
                ),
            ),
            DeclareLaunchArgument(
                "model_cache_dir",
                default_value="",
                description=(
                    "Writable model cache; default auto_generated/m3t/<object>"
                ),
            ),
            DeclareLaunchArgument("sequence_dir", default_value=""),
            DeclareLaunchArgument(
                "sequence_config",
                default_value="",
                description="ROS parameter YAML containing sequence metadata/GT",
            ),
            DeclareLaunchArgument(
                "init_mode",
                default_value="gt",
                description="gt, tf, or static",
            ),
            DeclareLaunchArgument("source_rate", default_value="60.0"),
            DeclareLaunchArgument("track_rate", default_value="0.0"),
            DeclareLaunchArgument("publish_rate", default_value="60.0"),
            DeclareLaunchArgument("log_period", default_value="2.0"),
            DeclareLaunchArgument("event_driven", default_value="true"),
            DeclareLaunchArgument(
                "image_outputs",
                default_value="overlay,keypoints",
                description=(
                    "Tracker images: none, overlay, keypoints, "
                    "or overlay,keypoints"
                ),
            ),
            DeclareLaunchArgument("sync_tolerance", default_value="0.02"),
            DeclareLaunchArgument("n_frames", default_value="240"),
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("depth_noise", default_value="10.0"),
            DeclareLaunchArgument("distortion", default_value="0.05"),
            DeclareLaunchArgument("depth_scale", default_value="0.001"),
            DeclareLaunchArgument(
                "motion_mode",
                default_value="",
                description="Optional synthetic override: orbit or static",
            ),
            DeclareLaunchArgument(
                "spin_turns",
                default_value="",
                description="Optional synthetic YAML override",
            ),
            DeclareLaunchArgument(
                "nod_degrees",
                default_value="",
                description="Optional synthetic YAML override",
            ),
            DeclareLaunchArgument("world_frame", default_value="camera"),
            DeclareLaunchArgument("camera_frame", default_value="camera"),
            DeclareLaunchArgument("gt_frame", default_value="object_gt"),
            DeclareLaunchArgument(
                "color_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_raw"
            ),
            DeclareLaunchArgument(
                "color_info_topic",
                default_value="/camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "depth_info_topic",
                default_value="/camera/depth/camera_info",
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )