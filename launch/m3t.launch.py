# SPDX-License-Identifier: MIT
"""Unified ROS 2 launch interface for M3T.

Examples:
  ros2 launch m3t_ros2 m3t.launch.py \
    object:=006_mustard_bottle \
    modalities:=region,depth,texture

  ros2 launch m3t_ros2 m3t.launch.py \
    source:=sequence sequence_config:=/data/box/sequence.yaml \
    object:=box \
    modalities:=region,depth

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
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


BUILTIN_OBJECTS = {
    "triangle": os.path.join("primitives", "triangle.yaml"),
    "box": os.path.join("primitives", "box.yaml"),
    "cylinder": os.path.join("primitives", "cylinder.yaml"),
    "002_master_chef_can": os.path.join("ycb", "002_master_chef_can.yaml"),
    "003_cracker_box": os.path.join("ycb", "003_cracker_box.yaml"),
    "006_mustard_bottle": os.path.join("ycb", "006_mustard_bottle.yaml"),
}


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _read_ros_parameters(path, node_name):
    with open(path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    parameters = {}
    parameters.update(document.get("/**", {}).get("ros__parameters", {}))
    for key in (node_name, "/" + node_name):
        parameters.update(document.get(key, {}).get("ros__parameters", {}))
    return parameters


def _absolute_asset_path(path, config_path, parameter_name):
    if not path:
        return ""
    resolved = os.path.expanduser(str(path))
    if not os.path.isabs(resolved):
        resolved = os.path.join(os.path.dirname(config_path), resolved)
    resolved = os.path.abspath(resolved)
    if not os.path.isfile(resolved):
        raise RuntimeError(f"{parameter_name} asset does not exist: {resolved}")
    return resolved


def _resolve_object(context):
    selected_object = _value(context, "object")
    object_config_override = _value(context, "object_config")
    package_share = get_package_share_directory("m3t_ros2")

    if object_config_override:
        object_config = os.path.abspath(
            os.path.expanduser(object_config_override)
        )
    else:
        if selected_object not in BUILTIN_OBJECTS:
            supported = ", ".join(sorted(BUILTIN_OBJECTS))
            raise RuntimeError(
                f"unknown built-in object '{selected_object}' "
                f"(choose {supported}) or pass "
                "object_config:=/absolute/path/object.yaml"
            )
        object_config = os.path.join(
            package_share,
            "config",
            "objects",
            BUILTIN_OBJECTS[selected_object],
        )

    if not os.path.isfile(object_config):
        raise RuntimeError(f"object config does not exist: {object_config}")

    parameters = _read_ros_parameters(object_config, "m3t_tracker_node")
    geometry_path = _absolute_asset_path(
        parameters.get("geometry_path", ""),
        object_config,
        "geometry_path",
    )
    if not geometry_path:
        raise RuntimeError(f"geometry_path missing in {object_config}")
    texture_path = _absolute_asset_path(
        parameters.get("texture_path", ""),
        object_config,
        "texture_path",
    )

    mesh_resource = str(parameters.get("mesh_resource", "")).strip()
    if not mesh_resource:
        mesh_resource = "file://" + geometry_path

    return {
        "config": object_config,
        "geometry_path": geometry_path,
        "texture_path": texture_path,
        "mesh_resource": mesh_resource,
    }


def _parameter_file(path):
    return ParameterFile(path, allow_substs=True)


def _launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    source_mode = _value(context, "source").strip().lower()
    if source_mode not in ("synthetic", "sequence", "topics"):
        raise RuntimeError("source must be one of: synthetic, sequence, topics")

    package_share = get_package_share_directory("m3t_ros2")
    object_data = _resolve_object(context)

    config_file = os.path.abspath(
        os.path.expanduser(_value(context, "config_file"))
    )
    if not os.path.isfile(config_file):
        raise RuntimeError(f"main config does not exist: {config_file}")

    sequence_config = _value(context, "sequence_config")
    if sequence_config:
        sequence_config = os.path.abspath(os.path.expanduser(sequence_config))
        if not os.path.isfile(sequence_config):
            raise RuntimeError(
                f"sequence config does not exist: {sequence_config}"
            )
    if source_mode == "sequence" and not sequence_config:
        raise RuntimeError(
            "sequence_config is required when source:=sequence"
        )

    init_mode = _value(context, "init_mode").strip().lower()
    if init_mode not in ("gt", "tf", "static"):
        raise RuntimeError("init_mode must be one of: gt, tf, static")

    resolved_values = {
        "resolved_geometry_path": object_data["geometry_path"],
        "resolved_texture_path": object_data["texture_path"],
        "resolved_mesh_resource": object_data["mesh_resource"],
        "resolved_use_gt_initial_pose": init_mode in ("gt", "tf"),
        "resolved_publish_gt": init_mode == "gt",
    }

    resolved_configurations = [
        SetLaunchConfiguration(name, _launch_value(value))
        for name, value in resolved_values.items()
    ]

    def common_parameters():
        return [
            _parameter_file(object_data["config"]),
            _parameter_file(config_file),
        ]

    if source_mode == "synthetic":
        synthetic_source_node = Node(
            package="m3t_ros2",
            executable="m3t_synthetic_source_node",
            name="m3t_synthetic_source",
            output="screen",
            parameters=common_parameters(),
        )
        source_node = synthetic_source_node
    elif source_mode == "sequence":
        image_publisher_node = Node(
            package="m3t_ros2",
            executable="m3t_image_publisher_node",
            name="m3t_image_publisher",
            output="screen",
            parameters=[
                _parameter_file(object_data["config"]),
                _parameter_file(sequence_config),
                _parameter_file(config_file),
            ],
        )
        source_node = image_publisher_node

    tracker_node = Node(
        package="m3t_ros2",
        executable="m3t_tracker_node",
        name="m3t_tracker_node",
        output="screen",
        parameters=common_parameters(),
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(package_share, "rviz", "m3t.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return (
        resolved_configurations
        + [
           source_node, 
           tracker_node, 
           rviz_node,
          ]
    )


def generate_launch_description():
    package_share = get_package_share_directory("m3t_ros2")
    config_default = os.path.join(package_share, "config", "m3t.yaml")
    model_cache_default = PathJoinSubstitution(
        ["auto_generated", "m3t", LaunchConfiguration("object")]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "object", 
            default_value="box"
        ),
        DeclareLaunchArgument(
            "modalities",
            default_value="region,depth,texture",
            description="Comma-separated combination of region, depth, and texture",
        ),
        DeclareLaunchArgument(
            "source",
            default_value="synthetic",
            description="synthetic, sequence, or topics",
        ),
        DeclareLaunchArgument(
            "object_config",
            default_value="",
            description="Custom ROS object-parameter YAML; overrides object",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=config_default,
            description="Main ROS parameter YAML",
        ),
        DeclareLaunchArgument(
            "model_cache_dir",
            default_value=model_cache_default,
            description="Writable per-object model cache",
        ),
        DeclareLaunchArgument(
            "sequence_config",
            default_value="",
            description="ROS parameter YAML containing sequence metadata/GT",
        ),
        DeclareLaunchArgument(
            "init_mode", default_value="gt", description="gt, tf, or static"
        ),
        DeclareLaunchArgument(
            "motion_mode",
            default_value="six_dof_sine",
            description="Synthetic motion: six_dof_sine, orbit, or static",
        ),
        DeclareLaunchArgument(
            "image_outputs",
            default_value="overlay,keypoints",
            description=(
                "Tracker images: none, overlay, keypoints, or overlay,keypoints"
            ),
        ),
        DeclareLaunchArgument(
            "color_topic", 
            default_value="/camera/color/image_raw"
        ),
        DeclareLaunchArgument(
            "depth_topic", 
            default_value="/camera/depth/image_raw"
        ),
        DeclareLaunchArgument(
            "color_info_topic", 
            default_value="/camera/color/camera_info"
        ),
        DeclareLaunchArgument(
            "depth_info_topic", 
            default_value="/camera/depth/camera_info"
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
