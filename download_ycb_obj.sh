#!/usr/bin/env bash

# Download one or more YCB Google 16k textured models and create their M3T
# object configs.
# Assets and configs are grouped under assets/ycb and config/objects/ycb.
# Run this script from the m3t_ros2 package directory.
#
# Usage:
#   ./download_ycb_obj.sh <ycb_object_id_1> <ycb_object_id_2> ...
#
# Examples:
#   ./download_ycb_obj.sh 002_master_chef_can
#
#   ./download_ycb_obj.sh \
#     002_master_chef_can \
#     003_cracker_box \
#     004_sugar_box \
#     005_tomato_soup_can \
#     006_mustard_bottle
#
# The local object name is derived by removing the leading three-digit YCB ID
# and underscore. For example, 006_mustard_bottle becomes mustard_bottle.

set -euo pipefail

script_name="$(basename "$0")"

usage() {
  cat <<EOF
Usage:
  ./${script_name} <ycb_object_id_1> <ycb_object_id_2> ...

Arguments:
  ycb_object_id  One or more official YCB names

Examples:
  ./${script_name} 002_master_chef_can

  ./${script_name} \
    002_master_chef_can \
    003_cracker_box \
    004_sugar_box \
    005_tomato_soup_can \
    006_mustard_bottle
EOF
}

if (( $# == 1 )) && [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

if (( $# == 0 )); then
  usage >&2
  exit 2
fi

# Validate the complete request before creating or overwriting any files.
for ycb_object_id in "$@"; do
  if [[ ! "${ycb_object_id}" =~ ^[0-9]{3}_[a-z0-9_]+$ ]]; then
    echo "Error: '${ycb_object_id}' must look like 003_cracker_box." >&2
    exit 2
  fi
done

temp_root="$(mktemp -d)"

cleanup() {
  rm -rf "${temp_root}"
}
trap cleanup EXIT

download_object() {
  local ycb_object_id="$1"
  local asset_name="${ycb_object_id#???_}"
  local object_label="${asset_name//_/ }"
  local object_dir="assets/ycb/${asset_name}"
  local config_dir="config/objects/ycb"
  local config_path="${config_dir}/${asset_name}.yaml"
  local source_url="https://ycb-benchmarks.s3.amazonaws.com/data/google/${ycb_object_id}_google_16k.tgz"
  local temp_dir="${temp_root}/${ycb_object_id}"
  local archive_path="${temp_dir}/${ycb_object_id}_google_16k.tgz"
  local obj_file
  local mtl_file
  local texture_file
  local obj_sha
  local mtl_sha
  local texture_sha

  mkdir -p "${temp_dir}"

  echo "Downloading YCB ${ycb_object_id}..."
  curl -fL --retry 3 "${source_url}" -o "${archive_path}"
  tar -xzf "${archive_path}" -C "${temp_dir}"

  obj_file="$(find "${temp_dir}" -type f -name textured.obj -print -quit)"
  mtl_file="$(find "${temp_dir}" -type f -name textured.mtl -print -quit)"
  texture_file="$(find "${temp_dir}" -type f -name texture_map.png -print -quit)"

  if [[ -z "${obj_file}" || -z "${mtl_file}" || -z "${texture_file}" ]]; then
    echo "Error: ${ycb_object_id} does not contain textured.obj, textured.mtl, and texture_map.png." >&2
    return 1
  fi

  mkdir -p "${object_dir}" "${config_dir}"
  install -m 644 "${obj_file}" "${object_dir}/model.obj"
  install -m 644 "${texture_file}" "${object_dir}/texture_map.png"

  LC_ALL=C sed 's/[[:space:]]*$//' "${mtl_file}" \
    > "${object_dir}/textured.mtl"
  chmod 644 "${object_dir}/textured.mtl"

  obj_sha="$(sha256sum "${object_dir}/model.obj" | awk '{print $1}')"
  mtl_sha="$(sha256sum "${object_dir}/textured.mtl" | awk '{print $1}')"
  texture_sha="$(sha256sum "${object_dir}/texture_map.png" | awk '{print $1}')"

  cat > "${object_dir}/SOURCE.md" <<EOF
# YCB ${object_label} mesh

\`model.obj\`, \`textured.mtl\`, and \`texture_map.png\` are the Google 16k textured model for YCB object \`${ycb_object_id}\`, downloaded from \`${source_url}\`.

The YCB data is licensed under Creative Commons Attribution 4.0 International (CC BY 4.0). The original \`textured.obj\` is retained as \`model.obj\` to match the package-wide asset naming convention. Trailing whitespace is removed from the MTL so RViz/OGRE resolves its texture filename correctly.

SHA-256:

- \`model.obj\`: \`${obj_sha}\`
- \`textured.mtl\`: \`${mtl_sha}\`
- \`texture_map.png\`: \`${texture_sha}\`
EOF

  cat > "${config_path}" <<EOF
/**:
  ros__parameters:
    object_name: ${asset_name}
    geometry_path: "../../../assets/ycb/${asset_name}/model.obj"
    texture_path: "../../../assets/ycb/${asset_name}/texture_map.png"
    mesh_use_embedded_materials: true
    geometry_unit_in_meter: 1.0
    geometry_counterclockwise: true
    geometry_enable_culling: false
    geometry2body_pose: [
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
    ]
    body_id: 3
    region_id: 3
    # [x, y, z, roll, pitch, yaw], with angles in radians.
    initial_pose: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    gt_initial_pose: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    translation_amplitude: [0.0, 0.0, 0.0]
EOF

  echo "Downloaded YCB ${ycb_object_id} assets to ${object_dir}"
  echo "Generated M3T object config: ${config_path}"
}

for ycb_object_id in "$@"; do
  download_object "${ycb_object_id}"
done

echo "Finished ${#} YCB object(s)."