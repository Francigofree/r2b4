#!/usr/bin/env bash
# Rebuild only the SLAM Toolbox 2.8.5 Ceres plugin with one solver thread.
# The immutable RoboStack environment and its locked artifacts stay untouched.
set -eo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
base_prefix=${R2B4_SLAM_PREFIX:-"${repo_dir}/runtime/deps/slam-toolbox"}
overlay_prefix=${R2B4_SLAM_SOLVER_PREFIX:-"${repo_dir}/runtime/deps/slam-toolbox-solver"}
base_prefix=$(realpath -- "${base_prefix}")
overlay_prefix=$(realpath -m -- "${overlay_prefix}")
source_commit=ec8f7635dea317b531c419f798f87d90a336f32e

test -x "${base_prefix}/bin/cmake"
test -x /usr/bin/g++
/usr/bin/python3 - "${base_prefix}" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
p = Path(sys.argv[1]) / 'share/slam_toolbox/package.xml'
version = ET.parse(p).getroot().findtext('version')
if version != '2.8.5':
    raise SystemExit(f'Expected installed SLAM Toolbox 2.8.5, got {version!r}')
PY

mkdir -p -- "${repo_dir}/tmp"
build_dir=$(mktemp -d "${repo_dir}/tmp/slam_solver_build.XXXXXX")
printf 'Build directory: %s\n' "${build_dir}"

# Fetch just the existing plugin translation unit, its headers and its license.
# Each artifact is verified before the local one-line thread-limit patch.
/usr/bin/python3 - "${build_dir}" "${source_commit}" <<'PY'
from pathlib import Path
import hashlib
import sys
import urllib.request

root = Path(sys.argv[1]) / 'source'
commit = sys.argv[2]
files = {
    'solvers/ceres_solver.cpp': 'e55f6442834e9dc450b049570f34399d0654f86df956597af6eab700fcb7c118',
    'solvers/ceres_solver.hpp': 'b99c5d7f012827c396d6ac5377eddf4c3c56b60dc91c624d012682aa6c0adc6a',
    'solvers/ceres_utils.h': 'ed56f5d2a9843b3029ed85fa46db44de024e3c2a16036774a3c4dc73d0fde08a',
    'LICENSE': '20c17d8b8c48a600800dfd14f95d5cb9ff47066a9641ddeab48dc54aec96e331',
}
for name, expected in files.items():
    url = f'https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/{commit}/{name}'
    data = urllib.request.urlopen(url, timeout=30).read()
    if hashlib.sha256(data).hexdigest() != expected:
        raise SystemExit(f'Source checksum mismatch: {name}')
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
print(f'Verified SLAM Toolbox 2.8.5 source commit {commit}')
PY
patch --batch --fuzz=0 -d "${build_dir}/source" -p1 < "${repo_dir}/deploy/slam_toolbox-solver-thread-limit.patch"

cat > "${build_dir}/source/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(r2b4_slam_toolbox_solver LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem thread)
add_library(r2b4_ceres_solver_plugin SHARED solvers/ceres_solver.cpp)
target_include_directories(r2b4_ceres_solver_plugin PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}" "${R2B4_BASE_PREFIX}/include"
  "${R2B4_BASE_PREFIX}/include/slam_toolbox")
target_link_directories(r2b4_ceres_solver_plugin PRIVATE "${R2B4_BASE_PREFIX}/lib")
ament_target_dependencies(r2b4_ceres_solver_plugin rclcpp_lifecycle std_srvs pluginlib
  tf2_ros tf2_geometry_msgs nav_msgs sensor_msgs visualization_msgs)
target_link_libraries(r2b4_ceres_solver_plugin
  Ceres::ceres Boost::serialization Boost::filesystem Boost::thread
  "${R2B4_BASE_PREFIX}/lib/libkartoSlamToolbox.so")
set_target_properties(r2b4_ceres_solver_plugin PROPERTIES
  BUILD_RPATH "${R2B4_BASE_PREFIX}/lib")
CMAKE

# ROS environment hooks are allowed to refer to unset shell variables.
source "${base_prefix}/setup.bash"
"${base_prefix}/bin/cmake" -S "${build_dir}/source" -B "${build_dir}/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O2 -DNDEBUG' \
  -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DR2B4_BASE_PREFIX="${base_prefix}" \
  -DPython3_EXECUTABLE="${base_prefix}/bin/python"
nice -n 10 ionice -c 3 taskset -c "${R2B4_SLAM_BUILD_CPU:-1}" \
  "${base_prefix}/bin/cmake" --build "${build_dir}/build" --parallel 1

# Override only this plugin resource. The slam_toolbox package/config lookup
# still resolves to the underlay; the XML belongs to this separate package.
mkdir -p -- "${overlay_prefix}/lib" \
  "${overlay_prefix}/share/r2b4_slam_toolbox_solver" \
  "${overlay_prefix}/share/ament_index/resource_index/packages" \
  "${overlay_prefix}/share/ament_index/resource_index/slam_toolbox__pluginlib__plugin"
install -m 755 "${build_dir}/build/libr2b4_ceres_solver_plugin.so" \
  "${overlay_prefix}/lib/.libr2b4_ceres_solver_plugin.so.tmp"
mv -- "${overlay_prefix}/lib/.libr2b4_ceres_solver_plugin.so.tmp" \
  "${overlay_prefix}/lib/libr2b4_ceres_solver_plugin.so"
install -m 644 "${build_dir}/source/LICENSE" \
  "${overlay_prefix}/share/r2b4_slam_toolbox_solver/LICENSE"
cat > "${overlay_prefix}/share/r2b4_slam_toolbox_solver/package.xml" <<'XML'
<?xml version="1.0"?>
<package format="3">
  <name>r2b4_slam_toolbox_solver</name>
  <version>2.8.5</version>
  <description>SLAM Toolbox 2.8.5 Ceres plugin with one solver thread.</description>
  <maintainer email="robot@localhost">R2B4</maintainer>
  <license>LGPL-2.1-or-later</license>
</package>
XML
cat > "${overlay_prefix}/share/r2b4_slam_toolbox_solver/solver_plugins.xml" <<'XML'
<library path="r2b4_ceres_solver_plugin">
  <class name="solver_plugins::CeresSolver" type="solver_plugins::CeresSolver" base_class_type="karto::ScanSolver">
    <description>SLAM Toolbox 2.8.5 Ceres solver; num_threads=1.</description>
  </class>
</library>
XML
: > "${overlay_prefix}/share/ament_index/resource_index/packages/r2b4_slam_toolbox_solver"
printf '%s\n' 'share/r2b4_slam_toolbox_solver/solver_plugins.xml' > \
  "${overlay_prefix}/share/ament_index/resource_index/slam_toolbox__pluginlib__plugin/slam_toolbox"
printf 'source_commit=%s\nceres_num_threads=1\n' "${source_commit}" > \
  "${overlay_prefix}/share/r2b4_slam_toolbox_solver/build-info.txt"
printf 'Built %s\nPrepend this prefix to AMENT_PREFIX_PATH before starting SLAM.\n' "${overlay_prefix}"
