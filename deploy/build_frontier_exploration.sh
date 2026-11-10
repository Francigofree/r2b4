#!/usr/bin/env bash
# Build the pinned ROS 2 explore_lite provider outside the control interpreter.
set -eo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
base_prefix="${repo_dir}/runtime/deps/nav2-planner"
overlay_prefix="${repo_dir}/runtime/deps/frontier-exploration"
upstream_sha=326cf8a0b487c34246bb8f3326afbcd69576dc60
mkdir -p "${repo_dir}/tmp"
build_dir=$(mktemp -d "${repo_dir}/tmp/frontier_exploration_build.XXXXXX")
git init --quiet "${build_dir}/source"
git -C "${build_dir}/source" fetch --quiet --depth 1 \
  https://github.com/robo-friends/m-explore-ros2.git "${upstream_sha}"
git -C "${build_dir}/source" checkout --quiet --detach FETCH_HEAD
test "$(git -C "${build_dir}/source" rev-parse HEAD)" = "${upstream_sha}"

source "${base_prefix}/setup.bash"
export LD_LIBRARY_PATH="${base_prefix}/lib:${LD_LIBRARY_PATH:-}"
for package in explore_lite_msgs explore; do
  "${base_prefix}/bin/cmake" -S "${build_dir}/source/${package}" -B "${build_dir}/${package}" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O2 -DNDEBUG' \
    -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
    -DCMAKE_EXE_LINKER_FLAGS="-L${base_prefix}/lib" \
    -DPython3_EXECUTABLE="${base_prefix}/bin/python" \
    -DCMAKE_INSTALL_PREFIX="${overlay_prefix}" -DBUILD_TESTING=OFF
  nice -n 10 taskset -c 1 "${base_prefix}/bin/cmake" --build "${build_dir}/${package}" --parallel 1
  "${base_prefix}/bin/cmake" --install "${build_dir}/${package}"
  if [[ "${package}" == explore_lite_msgs ]]; then
    source "${overlay_prefix}/share/explore_lite_msgs/local_setup.bash"
  fi
done
printf '%s\n' "${upstream_sha}" > "${overlay_prefix}/upstream_revision"
printf 'Installed explore_lite %s at %s\n' "${upstream_sha}" "${overlay_prefix}"
