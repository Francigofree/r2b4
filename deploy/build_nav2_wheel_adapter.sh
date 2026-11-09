#!/usr/bin/env bash
set -eo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
base_prefix="${repo_dir}/runtime/deps/nav2-planner"
overlay_prefix="${repo_dir}/runtime/deps/nav2-wheel-adapter"
build_dir=$(mktemp -d "${repo_dir}/tmp/nav2_wheel_build.XXXXXX")
source "${base_prefix}/setup.bash"
"${base_prefix}/bin/cmake" -S "${repo_dir}/deploy/nav2_wheel_adapter" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O2 -DNDEBUG' \
  -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DPython3_EXECUTABLE="${base_prefix}/bin/python" -DR2B4_NAV2_PREFIX="${base_prefix}" \
  -DCMAKE_INSTALL_PREFIX="${overlay_prefix}"
nice -n 10 taskset -c 1 "${base_prefix}/bin/cmake" --build "${build_dir}" --parallel 1
"${base_prefix}/bin/cmake" --install "${build_dir}"
