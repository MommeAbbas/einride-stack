#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
vendor_dir="$project_root/vendor_src"
mkdir -p "$vendor_dir"

if [[ ! -d "$vendor_dir/m-explore-ros2/.git" ]]; then
  git clone https://github.com/robo-friends/m-explore-ros2 "$vendor_dir/m-explore-ros2"
fi

git -C "$vendor_dir/m-explore-ros2" fetch --tags origin
git -C "$vendor_dir/m-explore-ros2" checkout --detach 326cf8a0b487c34246bb8f3326afbcd69576dc60

for patch in "$project_root"/tools/patches/*.patch; do
  if git -C "$vendor_dir/m-explore-ros2" apply --check "$patch" >/dev/null 2>&1; then
    git -C "$vendor_dir/m-explore-ros2" apply "$patch"
  elif git -C "$vendor_dir/m-explore-ros2" apply --reverse --check "$patch" >/dev/null 2>&1; then
    echo "Already applied: $(basename "$patch")"
  else
    echo "Patch does not apply cleanly: $patch" >&2
    exit 1
  fi
done

echo "Dependencies ready in $vendor_dir"
