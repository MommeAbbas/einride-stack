#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 || ! $1 =~ ^[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+$ ]]; then
  echo 'Usage: tools/deploy_source.sh user@host' >&2
  exit 2
fi

target=$1
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
remote_workspace=${AUTONOMY_REMOTE_WORKSPACE:-autonomy_ws}

command -v rsync >/dev/null
command -v ssh >/dev/null

ssh "$target" "mkdir -p \"\$HOME/$remote_workspace/src/einride-stack\""
rsync -az --delete \
  --exclude='.git/' --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='data/' --exclude='results/' --exclude='bags/' --exclude='maps/' \
  --exclude='vendor_src/' --exclude='*.db3' --exclude='*.mcap' --exclude='*.bag' \
  --exclude='*.engine' --exclude='*.onnx' --exclude='.env*' --exclude='*.token' \
  "$project_root/" "$target:$remote_workspace/src/einride-stack/"

cat <<EOF
Source copied to $target:~/$remote_workspace/src/einride-stack
On the target, source its ROS underlay and run:
  cd ~/$remote_workspace/src/einride-stack
  ./tools/fetch_dependencies.sh
  cd ~/$remote_workspace
  export PYTHONNOUSERSITE=1
  colcon build --symlink-install --base-paths src/einride-stack \\
    --packages-up-to interfaces slam planning control \\
      control_docking control_mppi_critic perception explore_lite
EOF
