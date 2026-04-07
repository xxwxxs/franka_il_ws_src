#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${1:-$HOME/franka_il_ws}"
SRC_DIR="$WS_ROOT/src"
OUT_DIR="$HOME/franka_audit_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR"
REPORT="$OUT_DIR/report.txt"

log() {
  printf '\n===== %s =====\n' "$1" | tee -a "$REPORT"
}

run() {
  printf '\n$ %s\n' "$*" | tee -a "$REPORT"
  bash -lc "$*" 2>&1 | tee -a "$REPORT"
}

printf 'Franka desktop audit\nGenerated at: %s\nWorkspace: %s\n' "$(date -Is)" "$WS_ROOT" > "$REPORT"

log "OS and kernel"
run 'hostnamectl || true'
run 'lsb_release -a || cat /etc/os-release || true'
run 'uname -a'
run 'uname -r'
run 'cat /sys/kernel/realtime 2>/dev/null || echo no_/sys/kernel/realtime'
run 'grep -E "PREEMPT|HZ=" /boot/config-$(uname -r) 2>/dev/null || true'

log "Realtime permissions"
run 'ulimit -r || true'
run 'ulimit -l || true'
run 'getent group realtime || true'
run 'grep -Rns "rtprio|memlock|priority" /etc/security/limits.conf /etc/security/limits.d 2>/dev/null || true'

log "ROS 2 environment"
run 'printenv | grep -E "^(ROS|RMW|AMENT|COLCON|CYCLONEDDS|FASTRTPS)" | sort || true'
run 'which ros2 || true'
run 'ros2 --version || true'
run 'ros2 doctor --report || true'

log "Installed packages"
run 'dpkg -l | grep -E "ros-.*(humble|jazzy|iron|rolling)|libfranka|moveit|rviz|spacenav|spacemouse|3dconnexion" || true'
run 'apt-cache policy libfranka || true'
run 'apt-cache policy ros-humble-desktop ros-humble-moveit ros-humble-rviz2 2>/dev/null || true'
run 'python3 -m pip show pyspacemouse 2>/dev/null || true'

log "USB and SpaceMouse"
run 'lsusb || true'
run 'grep -H . /sys/class/hidraw/hidraw*/device/uevent 2>/dev/null | grep -i SpaceMouse || true'
run 'ls -l /dev/hidraw* 2>/dev/null || true'
run 'udevadm info -q all -n /dev/hidraw0 2>/dev/null | sed -n "1,80p" || true'

log "Workspace overview"
run "test -d '$WS_ROOT' && ls -lah '$WS_ROOT' || echo workspace_not_found"
run "test -d '$SRC_DIR' && find '$SRC_DIR' -maxdepth 2 -type f \( -name package.xml -o -name '*.launch.py' -o -name '*.yaml' -o -name '*.yml' \) | sort | sed -n '1,400p' || true"
run "test -d '$SRC_DIR' && grep -RnsE 'robot_ip|namespace|device_path|operator_position_front|thread_priority|update_rate|use_fake_hardware|gripper|spacemouse' '$SRC_DIR' --include='*.yaml' --include='*.yml' --include='*.launch.py' | sed -n '1,400p' || true"

log "Git state"
if [ -d "$SRC_DIR/.git" ]; then
  run "cd '$SRC_DIR' && git status --short"
  run "cd '$SRC_DIR' && git branch --show-current || true"
  run "cd '$SRC_DIR' && git rev-parse HEAD || true"
  run "cd '$SRC_DIR' && git remote -v || true"
else
  run "find '$SRC_DIR' -maxdepth 3 -type d -name .git | sed 's#/.git##' | sort || true"
  while IFS= read -r repo; do
    [ -z "$repo" ] && continue
    run "cd '$repo' && printf 'REPO=%s\n' '$repo' && git branch --show-current && git rev-parse HEAD && git status --short && git remote -v"
  done < <(find "$SRC_DIR" -maxdepth 3 -type d -name .git | sed 's#/.git##' | sort)
fi

log "VCS export"
run "test -d '$WS_ROOT' && cd '$WS_ROOT' && vcs export --exact src > '$OUT_DIR/exact.repos' || true"

log "Build artifacts"
run "test -d '$WS_ROOT' && ls -lah '$WS_ROOT/build' '$WS_ROOT/install' '$WS_ROOT/log' 2>/dev/null || true"

log "Suggested files to copy"
run "test -d '$SRC_DIR' && find '$SRC_DIR' \( -path '*/config/*' -o -name 'dependency.repos' -o -name 'Dockerfile' -o -name 'docker-compose.yml' -o -name 'limits.conf' -o -name '*.rviz' \) -type f | sort | sed -n '1,400p' || true"

printf '\nAudit complete. Report: %s\n' "$REPORT"
if [ -f "$OUT_DIR/exact.repos" ]; then
  printf 'Exact repos: %s\n' "$OUT_DIR/exact.repos"
fi
