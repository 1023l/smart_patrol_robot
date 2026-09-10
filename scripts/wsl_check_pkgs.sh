#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/inspect/inspect_ws/install/setup.bash
echo "AMENT_PREFIX_PATH:"
echo "$AMENT_PREFIX_PATH" | tr ':' '\n'
echo "--- index files ---"
ls /home/inspect/inspect_ws/install/*/share/ament_index/resource_index/packages/ 2>/dev/null
echo "--- ros2 pkg ---"
ros2 pkg list | grep -E 'inspect_|nav2_gradient|nav2_keepout' || true
python3 <<'PY'
from ament_index_python.packages import get_packages_with_prefixes
p = get_packages_with_prefixes()
keys = sorted(k for k in p if 'inspect' in k or 'gradient' in k or 'keepout' in k)
print('python index:', keys)
for k in keys:
    print(k, '->', p[k])
PY
