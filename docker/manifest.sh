#!/bin/bash
# Prints the environment as "section<TAB>name<TAB>version" lines; run on host and in the container, then diff.
set +u
WS="${FXTESO_WS:-$HOME/Robotics/fxteso_ibvs}"
source /opt/ros/jazzy/setup.bash 2>/dev/null

dpkg-query -W -f='${db:Status-Abbrev}\t${Package}\t${Version}\n' 2>/dev/null \
  | awk -F'\t' '$1 ~ /^ii/ {print "dpkg\t" $2 "\t" $3}'

python3 - <<'EOF'
import importlib.metadata as m
seen = {}
for d in m.distributions():
    name = (d.metadata['Name'] or '').lower().replace('_', '-')
    if name and name not in seen:
        seen[name] = d.version
for n in sorted(seen):
    print(f"python\t{n}\t{seen[n]}")
EOF

echo -e "tool\tpython3\t$(command -v python3) $(python3 --version 2>&1)"
echo -e "tool\tgz-sim\t$(command -v gz) $(gz sim --version 2>/dev/null | head -1)"
echo -e "tool\tarm-none-eabi-gcc\t$(arm-none-eabi-gcc -dumpversion 2>/dev/null)"
echo -e "tool\tcmake\t$(cmake --version | head -1)"
echo -e "tool\topencv(/usr/local)\t$(PKG_CONFIG_PATH= /usr/local/bin/opencv_version 2>/dev/null)"
echo -e "tool\tlibopencv_core(ldconfig first)\t$(ldconfig -p | grep -m1 'libopencv_core.so ' | awk '{print $NF}')"
for mod in gz.transport13 gz.msgs10 pymavlink pyulog cv2 numpy rclpy; do
  echo -e "pyimport\t${mod}\t$(python3 -c "import ${mod} as x; print(getattr(x,'__version__',''), x.__file__)" 2>&1 | tail -1)"
done

for repo in external/PX4-Autopilot external/Micro-XRCE-DDS-Agent src/px4_msgs src/vision_opencv; do
  echo -e "git\t${repo}\t$(git -C "$WS/$repo" rev-parse HEAD 2>/dev/null)"
done
