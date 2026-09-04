# 地图目录说明
# 本目录下的 floor1~floor5 地图（.pgm + .yaml）由脚本生成：
#   cd <inspect_navigation 包源码目录>
#   python3 scripts/generate_maps.py --output-dir maps/
# 生成后重新 colcon build 即可安装到 share 目录。
#
# 楼层切换由 floor_manager 通过 /map_server/load_map 动态加载本目录的 yaml。
