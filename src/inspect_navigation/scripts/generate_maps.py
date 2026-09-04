#!/usr/bin/env python3
"""
工业巡检厂房 5 层地图生成器
----------------------------------
程序化生成 SLAM 标准格式地图（.pgm + .yaml）：

每层差异（模拟真实厂房：各层布局不同、电梯位置一致保证跨层可达）：
  floor1  标准层：主通道 + 两侧货架区 + 玻璃墙（激光穿透场景源）
  floor2  货架加密层：窄通道场景源（通道宽 ~1.2m）
  floor3  开阔设备层：中央大型设备 + 四角巡检点
  floor4  密集隔间层：办公隔间矩阵（人流密集场景源）
  floor5  顶层简配层：仓储开阔区

图纸要素：
  - 外墙 20m x 20m，墙体厚度 0.4m
  - 电梯厅位于北侧中部（9~11m, 17~19m），与 keepout 电梯井禁区一致
  - 通道宽度按机器人直径 0.44m + 双侧安全裕量设计
  - 玻璃墙区域在地图上标记为自由空间（真实场景激光穿不回来 → 栅格空洞，
    由 KeepOutZone 层补齐虚拟障碍，这正是该层存在的原因）
  - 每层 yaml 的 origin 一致，保证跨层 AMCL 重定位时坐标语义统一

用法：
  python3 generate_maps.py --output-dir <maps目录> [--floors 1 2 3 4 5]
"""

import argparse
import os

import numpy as np

# 地图参数（与 nav2_params / keepout_zones 配置对齐）
MAP_SIZE_M = 20.0          # 地图边长（米）
RESOLUTION = 0.05          # 栅格分辨率（米/格）
WALL_THICK = 0.4           # 墙体厚度（米）
ROBOT_RADIUS = 0.22        # 机器人半径（用于通道宽度校验）

# 像素值（SLAM 标准：255=自由, 0=占据, 205=未知）
FREE = 255
OCCUPIED = 0
UNKNOWN = 205


def meters_to_cells(m):
    return int(round(m / RESOLUTION))


def fill_rect(grid, x0, y0, x1, y1, value):
    """世界坐标矩形填充（米），栅格中心点判定"""
    i0 = max(0, meters_to_cells(x0))
    i1 = min(grid.shape[1] - 1, meters_to_cells(x1) - 1)
    j0 = max(0, meters_to_cells(y0))
    j1 = min(grid.shape[0] - 1, meters_to_cells(y1) - 1)
    if i1 >= i0 and j1 >= j0:
        grid[j0:j1 + 1, i0:i1 + 1] = value


def fill_wall(grid, x0, y0, x1, y1):
    """墙体（占据）"""
    fill_rect(grid, x0, y0, x1, y1, OCCUPIED)


def build_common_structure(grid):
    """各层公共结构：外墙 + 电梯厅"""
    t = WALL_THICK
    s = MAP_SIZE_M
    # 外墙
    fill_wall(grid, 0, 0, s, t)                # 南墙
    fill_wall(grid, 0, s - t, s, s)            # 北墙
    fill_wall(grid, 0, 0, t, s)                # 西墙
    fill_wall(grid, s - t, 0, s, s)            # 东墙
    # 电梯厅（北侧中部 9~11m）：三面围合，南侧开口（机器人进出通道）
    fill_wall(grid, 9.0, 17.0, 9.4, 19.0)      # 左壁
    fill_wall(grid, 10.6, 17.0, 11.0, 19.0)    # 右壁
    # 电梯井本体由 keepout 层禁区实现（地图上保持自由，模拟真实空旷）


def build_floor1(grid):
    """标准层：主通道 + 两侧货架 + 东侧玻璃墙（激光穿透场景源）"""
    # 中央十字主通道两侧的货架区（2m 深货架）
    # 西侧货架：三段，段间留 1.2m 窄通道
    for y0, y1 in [(3.0, 5.5), (6.7, 9.2), (10.4, 12.9)]:
        fill_wall(grid, 2.0, y0, 3.6, y1)
    # 东侧货架（对称）
    for y0, y1 in [(3.0, 5.5), (6.7, 9.2), (10.4, 12.9)]:
        fill_wall(grid, 16.4, y0, 18.0, y1)
    # 东侧玻璃墙：地图上为自由空间！真实激光穿透导致此处栅格空洞，
    # 由 KeepOutZone 层的 glass_wall_east 禁区补齐虚拟障碍
    # fill_wall(grid, 18.6, 2.0, 19.0, 14.0)  # 有意不画


def build_floor2(grid):
    """货架加密层：窄通道场景源（通道宽 ~1.2m）"""
    # 密集货架矩阵：8 列 x 4 行，通道 1.2m
    for col in range(4):
        x = 3.0 + col * 3.8
        for row in range(3):
            y = 4.0 + row * 3.8
            fill_wall(grid, x, y, x + 2.4, y + 2.6)


def build_floor3(grid):
    """开阔设备层：中央大型设备 + 四角巡检点"""
    # 中央设备群
    fill_wall(grid, 8.0, 8.0, 12.0, 12.0)
    # 四角辅助设备
    fill_wall(grid, 3.0, 3.0, 5.0, 4.5)
    fill_wall(grid, 15.0, 3.0, 17.0, 4.5)
    fill_wall(grid, 3.0, 15.5, 5.0, 17.0)
    fill_wall(grid, 15.0, 15.5, 17.0, 17.0)


def build_floor4(grid):
    """密集隔间层：办公隔间矩阵（人流密集场景源）"""
    for col in range(5):
        x = 2.5 + col * 3.5
        for row in range(4):
            y = 2.5 + row * 4.0
            # 隔间：三面矮隔断（连通性好，行人多路径穿越）
            fill_wall(grid, x, y, x + 2.0, y + 0.2)
            fill_wall(grid, x, y, x + 0.2, y + 2.8)
            fill_wall(grid, x + 1.8, y, x + 2.0, y + 2.8)


def build_floor5(grid):
    """顶层简配层：仓储开阔区"""
    fill_wall(grid, 2.0, 9.0, 6.0, 11.0)
    fill_wall(grid, 14.0, 9.0, 18.0, 11.0)


FLOOR_BUILDERS = {
    1: build_floor1,
    2: build_floor2,
    3: build_floor3,
    4: build_floor4,
    5: build_floor5,
}


def write_pgm(path, grid):
    """写 P5 二进制 PGM（SLAM 标准格式）"""
    h, w = grid.shape
    # PGM 图像坐标系 y 向下，地图世界系 y 向上 → 上下翻转
    flipped = np.flipud(grid)
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'# CREATOR: inspect_navigation map generator\n'.encode())
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(flipped.tobytes())


def write_yaml(path, floor, pgm_name):
    """地图元数据（origin 对齐 keepout 禁区与巡检点坐标系）"""
    content = f"""image: {pgm_name}
resolution: {RESOLUTION}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# 楼层: {floor}
# 电梯口（AMCL 重定位点）: x=9.8, y=16.5
"""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)


def main():
    parser = argparse.ArgumentParser(description='工业巡检厂房 5 层地图生成器')
    parser.add_argument('--output-dir', required=True, help='输出目录（maps/）')
    parser.add_argument('--floors', nargs='+', type=int, default=[1, 2, 3, 4, 5],
                        help='要生成的楼层（默认 1 2 3 4 5）')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    size_cells = meters_to_cells(MAP_SIZE_M)

    for floor in args.floors:
        if floor not in FLOOR_BUILDERS:
            print(f'跳过未知楼层 {floor}')
            continue
        # 初始化为自由空间（厂房内部已知空旷），外围未知
        grid = np.full((size_cells, size_cells), FREE, dtype=np.uint8)
        # 未探明区域留 UNKNOWN 更真实，但 SLAM 已建图区域应为 FREE：
        # 这里全部按已建图处理（save_map 产物语义）
        build_common_structure(grid)
        FLOOR_BUILDERS[floor](grid)

        pgm_name = f'floor{floor}.pgm'
        pgm_path = os.path.join(args.output_dir, pgm_name)
        yaml_path = os.path.join(args.output_dir, f'floor{floor}.yaml')
        write_pgm(pgm_path, grid)
        write_yaml(yaml_path, floor, pgm_name)
        occ = np.count_nonzero(grid == OCCUPIED)
        print(f'floor{floor}: {pgm_name} + yaml 已生成（占据栅格 {occ} 个）')

    print(f'\n完成。输出目录: {args.output_dir}')


if __name__ == '__main__':
    main()
