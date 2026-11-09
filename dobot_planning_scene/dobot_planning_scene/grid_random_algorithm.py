"""Generate separated spherical obstacles in randomly selected grid cells.

This module only builds CollisionObject messages; it does not change a live
Planning Scene. All bounds are expressed in metres in base_link. The workspace
bounds constrain sphere centres, not their entire volumes.
"""

import itertools
import math
import random

from dobot_planning_scene.spheres_generator import generate_spheres


def generate_grid_spheres(
    num_spheres = 15,
    random_seed = None,
    bounds = ((-0.8, 0.8), (-0.8, 0.8), (0.3, 1.3)),
    grid_shape = (3, 3, 3),
    radius_range = (0.05, 0.2),
    min_gap = 0.05,
    exclusion_boxes = (),
    candidate_validator = None,
    max_attempts = 100,
):
    """Return spheres sampled from distinct cells, or raise on insufficient space.

    ``exclusion_boxes`` contains axis-aligned boxes in the same format as
    ``bounds``. Spheres must clear each box by ``min_gap``. Supply conservative
    boxes for the fixed base, ground, or other static structures; none are
    assumed by default. This is a box approximation, not a URDF mesh check.

    Optional ``candidate_validator(CollisionObject) -> bool`` can perform a more
    precise static-geometry check. It should not modify the live scene or reject
    candidates based on an arbitrary movable-arm posture. Full-arm and path
    collision checks remain the responsibility of the dataset collector.

    A local RNG preserves reproducibility without changing global random state.
    A failed cell is skipped; partial results are never returned.
    """

    rng = random.Random(random_seed)            # 创建独立的随机数生成器

    # 生成所有网格单元的索引
    cells = []
    for x in range(grid_shape[0]):
        for y in range(grid_shape[1]):
            for z in range(grid_shape[2]):
                if (x, y, z) == (1, 1, 0):      # 忽略base_link的网格单元，防止球体生成在机器人底座位置
                    continue
                cells.append((x, y, z))

    rng.shuffle(cells)

    # 计算每个网格单元的宽度
    x_width = (bounds[0][1] - bounds[0][0]) / grid_shape[0]
    y_width = (bounds[1][1] - bounds[1][0]) / grid_shape[1]
    z_width = (bounds[2][1] - bounds[2][0]) / grid_shape[2]
    widths = (x_width, y_width, z_width)

    spheres = []
    accepted = []

    
    for cell in cells:
        for _ in range(max_attempts):
            
            # 计算网格单元的中心点坐标
            x_min = bounds[0][0] + cell[0] * widths[0]
            y_min = bounds[1][0] + cell[1] * widths[1]
            z_min = bounds[2][0] + cell[2] * widths[2]
            centre = (
                rng.uniform(x_min, x_min + widths[0]),
                rng.uniform(y_min, y_min + widths[1]),
                rng.uniform(z_min, z_min + widths[2]),
            )

            radius = rng.uniform(radius_range[0], radius_range[1])

            # 检查新球体是否与其他的球体重叠
            if any(math.dist(centre, other) <= radius + other_radius + min_gap
                   for other, other_radius in accepted):
                continue

            # # 检查新球体是否与排除的盒子重叠
            # if any(
            #     sum(max(lo - value, 0.0, value - hi) ** 2
            #         for value, (lo, hi) in zip(centre, box))
            #     <= (radius + min_gap) ** 2
            #     for box in exclusion_boxes
            # ):
            #     continue

            sphere = generate_spheres(radius, *centre, f'sphere_{len(spheres)}')

            spheres.append(sphere)
            accepted.append((centre, radius))
            if len(spheres) == num_spheres:
                return spheres
            break