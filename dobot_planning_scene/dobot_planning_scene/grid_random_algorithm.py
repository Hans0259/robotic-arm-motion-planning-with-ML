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
    num_spheres=10,
    random_seed=None,
    bounds=((-1.0, 1.0), (-1.0, 1.0), (0.3, 1.3)),
    grid_shape=(4, 4, 4),
    radius_range=(0.05, 0.2),
    min_gap=0.05,
    exclusion_boxes=(),
    candidate_validator=None,
    max_attempts_per_cell=100,
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
    def check_box(box):
        if len(box) != 3 or any(
            len(axis) != 2
            or not all(math.isfinite(value) for value in axis)
            or axis[0] >= axis[1]
            for axis in box
        ):
            raise ValueError('Boxes must contain three finite (lower, upper) pairs')

    check_box(bounds)
    exclusion_boxes = tuple(exclusion_boxes)
    for box in exclusion_boxes:
        check_box(box)
    if len(grid_shape) != 3 or any(
        not isinstance(count, int) or isinstance(count, bool) or count <= 0
        for count in grid_shape
    ):
        raise ValueError('grid_shape must contain three positive integers')
    if (not isinstance(num_spheres, int) or isinstance(num_spheres, bool)
            or not 0 <= num_spheres <= math.prod(grid_shape)):
        raise ValueError('num_spheres must be an integer within grid capacity')
    if (len(radius_range) != 2
            or not all(math.isfinite(value) for value in radius_range)
            or not 0 < radius_range[0] <= radius_range[1]):
        raise ValueError('radius_range must contain positive ordered radii')
    if not math.isfinite(min_gap) or min_gap < 0:
        raise ValueError('min_gap must be finite and nonnegative')
    if (not isinstance(max_attempts_per_cell, int)
            or isinstance(max_attempts_per_cell, bool)
            or max_attempts_per_cell <= 0):
        raise ValueError('max_attempts_per_cell must be a positive integer')
    if num_spheres == 0:
        return []

    rng = random.Random(random_seed)
    cells = list(itertools.product(*(range(count) for count in grid_shape)))
    rng.shuffle(cells)
    widths = [(hi - lo) / count for (lo, hi), count in zip(bounds, grid_shape)]
    spheres = []
    accepted = []

    for cell in cells:
        for _ in range(max_attempts_per_cell):
            centre = tuple(
                rng.uniform(lo + index * width, lo + (index + 1) * width)
                for (lo, _), index, width in zip(bounds, cell, widths)
            )
            radius = rng.uniform(*radius_range)
            if any(math.dist(centre, other) <= radius + other_radius + min_gap
                   for other, other_radius in accepted):
                continue
            # Distance from the sphere centre to the closest point on each box.
            if any(
                sum(max(lo - value, 0.0, value - hi) ** 2
                    for value, (lo, hi) in zip(centre, box))
                <= (radius + min_gap) ** 2
                for box in exclusion_boxes
            ):
                continue
            sphere = generate_spheres(radius, *centre, f'sphere_{len(spheres)}')
            if candidate_validator is not None and not candidate_validator(sphere):
                continue
            spheres.append(sphere)
            accepted.append((centre, radius))
            if len(spheres) == num_spheres:
                return spheres
            break

    raise RuntimeError(
        f'Placed only {len(spheres)}/{num_spheres} spheres. '
        'Increase space or attempts, or reduce radii, gap, or obstacle count.'
    )
