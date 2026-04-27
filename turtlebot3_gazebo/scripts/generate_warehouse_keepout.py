#!/usr/bin/env python3
"""Generate occupancy maps, keepout masks, episode goals, and previews.

Layouts: AWS Small Warehouse (no-roof) — three keepout-only benchmark variants.
Keepout zones are derived from the physical structures and painted floor markings
visible in the Gazebo world:

  West wall  — ShelfF_01 at (-5.795, -0.957) plus the overhead support structure
               it anchors.  Five horizontal leg strips run from y≈-10 to y≈+8
               with the same x-extent; the entire strip is one connected structure
               → one big keepout rectangle.

  East side  — ShelfD/ShelfE overhang footprints.  The occupancy grid only shows
               legs/edges for these structures, but the whole footprint is blocked
               from the western legs to the east wall.

Outputs written to maps/warehouse{,_v2,_v3}/:
  map.pgm / map.yaml          — static occupancy map (AWS basemap, unmodified)
  keepout_mask.pgm / .yaml    — white background, keepout rects painted black
  goals.yaml                  — versioned episode list (start, goal, distance bucket)
  preview.png                 — red overlay + labelled zones + spawn + goal spots
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from math import cos, sin
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw

# ---------------------------------------------------------------------------
# Map constants — match maps/005/map.yaml
# ---------------------------------------------------------------------------
RES = 0.05          # metres per pixel
ORIGIN = (-7.0, -10.5)  # world coords of bottom-left corner (col=0, row=H-1)
W, H = 286, 423     # image width, height in pixels
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196
FREE_PIXEL_MIN = int(np.floor(255 * (1 - FREE_THRESH))) + 1

AWS_PNG = Path(
    '/home/ruslan/turtlebot3_ws/src/aws-robomaker-small-warehouse-world/'
    'maps/005/map_rotated.png'
)
OUT_ROOT = Path(__file__).parent.parent / 'maps'


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
@dataclass
class Rect:
    cx: float   # world x of centre
    cy: float   # world y of centre
    w: float    # width  (x-extent)
    h: float    # height (y-extent)
    label: str = ''
    yaw: float = 0.0


@dataclass(frozen=True)
class Patrol:
    name: str
    speed: float
    waypoints: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class Benchmark:
    name: str
    title: str
    zones: tuple[Rect, ...]
    spawn: tuple[float, float] = (0.0, 0.0)
    patrols: tuple[Patrol, ...] = ()


def world_to_px(x: float, y: float) -> tuple[int, int]:
    col = int(round((x - ORIGIN[0]) / RES))
    row = int(round(H - (y - ORIGIN[1]) / RES))
    return col, row


def _rect_corners(r: Rect) -> list[tuple[int, int]]:
    dx, dy = r.w / 2, r.h / 2
    c, s = cos(r.yaw), sin(r.yaw)
    corners_world = [
        (r.cx + c * sx - s * sy, r.cy + s * sx + c * sy)
        for sx, sy in [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
    ]
    return [world_to_px(x, y) for x, y in corners_world]


def paint_rect(draw: ImageDraw.ImageDraw, r: Rect, fill: int) -> None:
    draw.polygon(_rect_corners(r), fill=fill)


# ---------------------------------------------------------------------------
# Keepout zone definitions
# ---------------------------------------------------------------------------
# --- West wall ---
# ShelfF_01 at (-5.795, -0.957) anchors an overhead support structure whose
# five floor-level leg strips span:
#   y positions: +7.85, +3.30, -1.20, -5.50, -9.95
#   x extent:    -6.55 to -4.75  (verified from map pixel runs)
# The structure above connects all legs → one keepout rectangle.
WEST_ZONES: list[Rect] = [
    Rect(cx=-5.65, cy=-1.05, w=1.90, h=18.10, label='W0'),
]

# --- East-side shelf overhangs ---
# The center/east ShelfD and ShelfE structures are visible in the occupancy grid
# as western/middle leg strips around x≈2.6 and x≈4.5 plus the eastern support
# near the wall. The full overhang footprint still runs to the east wall, but
# each band should only be as tall as the visible leg rows.
EAST_STRUCTURE_LEFT_X = 2.55
EAST_WALL_X = 6.85
EAST_STRUCTURE_CX = (EAST_STRUCTURE_LEFT_X + EAST_WALL_X) / 2
EAST_STRUCTURE_W = EAST_WALL_X - EAST_STRUCTURE_LEFT_X

EAST_STRUCTURE_ZONES: list[Rect] = [
    Rect(cx=EAST_STRUCTURE_CX, cy=+0.45, w=EAST_STRUCTURE_W, h=0.90, label='C0'),
    Rect(cx=EAST_STRUCTURE_CX, cy=-1.35, w=EAST_STRUCTURE_W, h=0.90, label='C1'),
    Rect(cx=EAST_STRUCTURE_CX, cy=-3.15, w=EAST_STRUCTURE_W, h=0.90, label='C2'),
    Rect(cx=EAST_STRUCTURE_CX, cy=-4.90, w=EAST_STRUCTURE_W, h=0.90, label='C3'),
    Rect(cx=EAST_STRUCTURE_CX, cy=-6.80, w=EAST_STRUCTURE_W, h=0.90, label='C4'),
    Rect(cx=EAST_STRUCTURE_CX, cy=-8.75, w=EAST_STRUCTURE_W, h=0.90, label='C5'),
]

# --- Center painted floor squares ---
# These are explicit painted keepout squares from the warehouse ground-marking
# mesh (GroundB_01 visual). They are not present in the occupancy map, so we
# add them only in the denser second benchmark variant.
CENTER_SQUARE_ZONES: list[Rect] = [
    Rect(cx=-1.05, cy=+5.57, w=3.00, h=3.00, label='S0'),
    Rect(cx=-1.05, cy=+2.35, w=3.00, h=3.00, label='S1'),
    Rect(cx=-1.05, cy=-0.88, w=3.00, h=3.00, label='S2'),
    Rect(cx=-1.05, cy=-4.10, w=3.00, h=3.00, label='S3'),
    Rect(cx=-1.05, cy=-7.32, w=3.00, h=3.00, label='S4'),
]

# KEEP IN SYNC with PEDESTRIAN_PATROLS in launch/nav2_aws_warehouse_v3.launch.py.
# Validated by validate_patrols() — the generator aborts if any path crosses
# the static occupancy or the keepout mask within PED_FOOTPRINT_M of the line.
PED_FOOTPRINT_M = 0.30  # half-width of the cylinder-equivalent footprint
WAREHOUSE_V3_PATROLS: tuple[Patrol, ...] = (
    Patrol(name='ped_east', speed=0.6, waypoints=((1.5, 6.0), (1.5, -6.0))),
    Patrol(name='ped_west', speed=0.5, waypoints=((-3.7, -6.0), (-3.7, 6.0))),
)

BENCHMARKS: tuple[Benchmark, ...] = (
    Benchmark(
        name='warehouse',
        title='warehouse benchmark',
        zones=tuple(WEST_ZONES + EAST_STRUCTURE_ZONES),
        spawn=(0.0, 0.0),
    ),
    Benchmark(
        name='warehouse_v2',
        title='warehouse v2 benchmark',
        zones=tuple(WEST_ZONES + EAST_STRUCTURE_ZONES + CENTER_SQUARE_ZONES),
        spawn=(-3.0, 0.0),
    ),
    Benchmark(
        name='warehouse_v3',
        title='warehouse v3 benchmark (with pedestrians)',
        zones=tuple(WEST_ZONES + EAST_STRUCTURE_ZONES + CENTER_SQUARE_ZONES),
        spawn=(-3.0, 0.0),
        patrols=WAREHOUSE_V3_PATROLS,
    ),
)


# ---------------------------------------------------------------------------
# Map YAML
# ---------------------------------------------------------------------------
_YAML_BODY = (
    'image: {name}\n'
    f'resolution: {RES:.6f}\n'
    f'origin: [{ORIGIN[0]:.3f}, {ORIGIN[1]:.3f}, 0.000000]\n'
    'negate: 0\n'
    f'occupied_thresh: {OCCUPIED_THRESH:.2f}\n'
    f'free_thresh: {FREE_THRESH:.3f}\n'
    'mode: trinary\n'
)


def _save_yaml(path: Path, image_name: str) -> None:
    path.write_text(_YAML_BODY.format(name=image_name))


def _zone_count_summary(zones: list[Rect] | tuple[Rect, ...]) -> str:
    parts = []
    for prefix, label in (('W', 'west'), ('C', 'east structures'), ('S', 'center squares')):
        count = sum(1 for z in zones if z.label.startswith(prefix))
        if count:
            parts.append(f'{count} {label}')
    return ', '.join(parts)


# ---------------------------------------------------------------------------
# Goal-spot generation
# ---------------------------------------------------------------------------
GOAL_CLEARANCE_M = 0.50   # min distance from any obstacle pixel
KEEPOUT_BUFFER_M = 0.40   # extra buffer inside keepout rects
GOAL_GRID_M = 0.80        # goal candidate spacing


def _pt_in_rect(px: float, py: float, r: Rect, buf: float = 0.0) -> bool:
    dx = abs(px - r.cx)
    dy = abs(py - r.cy)
    return dx <= (r.w / 2 + buf) and dy <= (r.h / 2 + buf)


def generate_goal_spots(base_arr: np.ndarray,
                        zones: list[Rect] | tuple[Rect, ...]) -> list[tuple[float, float]]:
    """Return a list of (x, y) world positions valid for goal placement."""
    from scipy.ndimage import binary_dilation

    # Dilate all non-free cells so goals do not land on gray unknown bands or
    # close enough for the robot footprint to overlap shelves/walls.
    nonfree = base_arr < FREE_PIXEL_MIN
    radius_px = int(round(GOAL_CLEARANCE_M / RES))
    struct = np.ones((2 * radius_px + 1,) * 2, dtype=bool)
    blocked = binary_dilation(nonfree, structure=struct)

    goals: list[tuple[float, float]] = []
    step_px = int(round(GOAL_GRID_M / RES))

    for row in range(0, H, step_px):
        for col in range(0, W, step_px):
            if row < radius_px or row >= H - radius_px:
                continue
            if col < radius_px or col >= W - radius_px:
                continue
            if blocked[row, col]:
                continue
            wx = ORIGIN[0] + (col + 0.5) * RES
            wy = ORIGIN[1] + (H - row - 0.5) * RES
            if any(_pt_in_rect(wx, wy, z, KEEPOUT_BUFFER_M) for z in zones):
                continue
            goals.append((wx, wy))

    return goals


# ---------------------------------------------------------------------------
# Episode generation
# ---------------------------------------------------------------------------
# Schema is versioned so harnesses can refuse to run against a stale file.
GOALS_SCHEMA_VERSION = 1
EPISODE_SEED = 42
N_PER_BUCKET = 17                 # 17 short + 17 medium + 17 long = 51 episodes
DIST_BUCKETS = ((0.0, 3.0, 'short'),
                (3.0, 7.0, 'medium'),
                (7.0, float('inf'), 'long'))


def _bucket_of(dist: float) -> str:
    for lo, hi, name in DIST_BUCKETS:
        if lo <= dist < hi:
            return name
    return DIST_BUCKETS[-1][2]


def generate_episodes(goals: list[tuple[float, float]],
                      spawn: tuple[float, float],
                      seed: int = EPISODE_SEED,
                      n_per_bucket: int = N_PER_BUCKET) -> list[dict]:
    """Sample N episodes per distance bucket with a fixed seed.

    Start is fixed to spawn (matches Habitat-style episodic eval). Goal yaw is
    set so that facing the goal from spawn lines up with the episode geometry.
    """
    rng = np.random.default_rng(seed)
    sx, sy = spawn

    by_bucket: dict[str, list[tuple[float, tuple[float, float]]]] = {
        name: [] for _, _, name in DIST_BUCKETS
    }
    for gx, gy in goals:
        d = float(np.hypot(gx - sx, gy - sy))
        if d < 0.5:                    # too close to spawn — skip
            continue
        by_bucket[_bucket_of(d)].append((d, (gx, gy)))

    episodes: list[dict] = []
    ep_idx = 0
    for _, _, name in DIST_BUCKETS:
        candidates = by_bucket[name]
        if not candidates:
            continue
        idxs = rng.permutation(len(candidates))[:n_per_bucket]
        for i in idxs:
            d, (gx, gy) = candidates[i]
            yaw_to_goal = float(np.arctan2(gy - sy, gx - sx))
            episodes.append({
                'id': f'ep_{ep_idx:03d}',
                'start': [round(sx, 3), round(sy, 3), 0.0],
                'goal': [round(gx, 3), round(gy, 3), round(yaw_to_goal, 4)],
                'distance_m': round(d, 3),
                'bucket': name,
            })
            ep_idx += 1
    return episodes


def validate_patrols(base_arr: np.ndarray, ko_arr: np.ndarray,
                     patrols: tuple[Patrol, ...]) -> list[str]:
    """Return a list of error strings if any patrol crosses occupancy/keepout.

    Sampled along each segment plus the closing segment back to waypoint 0,
    with a square footprint of ±PED_FOOTPRINT_M around the centerline.
    """
    errs: list[str] = []
    if not patrols:
        return errs

    samples_per_m = 8
    halfpx = max(1, int(round(PED_FOOTPRINT_M / RES)))

    for p in patrols:
        wpts = list(p.waypoints) + [p.waypoints[0]]
        bad_obst = bad_keep = total = 0
        for a, b in zip(wpts, wpts[1:]):
            seg_len = float(np.hypot(b[0] - a[0], b[1] - a[1]))
            n = max(2, int(round(seg_len * samples_per_m)))
            for t in range(n):
                f = t / (n - 1)
                wx = a[0] + (b[0] - a[0]) * f
                wy = a[1] + (b[1] - a[1]) * f
                col, row = world_to_px(wx, wy)
                r0, r1 = max(0, row - halfpx), min(H, row + halfpx + 1)
                c0, c1 = max(0, col - halfpx), min(W, col + halfpx + 1)
                patch_base = base_arr[r0:r1, c0:c1]
                patch_ko = ko_arr[r0:r1, c0:c1]
                if (patch_base < 200).any():
                    bad_obst += 1
                if (patch_ko < 128).any():
                    bad_keep += 1
                total += 1
        if bad_obst or bad_keep:
            errs.append(
                f'patrol {p.name!r}: {bad_obst}/{total} samples cross occupancy, '
                f'{bad_keep}/{total} cross keepout')
    return errs


def write_goals_yaml(path: Path,
                     benchmark: 'Benchmark',
                     episodes: list[dict]) -> None:
    """Hand-formatted YAML so diffs stay readable across regenerations."""
    lines: list[str] = [
        '# Auto-generated by generate_warehouse_keepout.py — do not edit by hand.',
        '# Re-run the script to regenerate after changing keepout zones or seed.',
        f'schema_version: {GOALS_SCHEMA_VERSION}',
        f'benchmark: {benchmark.name}',
        f'seed: {EPISODE_SEED}',
        f'spawn: [{benchmark.spawn[0]:.3f}, {benchmark.spawn[1]:.3f}, 0.0]',
        f'clearance_m: {GOAL_CLEARANCE_M:.2f}',
        f'keepout_buffer_m: {KEEPOUT_BUFFER_M:.2f}',
        f'n_per_bucket: {N_PER_BUCKET}',
        'buckets:',
    ]
    for lo, hi, name in DIST_BUCKETS:
        hi_repr = '.inf' if hi == float('inf') else f'{hi:.1f}'
        lines.append(f'  - {{name: {name}, min_m: {lo:.1f}, max_m: {hi_repr}}}')
    lines.append('episodes:')
    for ep in episodes:
        gs = ', '.join(f'{v:.3f}' for v in ep['start'])
        gg = ', '.join(f'{v:.4f}' for v in ep['goal'])
        lines.append(f'  - {{id: {ep["id"]}, bucket: {ep["bucket"]}, '
                     f'distance_m: {ep["distance_m"]:.3f}, '
                     f'start: [{gs}], goal: [{gg}]}}')
    path.write_text('\n'.join(lines) + '\n')


# ---------------------------------------------------------------------------
# Preview renderer
# ---------------------------------------------------------------------------
EXTENT = (ORIGIN[0], ORIGIN[0] + W * RES, ORIGIN[1], ORIGIN[1] + H * RES)

ZONE_COLORS = {
    'W': '#8B0000',   # dark red — west wall
    'C': '#B22222',   # firebrick — east shelf overhangs
    'S': '#C67C00',   # amber — painted center squares
}

ZONE_LEGENDS = {
    'W': 'west zone (W0)',
    'C': 'east structures (C0-C5)',
    'S': 'center squares (S0-S4)',
}


def _zone_color(r: Rect) -> str:
    return ZONE_COLORS.get(r.label[0] if r.label else 'C', '#CC0000')


def render_preview(base_arr: np.ndarray, ko_arr: np.ndarray,
                   goals: list[tuple[float, float]],
                   zones: list[Rect] | tuple[Rect, ...],
                   spawn: tuple[float, float],
                   title: str,
                   out_path: Path,
                   patrols: tuple[Patrol, ...] = ()) -> None:
    fig, ax = plt.subplots(figsize=(7, 10), dpi=130)

    ax.imshow(base_arr, cmap='gray', vmin=0, vmax=255, extent=EXTENT,
              origin='upper', interpolation='nearest')

    # Red overlay on keepout cells
    overlay = np.zeros((*ko_arr.shape, 4), dtype=np.float32)
    overlay[ko_arr < 128] = (1.0, 0.15, 0.15, 0.45)
    ax.imshow(overlay, extent=EXTENT, origin='upper', interpolation='nearest')

    # Zone outlines + labels
    for r in zones:
        col = _zone_color(r)
        corners_w = []
        dx, dy = r.w / 2, r.h / 2
        c, s = cos(r.yaw), sin(r.yaw)
        for sx, sy in [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]:
            corners_w.append((r.cx + c * sx - s * sy, r.cy + s * sx + c * sy))
        poly = mpatches.Polygon(
            corners_w, closed=True, linewidth=1.5,
            edgecolor=col, facecolor='none', linestyle='--',
        )
        ax.add_patch(poly)
        ax.text(r.cx, r.cy, r.label, color=col, ha='center', va='center',
                fontsize=8, fontweight='bold')

    # Spawn
    ax.plot(spawn[0], spawn[1], marker='o', color='limegreen', markersize=10,
            markeredgecolor='black', zorder=5,
            label=f'spawn ({spawn[0]:.1f},{spawn[1]:.1f})')

    # Goal spots
    if goals:
        gx, gy = zip(*goals)
        ax.scatter(gx, gy, marker='x', color='dodgerblue', s=40,
                   linewidths=1.2, zorder=4, label=f'goal spots ({len(goals)})')

    # Pedestrian patrols (closed loops, with current-direction arrow)
    for p in patrols:
        wx = [w[0] for w in p.waypoints] + [p.waypoints[0][0]]
        wy = [w[1] for w in p.waypoints] + [p.waypoints[0][1]]
        ax.plot(wx, wy, color='#7B2CBF', linewidth=1.6, linestyle='-',
                marker='s', markersize=4, zorder=6)
        ax.text(p.waypoints[0][0], p.waypoints[0][1] + 0.35, p.name,
                color='#7B2CBF', fontsize=7, ha='center', va='bottom')

    # Legend proxies
    legend_patches = [
        mpatches.Patch(facecolor=ZONE_COLORS[prefix], alpha=0.6,
                       label=ZONE_LEGENDS[prefix])
        for prefix in ('W', 'C', 'S')
        if any(r.label.startswith(prefix) for r in zones)
    ]
    legend_handles = [
        *legend_patches,
        plt.Line2D([0], [0], marker='o', color='limegreen', markersize=8,
                   markeredgecolor='black', linestyle='', label='spawn'),
        plt.Line2D([0], [0], marker='x', color='dodgerblue', markersize=6,
                   linewidth=1.2, linestyle='', label=f'goal spots ({len(goals)})'),
    ]
    if patrols:
        legend_handles.append(
            plt.Line2D([0], [0], color='#7B2CBF', marker='s', markersize=4,
                       linewidth=1.6, label=f'pedestrian patrols ({len(patrols)})')
        )
    ax.legend(handles=legend_handles, loc='upper left', fontsize=7, framealpha=0.85)

    ax.set_title(f'{title} — keepout zones + goal spots', fontsize=11)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_xlim(EXTENT[0], EXTENT[1])
    ax.set_ylim(EXTENT[2], EXTENT[3])
    ax.grid(True, linestyle=':', linewidth=0.4, alpha=0.5)

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def generate_benchmark(base: Image.Image, benchmark: Benchmark) -> None:
    out_dir = OUT_ROOT / benchmark.name
    out_dir.mkdir(parents=True, exist_ok=True)

    # Static map — unchanged copy of AWS basemap
    base.save(out_dir / 'map.pgm')
    _save_yaml(out_dir / 'map.yaml', 'map.pgm')

    # Keepout mask — white background, zones painted black
    ko = Image.new('L', (W, H), color=254)
    ko_draw = ImageDraw.Draw(ko)
    for r in benchmark.zones:
        paint_rect(ko_draw, r, fill=0)
    ko.save(out_dir / 'keepout_mask.pgm')
    _save_yaml(out_dir / 'keepout_mask.yaml', 'keepout_mask.pgm')

    print(f'[{benchmark.name}] map + keepout mask → {out_dir}')

    # Preview
    base_arr = np.array(base)
    ko_arr = np.array(ko)
    goals = generate_goal_spots(base_arr, benchmark.zones)

    patrol_errs = validate_patrols(base_arr, ko_arr, benchmark.patrols)
    if patrol_errs:
        sys.exit('[' + benchmark.name + '] patrol validation FAILED:\n  '
                 + '\n  '.join(patrol_errs))

    render_preview(base_arr, ko_arr, goals, benchmark.zones, benchmark.spawn,
                   benchmark.title,
                   out_dir / 'preview.png',
                   patrols=benchmark.patrols)
    print(f'[{benchmark.name}] preview → {out_dir / "preview.png"}  ({len(goals)} goal spots)')

    # Versioned episode list — fixed seed → byte-stable across regenerations.
    episodes = generate_episodes(goals, benchmark.spawn)
    write_goals_yaml(out_dir / 'goals.yaml', benchmark, episodes)
    by_bucket = {b: sum(1 for e in episodes if e['bucket'] == b)
                 for _, _, b in DIST_BUCKETS}
    print(f'[{benchmark.name}] goals.yaml → {len(episodes)} episodes ('
          + ', '.join(f'{n} {b}' for b, n in by_bucket.items()) + ')')
    print(f'[{benchmark.name}] zones: {_zone_count_summary(benchmark.zones)}')
    print(f'[{benchmark.name}] recommended spawn: ({benchmark.spawn[0]:.2f}, '
          f'{benchmark.spawn[1]:.2f})')


def main() -> None:
    if not AWS_PNG.exists():
        sys.exit(f'ERROR: base map not found at {AWS_PNG}')

    base = Image.open(AWS_PNG).convert('L')
    if base.size != (W, H):
        sys.exit(f'ERROR: unexpected base map size {base.size}, expected ({W}, {H})')

    for benchmark in BENCHMARKS:
        generate_benchmark(base, benchmark)


if __name__ == '__main__':
    main()
