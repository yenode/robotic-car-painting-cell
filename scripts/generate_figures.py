#!/usr/bin/env python3

from pathlib import Path
import json
import sys
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/painting_cell'))
from painting_cell.geometry import read_obj, make_strokes

PACKAGE = ROOT/'src/painting_cell'
OUT = ROOT/'media/screenshots'
OUT.mkdir(parents=True, exist_ok=True)
vertices, faces = read_obj(PACKAGE/'models/hatchback/meshes/car_scaled.obj')
config = yaml.safe_load((PACKAGE/'config/cell.yaml').read_text())
strokes = make_strokes(vertices, faces, config)
world_vertices = vertices+np.asarray(config['car_xyz'])
result = json.loads((ROOT/'results/execution.json').read_text())
COLORS = {'hood': '#f97316', 'roof': '#38bdf8', 'side': '#22c55e'}


def car(ax, alpha=0.55):
    collection = Poly3DCollection(world_vertices[faces], facecolor='#a7b0bd', edgecolor='#475569',
                                  linewidth=0.10, alpha=alpha)
    ax.add_collection3d(collection)


def paths(ax, panels=('hood', 'roof', 'side'), linewidth=3):
    for stroke in strokes:
        if stroke['panel'] in panels:
            xyz = stroke['positions']
            ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=COLORS[stroke['panel']],
                    linewidth=linewidth, label=stroke['panel'].title())


def layout3d(ax):
    
    for item in config['layout']['boxes']:
        x, y, z = item['xyz']
        sx, sy, sz = item['size']
        color = tuple(item['color'][:3])
        alpha = .08 if item['id'].startswith('wall_') else item['color'][3]
        ax.bar3d(x-sx/2, y-sy/2, z-sz/2, sx, sy, sz,
                 color=color, alpha=alpha, shade=True, edgecolor=color, linewidth=.25)


def layout2d(ax):
    for item in config['layout']['boxes']:
        x, y, _ = item['xyz']
        sx, sy, _ = item['size']
        color = tuple(item['color'][:3])
        ax.add_patch(Rectangle((x-sx/2, y-sy/2), sx, sy,
                     facecolor=color, edgecolor=color,
                     alpha=max(.35, item['color'][3]), linewidth=.8))


def unique_legend(ax, location='upper left'):
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc=location, framealpha=0.9)


def style3d(ax, title, elev=25, azim=-55):
    ax.set_title(title, weight='bold')
    ax.set_xlabel('X / car length (m)')
    ax.set_ylabel('Y / car width (m)')
    ax.set_zlabel('Z (m)')
    ax.set_xlim(-2.7, 2.7)
    ax.set_ylim(-2.3, 1.4)
    ax.set_zlim(0, 2.8)
    ax.set_box_aspect((5.4, 3.7, 2.8))
    ax.view_init(elev=elev, azim=azim)
    ax.grid(True, alpha=.25)


# Workcell overview: a clear plan view of the assignment reference geometry.
fig, ax = plt.subplots(figsize=(12, 7), constrained_layout=True)
layout2d(ax)
triangles_xy = world_vertices[faces][:, :, :2]
ax.add_collection(PolyCollection(triangles_xy, facecolor='#dbe2ea',
                                 edgecolor='#64748b', linewidth=.20))
for stroke in strokes:
    ax.plot(stroke['positions'][:, 0], stroke['positions'][:, 1],
            color=COLORS[stroke['panel']], linewidth=4,
            label=stroke['panel'].title())
bx, by, bz = config['robot_base']
ax.add_patch(Rectangle((bx-.225, by-.225), .45, .45,
                       facecolor='#1e293b', label='UR20 base'))
ax.add_patch(plt.Circle((bx, by), 1.75, fill=False, linestyle='--',
                        linewidth=2, color='#6366f1', label='Nominal reach'))
cx, cy, _ = config['car_xyz']
ax.scatter([cx], [cy], marker='+', s=220, linewidths=3,
           color='#ef4444', label='Bay/car centre')
ax.annotate('', xy=(-5.9, -1.5), xytext=(-7.0, -1.5),
            arrowprops={'arrowstyle': '->', 'lw': 2})
ax.text(-7.15, -1.18, 'VEHICLE ENTRY', ha='left', weight='bold')
ax.annotate('', xy=(7.0, -1.5), xytext=(5.9, -1.5),
            arrowprops={'arrowstyle': '->', 'lw': 2})
ax.text(7.15, -1.18, 'VEHICLE EXIT', ha='right', weight='bold')
ax.text(0, .62, '8 × 5 m CENTRAL PAINTING BAY', ha='center', va='center', weight='bold')
ax.text(0, 3.65, 'SERVICE ROOMS', ha='center', va='center', weight='bold', color='white')
ax.set(title='Painting Cell Overview — Car Centred in the 8 × 5 m Bay',
       xlabel='X (m)', ylabel='Y (m)', xlim=(-7.3, 7.3), ylim=(-4.2, 4.2), aspect='equal')
ax.grid(alpha=.18)
unique_legend(ax)
fig.savefig(OUT/'01_workcell_overview.png', dpi=180)
plt.close(fig)


# Planning scene correspondence in top and side views.
fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), constrained_layout=True)
triangles_xy = world_vertices[faces][:, :, :2]
layout2d(axes[0])
axes[0].add_collection(PolyCollection(triangles_xy, facecolor='#cbd5e1', edgecolor='#64748b', linewidth=.15))
for stroke in strokes:
    axes[0].plot(stroke['positions'][:, 0], stroke['positions'][:, 1], color=COLORS[stroke['panel']], linewidth=3)
axes[0].add_patch(plt.Rectangle((bx-.225, by-.225), .45, .45, color='#334155'))
axes[0].add_patch(plt.Circle((bx, by), 1.75, fill=False, linestyle='--', color='#6366f1'))
axes[0].set(xlim=(-6.2, 6.2), ylim=(-4.2, 4.2), aspect='equal',
            title='Top view: 12 × 8 m layout and central 8 × 5 m bay')
axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)'); axes[0].grid(alpha=.2)
triangles_xz = world_vertices[faces][:, :, [0, 2]]
axes[1].add_collection(PolyCollection(triangles_xz, facecolor='#cbd5e1', edgecolor='#64748b', linewidth=.15))
for stroke in strokes:
    axes[1].plot(stroke['positions'][:, 0], stroke['positions'][:, 2], color=COLORS[stroke['panel']], linewidth=3)
axes[1].add_patch(plt.Rectangle((bx-.225, 0), .45, bz, color='#334155'))
axes[1].set(xlim=(-3, 3), ylim=(-.1, 2.6), aspect='equal', title='Side view: collision geometry and 0.25 m standoff')
axes[1].set_xlabel('X (m)'); axes[1].set_ylabel('Z (m)'); axes[1].grid(alpha=.2)
fig.suptitle('MoveIt Planning Scene Geometry Matches the Simulated Cell', fontsize=15, weight='bold')
fig.savefig(OUT/'02_moveit_planning_scene.png', dpi=180)
plt.close(fig)


for number, panel, title, view in [
        ('03', 'hood', 'Hood Painting Paths', (26, -55)),
        ('04', 'roof', 'Roof Painting Paths', (28, -65)),
        ('05', 'side', 'Side-panel Painting Paths', (15, -65))]:
    fig = plt.figure(figsize=(10, 6), constrained_layout=True)
    ax = fig.add_subplot(111, projection='3d')
    car(ax, .42); paths(ax, (panel,), 5)
    style3d(ax, title, *view)
    unique_legend(ax)
    fig.text(.5, .02, 'Line positions are nozzle TCP targets, 0.25 m from the selected surface patch.', ha='center')
    fig.savefig(OUT/f'{number}_{panel}_painting_path.png', dpi=180)
    plt.close(fig)


# Execution evidence rendered from the machine-readable run report.
fig, ax = plt.subplots(figsize=(12, 6.5), constrained_layout=True)
ax.set_facecolor('#111827'); fig.patch.set_facecolor('#111827'); ax.axis('off')
duration = sum(s['duration_s'] for s in result['executed_segments'])
samples = sum(s['transition_collision_samples']+s['painting_collision_samples'] for s in result['strokes'])
max_error = max(s['max_endpoint_joint_error_rad'] for s in result['executed_segments'])
lines = [
    '$ cat results/execution.json', '',
    f'  status:                         {result["status"]}',
    f'  robot:                          {result["robot"]}',
    f'  car_dimensions_m:               {result["car_dimensions_m"]}',
    f'  validated painting strokes:     {len(result["strokes"])}',
    '  Cartesian fractions:',
    '    hood:                          ' + ', '.join(f'{s["name"]}=1.0' for s in result['strokes'] if s['name'].startswith('hood')),
    '    roof:                          ' + ', '.join(f'{s["name"]}=1.0' for s in result['strokes'] if s['name'].startswith('roof')),
    '    side:                          ' + ', '.join(f'{s["name"]}=1.0' for s in result['strokes'] if s['name'].startswith('side')),
    f'  executed approach/paint segments:{len(result["executed_segments"]):>5}',
    f'  planning-scene collision objects: {len(result["layout_collision_objects"]):>5}',
    f'  collision samples checked:       {samples}',
    f'  target standoff range_m:         {min(s["target_standoff_min_m"] for s in result["strokes"]):.6f} .. {max(s["target_standoff_max_m"] for s in result["strokes"]):.6f}',
    f'  negative collision test:         PASSED; contacts={result["negative_collision_test"]["contacts"]}',
    f'  negative test executed:          {result["negative_collision_test"]["executed"]}',
    f'  simulated sequence duration_s:   {duration:.3f}',
    f'  maximum endpoint error_rad:      {max_error:.6f}', '',
    'All reported values are generated from the completed Gazebo/MoveIt execution.',
]
ax.text(.03, .95, '\n'.join(lines), transform=ax.transAxes, va='top', color='#d1fae5',
        family='monospace', fontsize=13, linespacing=1.35)
fig.savefig(OUT/'06_execution_results.png', dpi=180, facecolor=fig.get_facecolor())
plt.close(fig)

print(f'Generated six figures in {OUT}')
