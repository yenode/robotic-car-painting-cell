#!/usr/bin/env python3

from pathlib import Path
import json
import numpy as np

root = Path(__file__).resolve().parents[1] / 'src/painting_cell/models/hatchback'
source = root / 'meshes/hatchback.obj'
lines = source.read_text().splitlines()
vertices = np.array([list(map(float, l.split()[1:4])) for l in lines if l.startswith('v ')])
rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
rotated = vertices @ rotation.T
low, high = rotated.min(axis=0), rotated.max(axis=0)
scale = np.array([4.8, 1.9, 1.6]) / (high - low)
offset = np.array([(low[0]+high[0])/2, (low[1]+high[1])/2, low[2]])
normalized = (rotated-offset) * scale
output, index = [], 0
for line in lines:
    if line.startswith('v '):
        output.append('v ' + ' '.join(f'{v:.9f}' for v in normalized[index]))
        index += 1
    elif line.startswith('vn '):
        n = (rotation @ np.array(list(map(float, line.split()[1:4])))) / scale
        n /= max(np.linalg.norm(n), 1e-12)
        output.append('vn ' + ' '.join(f'{v:.9f}' for v in n))
    else:
        output.append(line)
(root/'meshes/car_scaled.obj').write_text('\n'.join(output)+'\n')
(root/'model.sdf').write_text()
(root/'dimensions.json').write_text(json.dumps({'dimensions_m': (normalized.max(0)-normalized.min(0)).tolist(),
    'min_m': normalized.min(0).tolist(), 'max_m': normalized.max(0).tolist(),
    'source': 'OpenRobotics Hatchback v3', 'method': '90 degree Z rotation, axis scaling, centered XY and grounded Z'}, indent=2)+'\n')
print((root/'dimensions.json').read_text())
