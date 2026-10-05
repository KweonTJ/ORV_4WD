#!/usr/bin/env python3
"""Convert our generated COLLADA triangle groups to MuJoCo OBJ assets."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def convert(source, output):
    root = ET.parse(source).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    groups = []
    for geometry in root.findall('c:library_geometries/c:geometry', ns):
        mesh = geometry.find('c:mesh', ns)
        vertices = mesh.find('c:vertices/c:input', ns).get('source')[1:]
        points = np.fromstring(mesh.find(f"c:source[@id='{vertices}']/c:float_array", ns).text,
                               sep=' ').reshape(-1, 3)
        triangles = mesh.find('c:triangles', ns)
        inputs = triangles.findall('c:input', ns)
        stride = 1 + max(int(i.get('offset')) for i in inputs)
        offset = int(next(i for i in inputs if i.get('semantic') == 'VERTEX').get('offset'))
        indices = np.fromstring(triangles.find('c:p', ns).text, sep=' ', dtype=int).reshape(-1, stride)[:, offset]
        material = triangles.get('material')
        color = root.find(f"c:library_effects/c:effect[@id='{material}-fx']//c:diffuse/c:color", ns)
        rgba = color.text if color is not None else '.2 .2 .2 1'
        unique, inverse = np.unique(points[indices], axis=0, return_inverse=True)
        name = source.stem + '_' + material
        path = output / (name + '.obj')
        with path.open('w') as f:
            f.write('# Generated from the ORV RViz COLLADA asset; units: metres\n')
            for x, y, z in unique: f.write(f'v {x:.8g} {y:.8g} {z:.8g}\n')
            for a, b, c in inverse.reshape(-1, 3) + 1: f.write(f'f {a} {b} {c}\n')
        groups.append({'name': name, 'file': path.name, 'rgba': rgba})
    return {'sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'groups': groups}


if __name__ == '__main__':
    package = Path(__file__).resolve().parents[1]
    meshes = package.parent / 'orv_description/meshes'
    manifest = {name: convert(meshes / (name + '.dae'), package / 'meshes')
                for name in ('v3_chassis', 'rubber_wheel')}
    (package / 'meshes/manifest.json').write_text(json.dumps(manifest, indent=2))
