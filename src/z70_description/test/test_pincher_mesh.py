"""Pincher mesh manifold, normal, installation, and generator tests."""
from pathlib import Path
import shutil
import struct
import subprocess
import sys

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from test_physical_visuals import ROBOT, SOURCE, origin, relative_frames

PACKAGE = SOURCE.parents[1]


def stl_records(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + 50*count
    records = np.dtype([('normal', '<f4', (3,)), ('points', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(data, dtype=records, offset=84)


def triangles(path):
    return stl_records(path)['points'].astype(float)


def manifold_edges(points):
    vertices, indices = np.unique(np.round(points.reshape(-1, 3), 8), axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    assert np.all(counts == 2)
    return vertices, edges


OPERATIONAL = {
    'arm_wrist_link': 'pincher_shell.stl',
    'finger_left': 'pincher_finger_left.stl',
    'finger_right': 'pincher_finger_right.stl',
}


def visual_triangles(robot, link):
    result = []
    for visual in robot.find(f"link[@name='{link}']").findall('visual'):
        mesh = visual.find('geometry/mesh')
        if mesh is None:
            continue
        uri = mesh.get('filename')
        name = Path(uri).name
        if link in OPERATIONAL and name != OPERATIONAL[link]:
            continue
        assert uri.startswith('package://z70_description/meshes/')
        points = triangles(PACKAGE / uri.split('package://z70_description/')[1])
        t = origin(visual.find('origin'))
        result.append(points @ t[:3, :3].T + t[:3, 3])
    if not result and link in OPERATIONAL:
        # Vendor display meshes are not the operational loft. Tests keep the loft.
        result.append(triangles(PACKAGE / 'meshes' / OPERATIONAL[link]))
    assert result, f'{link} must use sculpted mesh geometry, not separate box blobs'
    return np.concatenate(result)


def test_one_connected_housing_has_recess_with_clear_calibrated_optical_ray():
    shell = visual_triangles(ROBOT, 'arm_wrist_link')
    vertices, edges = manifold_edges(shell)
    neighbors = {i: set() for i in range(len(vertices))}
    for a, b in edges:
        neighbors[a].add(b)
        neighbors[b].add(a)
    reached, pending = set(), [0]
    while pending:
        index = pending.pop()
        if index not in reached:
            reached.add(index)
            pending.extend(neighbors[index]-reached)
    assert len(reached) == len(vertices), 'no disconnected camera blob'
    assert np.linalg.norm(np.cross(shell[:, 1]-shell[:, 0], shell[:, 2]-shell[:, 0]), axis=1).min() > 1e-12
    assert np.einsum('ij,ij->', shell[:, 0], np.cross(shell[:, 1], shell[:, 2]))/6 > 0
    camera = relative_frames(ROBOT, 'arm_wrist_link', {})['gripper_camera_link']
    assert camera[:3, 3] == pytest.approx([0.0355, 0.0075, -0.0109], abs=1e-4)


def test_package_meshes_are_installed_for_foxglove_asset_lookup():
    installed = Path(get_package_share_directory('z70_description'))
    meshes = ROBOT.findall('.//geometry/mesh')
    assert meshes
    for mesh in meshes:
        relative = mesh.get('filename').split('package://z70_description/')[1]
        assert (installed / relative).read_bytes() == (PACKAGE / relative).read_bytes()


def test_all_mesh_normals_match_non_degenerate_positive_manifold_geometry():
    for path in (PACKAGE / 'meshes').glob('*.stl'):
        rec = stl_records(path)
        points = rec['points'].astype(float)
        stored_normals = rec['normal']
        cross = np.cross(points[:, 1]-points[:, 0], points[:, 2]-points[:, 0])
        area = np.linalg.norm(cross, axis=1)
        assert area.min() > 1e-12
        assert np.einsum('ij,ij->i', stored_normals, cross/area[:, None]) == pytest.approx(np.ones(len(points)), abs=1e-5)
        assert np.einsum('ij,ij->', points[:, 0], np.cross(points[:, 1], points[:, 2]))/6 > 0
        manifold_edges(points)


def test_offline_generator_reproduces_every_installed_triangle(tmp_path):
    for d in ('meshes', 'config', 'urdf'):
        (tmp_path / d).mkdir()
    generator = PACKAGE / 'meshes/generate_pincher.py'
    shutil.copy(generator, tmp_path / 'meshes' / generator.name)
    for cfg in ('cad_arm.yaml', 'mount_calibration.yaml', 'pincher_geometry.yaml'):
        shutil.copy(PACKAGE / 'config' / cfg, tmp_path / 'config' / cfg)
    shutil.copytree(PACKAGE / 'z70_description', tmp_path / 'z70_description', ignore=shutil.ignore_patterns('__pycache__'))
    subprocess.run([sys.executable, str(tmp_path / 'meshes' / generator.name)], check=True, capture_output=True)
    for source in (PACKAGE / 'meshes').glob('*.stl'):
        assert (tmp_path / 'meshes' / source.name).read_bytes() == source.read_bytes()
