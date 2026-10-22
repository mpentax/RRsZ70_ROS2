"""Pincher mesh manifold, normal, installation, URDF link lengths, generator, and wrist TF tests."""
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
import xacro
import yaml
from z70_description import arm_model as g
from z70_description.arm_model import rpy
from z70_driver.driver import app_to_urdf
from z70_manipulation import planner
from z70_manipulation.node import rotation as quat_matrix

SOURCE = Path(__file__).resolve().parents[1] / 'urdf/z70.urdf.xacro'
ROBOT = ET.fromstring(xacro.process_file(str(SOURCE)).toxml())
PACKAGE = SOURCE.parents[1]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = rpy(np.fromstring(element.get('rpy', '0 0 0'), sep=' '))
    return transform


def relative_frames(robot, root, angles):
    frames = {root: np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            motion = np.eye(4)
            if joint.get('type') != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                mimic = joint.find('mimic')
                angle = angles.get(joint.get('name'), 0.)
                if mimic is not None:
                    angle = angles.get(mimic.get('joint'), 0.) * float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
                motion[:3, :3] = rotation(axis, angle)
            frames[joint.find('child').get('link')] = frames[parent] @ origin(joint.find('origin')) @ motion
            pending.remove(joint)
            progressed = True
        if not progressed:
            break
    return frames


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


def test_display_has_fiducial_cells_underside_collar_and_finger_pivots():
    cells = [v for v in ROBOT.find("link[@name='pincher_marker_68']").findall('visual')
             if v.find('geometry/box') is not None and abs(float(v.find('geometry/box').get('size').split()[0]) - .0025) < 1e-6]
    assert len(cells) >= 6, 'ID68 is a pattern, not a blank square'
    shell = ROBOT.find("link[@name='arm_wrist_link']").find('visual/geometry/mesh')
    assert shell is not None and shell.get('filename') and 'cad_hand.stl' in shell.get('filename')


def test_open_fingers_spread_sideways_and_never_cross_the_midplane():
    for m6 in (0, 15, 45, 75, 90):
        for sign in (-1, 1):
            T = g.finger_frame(sign, m6)
            vertices = np.concatenate(g.finger_rings(sign)) @ T[:3, :3].T + T[:3, 3]
            assert (sign * vertices[:, 1]).min() >= -1e-9
    tips = [g.finger_rings(sign)[-1].mean(axis=0) for sign in (-1, 1)]
    assert abs(tips[0][1]) < .012, 'local tip kink stays short'


def test_finger_anatomy_is_independent_of_nominal_opening_gain(monkeypatch):
    expected = np.concatenate(g.finger_rings(1)).copy()
    try:
        for ratio in (.5, 2 / 3, .85, .999999, 1.):
            monkeypatch.setitem(g.CONFIG, 'opening_ratio', ratio)
            g.finger_rings.cache_clear()
            assert np.concatenate(g.finger_rings(1)) == pytest.approx(expected)
    finally:
        g.finger_rings.cache_clear()


def test_shared_articulated_bodies_contain_visual_vertices_and_match_urdf():
    for m5, m6 in ((0, 0), (0, 45), (0, 90), (40, 90), (155, 45)):
        frames = relative_frames(ROBOT, 'arm_wrist_link', {
            'm6_left_joint': math.radians(m6) / 2,
            'm6_right_joint': math.radians(m6) / 2,
            'finger_left_articulation': math.radians(m6) * g.CONFIG['opening_ratio'],
            'finger_right_articulation': math.radians(m6) * g.CONFIG['opening_ratio'],
        })
        bodies = g.local_bodies(m6)
        for name in ('arm_wrist_link', 'finger_left', 'finger_right'):
            mesh = visual_triangles(ROBOT, name).reshape(-1, 3)
            pts = mesh @ frames[name][:3, :3].T + frames[name][:3, 3]
            own = [b for b in bodies if b.name == name]
            assert own and all(any(b.contains(p, 2e-7) for b in own) for p in pts)
            collisions = ROBOT.find(f"link[@name='{name}']").findall('collision')
            assert len(collisions) == len(own)
            for collision, body in zip(collisions, own):
                tf = frames[name] @ origin(collision.find('origin'))
                assert tf[:3, 3] == pytest.approx(body.center)
                assert np.fromstring(collision.find('geometry/box').get('size'), sep=' ') == pytest.approx(body.half * 2)


@pytest.mark.parametrize('m5', [0, 35, 155])
def test_open_second_assembly_uses_the_copied_wrist(m5):
    frames = relative_frames(ROBOT, 'arm_link2', {
        'm5_joint': -math.radians(m5),
        'm6_left_joint': math.pi / 4,
        'm6_right_joint': math.pi / 4,
    })
    assert frames['arm_wrist_link'][:3, 3] == pytest.approx([0.1157, 0, 0])


def test_first_segment_uses_the_copied_link_length():
    joint_origin = ROBOT.find("joint[@name='m4_joint']/origin").get('xyz').split()
    assert float(joint_origin[0]) == pytest.approx(0.1772, abs=1e-4)
    mesh = ROBOT.find("link[@name='arm_link1']").find("visual/geometry/mesh[@filename='package://z70_description/meshes/cad/cad_link1.stl']")
    assert mesh is not None


def test_marker_is_distal_and_flat_on_cad_underside():
    mount = origin(ROBOT.find("joint[@name='pincher_marker_68_mount']/origin"))
    assert mount[0, 3] == pytest.approx(.024)
    assert mount[1, 3] == pytest.approx(0.)
    distal = np.array([1., 0., -.078125]); distal /= np.linalg.norm(distal)
    assert mount[:3, 1] == pytest.approx(distal, abs=1e-8)
    assert mount[:3, 2] == pytest.approx(np.cross(mount[:3, 0], distal), abs=1e-8)
    assert mount[2, 2] < 0


@pytest.mark.parametrize('roll', [0., .6, 2.7])
def test_marker_rotation_recovers_pincher_rotation_without_translation(roll):
    frames = relative_frames(ROBOT, 'arm_link2', {'m5_joint': -roll})
    mount = origin(ROBOT.find("joint[@name='pincher_marker_68_mount']/origin"))
    recovered = frames['pincher_marker_68'][:3, :3] @ mount[:3, :3].T
    assert recovered == pytest.approx(frames['arm_wrist_link'][:3, :3], abs=1e-10)


@pytest.fixture(scope='module')
def tf_robot(tmp_path_factory):
    directory = tmp_path_factory.mktemp('wrist_tf')
    description = xacro.process_file(str(Path(get_package_share_directory('z70_description')) / 'urdf/z70.urdf.xacro')).toxml()
    params = directory / 'robot.yaml'
    params.write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': description}}}))
    executable = Path(get_package_prefix('robot_state_publisher')) / 'lib/robot_state_publisher/robot_state_publisher'
    log = (directory / 'publisher.log').open('w')
    process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(params)],
                               env={**os.environ, 'ROS_DOMAIN_ID': '91'}, stdout=log, stderr=log)
    rclpy.init(domain_id=91)
    node = rclpy.create_node('wrist_fk_test')
    buffer = Buffer()
    TransformListener(buffer, node)
    publisher = node.create_publisher(JointState, '/joint_states', 10)
    try:
        yield node, buffer, publisher, process
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()


def publish_and_lookup(tf_robot, angles, *frames):
    node, buffer, publisher, process = tf_robot
    msg = JointState(name=list(angles), position=list(angles.values()))
    started = node.get_clock().now()
    deadline = time.monotonic() + 8
    found = {}
    while time.monotonic() < deadline:
        assert process.poll() is None, 'robot_state_publisher exited'
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        for frame in frames:
            try:
                candidate = buffer.lookup_transform('base_footprint', frame, Time())
                if Time.from_msg(candidate.header.stamp) >= started:
                    found[frame] = candidate
            except TransformException:
                pass
        if len(found) == len(frames):
            return found[frames[0]] if len(frames) == 1 else found
    raise AssertionError(f'Missing TF for {frames}')


@pytest.mark.parametrize('m3,m4,m5,m6', [
    (150, 45, 0, 90), (150, 75, 0, 0), (200, 90, 35, 90),
    (240, 130, 155, 0), (90, 170, 0, 0),
])
def test_nominal_wrist_tf_matches_effective_chain(tf_robot, m3, m4, m5, m6):
    transform = publish_and_lookup(tf_robot, app_to_urdf(m3, m4, m5, m6), 'arm_wrist_link')
    wrist = transform.transform.translation
    expected = planner.measured_point((m3, m4), (planner.CAD_L2, 0., 0.))
    assert (wrist.x, wrist.y, wrist.z) == pytest.approx(expected, abs=1e-8)


@pytest.mark.parametrize('m3,m4', [(170, 65), (200, 95), (240, 105)])
@pytest.mark.parametrize('m5', [0, 35, 155])
def test_measured_gripper_tf(tf_robot, m3, m4, m5):
    _, buffer, _, _ = tf_robot
    tr = publish_and_lookup(tf_robot, app_to_urdf(m3, m4, m5, 90), 'gripper_camera_optical_frame')
    expected, R = planner.camera_pose((m3, m4, m5))
    p = tr.transform.translation; actual = quat_matrix(tr.transform.rotation)
    assert (p.x, p.y, p.z) == pytest.approx(expected, abs=1e-8)
    assert actual == pytest.approx(R, abs=1e-8)
    cfg = planner._CALIBRATION
    camera = np.array([planner.CAD_L2, 0, 0]) + np.array(planner._CAD['camera_from_wrist_m'], float)
    marker = np.array([planner.CAD_L2, 0, 0]) + np.array(planner._CAD['marker_from_wrist_m'], float)
    jaw = np.array(cfg['effective_jaw_from_elbow_m'])
    assert jaw == pytest.approx(np.array(cfg['marker_effective_from_elbow_m']) + np.array(cfg['marker_rotation_link2']) @ cfg['jaw_region_in_marker_m'], abs=1e-12)
    assert cfg['jaw_nominal_wrist_offset_m'] == pytest.approx(jaw - np.array([.205, 0, 0]), abs=1e-12)
    roll = np.deg2rad(m5); c, s = np.cos(roll), np.sin(roll)
    Q = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    tcp = buffer.lookup_transform('base_footprint', 'gripper_tcp', Time()).transform.translation
    expected_jaw = planner.measured_point((m3, m4), Q @ jaw)
    assert (tcp.x, tcp.y, tcp.z) == pytest.approx(expected_jaw, abs=1e-8)
    optical = planner.hand.rpy(planner._CAD['camera_rpy']) @ np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    marker_rotation = planner.hand.rpy(planner._CAD['marker_rpy'])
    marker_in_optical = optical.T @ (marker - camera)
    actual_marker = np.array([p.x, p.y, p.z]) + actual @ marker_in_optical
    expected_marker = planner.measured_point((m3, m4), Q @ marker)
    assert actual_marker == pytest.approx(expected_marker, abs=1e-8)
    marker_tf = buffer.lookup_transform('gripper_camera_optical_frame', 'pincher_marker_68', Time()).transform
    assert (marker_tf.translation.x, marker_tf.translation.y, marker_tf.translation.z) == pytest.approx(marker_in_optical, abs=1e-8)
    assert quat_matrix(marker_tf.rotation) == pytest.approx(optical.T @ marker_rotation, abs=1e-8)
    a = np.deg2rad(m4 - m3 + 90); ca, sa = np.cos(a), np.sin(a)
    cy, sy = np.cos(planner.YAW), np.sin(planner.YAW)
    link2_rotation = np.array([[cy * ca, -sy, -cy * sa], [sy * ca, cy, -sy * sa], [sa, 0, ca]])
    assert actual @ optical.T @ marker_rotation == pytest.approx(link2_rotation @ Q @ marker_rotation, abs=1e-8)


@pytest.mark.parametrize('state', [(150, 45, 0, 90), (240, 95, 0, 0), (200, 95, 40, 45), (170, 65, 155, 90)])
def test_actual_published_finger_frames_match_operational_four_angle_geometry(tf_robot, state):
    transforms = publish_and_lookup(tf_robot, app_to_urdf(*state), 'finger_left', 'finger_right')
    T = g.hand_pose(state)
    for side, sign in (('left', 1), ('right', -1)):
        expected = T @ g.finger_frame(sign, state[3])
        tr = transforms['finger_' + side].transform
        assert [tr.translation.x, tr.translation.y, tr.translation.z] == pytest.approx(expected[:3, 3], abs=1e-8)
        assert quat_matrix(tr.rotation) == pytest.approx(expected[:3, :3], abs=1e-8)


def test_cad_urdf_planner_mujoco_full_joint_sweep_agreement():
    import mujoco  # type: ignore[import-not-found]
    from z70_sim.world import DEFAULT_OBJECTS, SimWorld  # type: ignore[import-not-found]

    def all_urdf_frames(angles):
        frames = {'base_footprint': np.eye(4)}
        pending = list(ROBOT.findall('joint'))
        while pending:
            for joint in pending[:]:
                parent_el = joint.find('parent')
                child_el = joint.find('child')
                assert parent_el is not None and child_el is not None
                parent = parent_el.get('link') or ''
                child = child_el.get('link') or ''
                if parent not in frames:
                    continue
                motion = np.eye(4)
                jtype = joint.get('type')
                if jtype in ('revolute', 'continuous', 'prismatic'):
                    axis_el = joint.find('axis')
                    assert axis_el is not None
                    axis = np.fromstring(axis_el.get('xyz', '0 0 1'), sep=' ')
                    val = angles.get(joint.get('name') or '', 0.)
                    mimic = joint.find('mimic')
                    if mimic is not None:
                        val = angles.get(mimic.get('joint') or '', 0.) * float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
                    if jtype == 'prismatic':
                        motion[:3, 3] = axis * val
                    else:
                        motion[:3, :3] = rotation(axis, val)
                frames[child] = frames[parent] @ origin(joint.find('origin')) @ motion
                pending.remove(joint)
        return frames

    w = SimWorld(objects=DEFAULT_OBJECTS)
    body_names = (
        'base_footprint', 'arm_lid_base', 'arm_compartment_lid', 'arm_vendor_frame',
        'arm_shoulder_link', 'arm_link1', 'arm_link2', 'arm_wrist_link',
        'jaw_left', 'jaw_right', 'pincher_marker_68',
    )

    for deployed in (False, True):
        for base_pose in ((0.0, 0.0, 0.0), (0.35, -0.20, 0.45), (-0.15, 0.40, -1.10)):
            bx, by, byaw = base_pose
            cb, sb = math.cos(byaw), math.sin(byaw)
            T_world_base = np.array([
                [cb, -sb, 0.0, bx],
                [sb, cb, 0.0, by],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ])
            states = [(90.0, 170.0, 0.0, 0.0)] if not deployed else [
                (m3, m4, m5, m6)
                for m3 in (95.0, 150.0, 195.0, 240.0, 265.0)
                for m4 in (36.0, 75.0, 115.0, 165.0)
                for m5 in (0.0, 40.0, 155.0)
                for m6 in (0.0, 45.0, 90.0)
            ]
            for st in states:
                w.deployed = deployed
                w.pose = list(base_pose)
                w.joints = dict(m3=st[0], m4=st[1], m5=st[2], m6=st[3])
                w._sync_kinematics()
                mujoco.mj_forward(w.model, w.data)

                active = w._active_joints()
                q = app_to_urdf(*active)
                q['arm_lid'] = 0.0 if deployed else 0.058
                uf = all_urdf_frames(q)

                sh, el, wr, gr = g.cad_forward(active)
                T_hand = g.hand_pose(active)
                assert np.linalg.norm(uf['arm_shoulder_link'][:3, 3] - sh) < 5e-4
                assert np.linalg.norm(uf['arm_link2'][:3, 3] - el) < 5e-4
                assert np.linalg.norm(uf['arm_wrist_link'][:3, 3] - wr) < 5e-4
                assert np.linalg.norm(T_hand[:3, 3] - wr) < 1e-9
                assert np.allclose(uf['arm_wrist_link'][:3, :3], T_hand[:3, :3], atol=1e-6)
                assert np.allclose(planner.operational_point(active), gr, atol=1e-9)

                cam_p, cam_R = planner.camera_pose(active[:3])
                assert np.linalg.norm(uf['gripper_camera_optical_frame'][:3, 3] - cam_p) < 5e-4
                assert np.allclose(uf['gripper_camera_optical_frame'][:3, :3], cam_R, atol=1e-6)

                for bname in body_names:
                    bid = mujoco.mj_name2id(w.model, mujoco.mjtObj.mjOBJ_BODY, bname)
                    mj_p = w.data.xpos[bid]
                    mj_R = w.data.xmat[bid].reshape(3, 3)
                    T_urdf_world = T_world_base @ uf[bname]
                    pos_err = float(np.linalg.norm(mj_p - T_urdf_world[:3, 3]))
                    rot_err = math.acos(max(-1.0, min(1.0, (float(np.trace(mj_R @ T_urdf_world[:3, :3].T)) - 1.0) / 2.0)))
                    assert pos_err < 5e-4, f'{bname} pos_err={pos_err}'
                    assert rot_err < 2e-3, f'{bname} rot_err={rot_err}'

                for cname, ulink in (('front', 'camera_optical_frame'), ('gripper', 'gripper_camera_optical_frame')):
                    cid = mujoco.mj_name2id(w.model, mujoco.mjtObj.mjOBJ_CAMERA, cname)
                    mj_p = w.data.cam_xpos[cid]
                    mj_R_opt = w.data.cam_xmat[cid].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
                    T_cam_world = T_world_base @ uf[ulink]
                    pos_err = float(np.linalg.norm(mj_p - T_cam_world[:3, 3]))
                    rot_err = math.acos(max(-1.0, min(1.0, (float(np.trace(mj_R_opt @ T_cam_world[:3, :3].T)) - 1.0) / 2.0)))
                    assert pos_err < 5e-4, f'camera {cname} pos_err={pos_err}'
                    assert rot_err < 2e-3, f'camera {cname} rot_err={rot_err}'

