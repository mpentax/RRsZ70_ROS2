"""Operational pincher FK, TF, collision, and articulation tests."""
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import xacro
from test_physical_visuals import SOURCE, origin, relative_frames
from test_pincher_mesh import visual_triangles
from test_wrist_tf import publish_and_lookup, quat_matrix, tf_robot


@pytest.mark.parametrize('state',[(150,45,0,90),(240,95,0,0),(200,95,40,45),(170,65,155,90)])
def test_actual_published_finger_frames_match_operational_four_angle_geometry(tf_robot,state):
    from z70_driver.driver import app_to_urdf
    from z70_description import pincher_geometry as g
    transforms=publish_and_lookup(tf_robot, app_to_urdf(*state), 'finger_left', 'finger_right')
    T=g.hand_pose(state)
    for side,sign in (('left',1),('right',-1)):
        expected=T@g.finger_frame(sign,state[3]);tr=transforms['finger_'+side].transform
        assert [tr.translation.x,tr.translation.y,tr.translation.z]==pytest.approx(expected[:3,3],abs=1e-8)
        assert quat_matrix(tr.rotation)==pytest.approx(expected[:3,:3],abs=1e-8)



def test_display_has_fiducial_cells_underside_collar_and_finger_pivots():
    robot=ET.fromstring(xacro.process_file(str(SOURCE)).toxml())
    cells=[v for v in robot.find("link[@name='pincher_marker_68']").findall('visual')
           if v.find('geometry/box') is not None and abs(float(v.find('geometry/box').get('size').split()[0])-.0025)<1e-6]
    assert len(cells)>=6, 'ID68 is a pattern, not a blank square'
    shell = robot.find("link[@name='arm_wrist_link']").find('visual/geometry/mesh')
    assert shell is not None and shell.get('filename') and 'cad_hand.stl' in shell.get('filename')


def test_open_fingers_spread_sideways_and_never_cross_the_midplane():
    from z70_description import pincher_geometry as g
    for m6 in (0,15,45,75,90):
        for sign in (-1,1):
            T=g.finger_frame(sign,m6)
            vertices=np.concatenate(g.finger_rings(sign))@T[:3,:3].T+T[:3,3]
            assert (sign*vertices[:,1]).min()>=-1e-9
    # Two stills do not recover a closed contact. A 25mm curl was only there so
    # the tips met at M6=0, and it is what wrapped the open fingers around the nose.
    tips=[g.finger_rings(sign)[-1].mean(axis=0) for sign in (-1,1)]
    assert abs(tips[0][1]) < .012, 'local tip kink stays short'


def test_finger_anatomy_is_independent_of_nominal_opening_gain(monkeypatch):
    from z70_description import pincher_geometry as g
    expected=np.concatenate(g.finger_rings(1)).copy()
    # Regression for the inverse-cosine length explosion near90deg. Altering
    # nominal drive mapping must rotate the same anatomy, never resize it.
    try:
        for ratio in (.5,2/3,.85,.999999,1.):
            monkeypatch.setitem(g.CONFIG,'opening_ratio',ratio)
            g.finger_rings.cache_clear()
            assert np.concatenate(g.finger_rings(1))==pytest.approx(expected)
    finally:g.finger_rings.cache_clear()


def test_shared_articulated_bodies_contain_visual_vertices_and_match_urdf():
    from z70_description import pincher_geometry as g
    robot=ET.fromstring(xacro.process_file(str(SOURCE)).toxml())
    for m5,m6 in ((0,0),(0,45),(0,90),(40,90),(155,45)):
        frames=relative_frames(robot,'arm_wrist_link',{'m6_left_joint':math.radians(m6)/2,'m6_right_joint':math.radians(m6)/2,
            'finger_left_articulation':math.radians(m6)*g.CONFIG['opening_ratio'],
            'finger_right_articulation':math.radians(m6)*g.CONFIG['opening_ratio']})
        bodies=g.local_bodies(m6)
        for name in ('arm_wrist_link','finger_left','finger_right'):
            mesh=visual_triangles(robot,name).reshape(-1,3)
            pts=mesh@frames[name][:3,:3].T+frames[name][:3,3]
            own=[b for b in bodies if b.name==name]
            assert own and all(any(b.contains(p,2e-7) for b in own) for p in pts)
            collisions=robot.find(f"link[@name='{name}']").findall('collision')
            assert len(collisions)==len(own)
            for collision,body in zip(collisions,own):
                tf=frames[name]@origin(collision.find('origin'))
                assert tf[:3,3]==pytest.approx(body.center)
                assert np.fromstring(collision.find('geometry/box').get('size'),sep=' ')==pytest.approx(body.half*2)
