import math
import numpy as np
import pytest
from z70_mapping.decoder import VendorMap
from z70_mapping.geometry import alignment, transform_pose, room_names


def context_map():
    pixels = np.full((101, 101), 7, dtype=np.uint8)
    occupancy = np.zeros(pixels.shape, dtype=np.int8)
    return VendorMap(101, 101, (-2.525, -2.525), pixels, occupancy,
                     (0., 0., math.pi / 2), (1., 0., None),
                     {2: {'centroid': (0., 2.), 'area_m2': 3.12345},
                      4: {'centroid': (1., 0.), 'area_m2': 1.}}, {}, {})


def test_spatial_context_room_identification_and_egocentric_geometry():
    from z70_mapping.geometry import spatial_context
    m = context_map()
    m.pixels[50, 50] = 23  # Explicit segmentation wins over nearer centroid.
    ctx = spatial_context(m, {2: 'Kitchen'})
    assert ctx['robot_pose_map'] == {'x': 0., 'y': 0., 'yaw': round(math.pi / 2, 3)}
    assert ctx['current_room'] == {'segment_id': 2, 'name': 'Kitchen'}
    assert ctx['rooms'] == [
        {'segment_id': 4, 'name': 'Segment 4', 'centroid': [1., 0.], 'area_m2': 1.,
         'distance_m': 1., 'bearing_rad': round(-math.pi / 2, 3)},
        {'segment_id': 2, 'name': 'Kitchen', 'centroid': [0., 2.], 'area_m2': 3.123,
         'distance_m': 2., 'bearing_rad': 0.}]
    assert ctx['dock'] == {'x': 1., 'y': 0., 'yaw': None, 'distance_m': 1.,
                           'bearing_rad': round(-math.pi / 2, 3)}
    assert spatial_context(m, {}, (1., 0., 0.))['current_room']['segment_id'] == 4


@pytest.mark.parametrize('pose,occupancy,expected', [
    ((0., 0., None), 0, 4), ((0., 0., None), 100, None),
    ((0., 0., None), -1, None), ((-2., -2., 0.), 0, None),
    ((-3., 0., 0.), 0, None), ((3., 0., 0.), 0, None),
])
def test_current_room_fallback_only_on_free_in_bounds_nearby_cell(pose, occupancy, expected):
    from z70_mapping.geometry import spatial_context
    m = context_map()
    m.occupancy[:] = occupancy
    current = spatial_context(m, {}, pose)['current_room']
    assert (current['segment_id'] if current else None) == expected


def test_spatial_context_handles_missing_pose_and_heading():
    from z70_mapping.geometry import spatial_context
    m = context_map()
    m.robot = None
    ctx = spatial_context(m, {})
    assert ctx['current_room'] is None and ctx['clearance_sectors_m'] == {}
    assert [r['segment_id'] for r in ctx['rooms']] == [2, 4]
    assert all(r['distance_m'] is None and r['bearing_rad'] is None for r in ctx['rooms'])
    assert ctx['dock']['distance_m'] is None
    ctx = spatial_context(m, {}, (0., 0., None))
    assert ctx['dock']['distance_m'] == 1. and ctx['dock']['bearing_rad'] is None
    m.dock = None
    assert spatial_context(m, {})['dock'] is None


def test_clearance_rays_cover_eight_relative_directions():
    from z70_mapping.geometry import spatial_context
    m = context_map()
    m.occupancy[:] = -1  # Unknown cells are not walls.
    m.occupancy[70, 50] = 100  # One metre north, robot faces north.
    ctx = spatial_context(m, {})
    sectors = ctx['clearance_sectors_m']
    assert sectors == pytest.approx({
        'front': .975, 'front_left': 3.575, 'left': 2.525,
        'rear_left': 3.575, 'rear': 2.525, 'rear_right': 3.575,
        'right': 2.525, 'front_right': 3.575}, abs=.025)
    assert spatial_context(m, {}, (3., 0., 0.))['clearance_sectors_m'] == dict.fromkeys(sectors, 0.)
    m.occupancy[50, 50] = 100
    assert spatial_context(m, {})['clearance_sectors_m'] == dict.fromkeys(sectors, 0.)


def test_clearance_rays_stop_at_five_metres():
    from z70_mapping.geometry import spatial_context
    m = context_map()
    m.width = m.height = 301
    m.origin = (-7.525, -7.525)
    m.pixels = np.full((301, 301), 7, dtype=np.uint8)
    m.occupancy = np.zeros((301, 301), dtype=np.int8)
    assert set(spatial_context(m, {})['clearance_sectors_m'].values()) == {5.}


def test_alignment_rotates_then_translates_odom_into_vendor_map():
    vendor = (27.4, 24.9, math.pi / 2)
    odom = (1., 2., -.2)
    t = alignment(vendor, odom)
    assert transform_pose(t, odom) == pytest.approx(vendor)
    assert transform_pose(t, (1., 2., 0.))[2] == pytest.approx(math.pi / 2 + .2)


def test_missing_vendor_heading_does_not_invent_alignment():
    assert alignment((1., 2., None), (0., 0., 0.)) is None


def test_room_names_only_join_explicit_vendor_segment_mapping():
    assert room_names([[2, '123', 14], [4, '456', 6]], [{'id':'123','name':'Kitchen'}]) == {2:'Kitchen'}
