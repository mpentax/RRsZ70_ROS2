"""Vendor V1 binary map block decoding and occupancy grid conversion."""
import hashlib
import struct
import pytest
from z70_mapping.decoder import decode_map


def fixture(pixels=bytes([1, 23, 0, 39, 255, 7]), width=3, height=2, extra_blocks=b''):
    def block(kind, data, extra=b''):
        return struct.pack('<HHI', kind, 8 + len(extra), len(data)) + extra + data
    image = block(2, pixels, struct.pack('<iiII', 100, 200, height, width))
    robot = block(8, struct.pack('<iii', 10075, 5025, 90))
    dock = block(1, struct.pack('<ii', 10000, 5000))
    body = image + robot + dock + extra_blocks + struct.pack('<HHI', 1024, 8, 20)
    header = b'rr' + struct.pack('<HI', 20, len(body) + 20) + bytes(12)
    raw = header + body
    return raw + hashlib.sha1(raw).digest()


def test_vendor_grid_bottom_row_and_mm_positions():
    m = decode_map(fixture())
    assert (m.width, m.height, m.resolution) == (3, 2, .05)
    assert m.origin == (10., 5.)
    assert m.occupancy.tolist() == [[100, 0, -1], [0, 0, 0]]
    assert m.robot == pytest.approx((10.075, 5.025, 1.5707963267948966))
    assert m.dock == (10., 5., None)
    assert sorted(m.rooms) == [2, 4]
    assert m.rooms[2]['centroid'] == pytest.approx((10.075, 5.025))


def test_zero_vendor_photo_obstacles_are_explicit_not_assumed_from_floor():
    empty = b''.join(struct.pack('<HHIHH', kind, 12, 0, 0, 0) for kind in (12,15,16))
    m = decode_map(fixture(extra_blocks=empty))
    assert m.obstacle_counts == {12:0,15:0,16:0}


def block(kind, data):
    return struct.pack('<HHI', kind, 8, len(data)) + data


@pytest.mark.parametrize('kind,attribute', [(3, 'path'), (9, 'goto_path')])
def test_path_blocks_in_metres(kind, attribute):
    m = decode_map(fixture(extra_blocks=block(kind, struct.pack('<4H', 100, 200, 1200, 2300))))
    assert getattr(m, attribute) == [(0.1, 0.2), (1.2, 2.3)]


@pytest.mark.parametrize('kind', [18, 19, 47, 53])
def test_zone_blocks_in_metres(kind):
    corners = (1000, 2000, 3000, 2000, 3000, 4000, 1000, 4000)
    payload = struct.pack('<8H', *corners)
    if kind in (47, 53):
        payload = struct.pack('<2H', 123, 456) + payload
    m = decode_map(fixture(extra_blocks=block(kind, payload * 2)))
    assert m.zones == [[(1., 2.), (3., 2.), (3., 4.), (1., 4.)]] * 2


def test_virtual_walls_and_carpet():
    extras = block(10, struct.pack('<8H', 100, 200, 300, 400, 500, 600, 700, 800))
    extras += block(17, bytes([0, 1, 2, 0, 255, 0]))
    m = decode_map(fixture(extra_blocks=extras))
    assert m.virtual_walls == [(0.1, 0.2, 0.3, 0.4), (0.5, 0.6, 0.7, 0.8)]
    assert m.carpet.dtype == bool
    assert m.carpet.tolist() == [[False, True, True], [False, True, False]]


@pytest.mark.parametrize('kind,attribute', [(3, 'path'), (9, 'goto_path'),
    (10, 'virtual_walls'), (18, 'zones'), (19, 'zones'), (47, 'zones'), (53, 'zones')])
def test_malformed_optional_geometry_is_ignored(kind, attribute):
    m = decode_map(fixture(extra_blocks=block(kind, b'\x00')))
    assert getattr(m, attribute) == []


def test_wrong_carpet_size_is_ignored():
    assert decode_map(fixture(extra_blocks=block(17, b'\x00'))).carpet is None


def test_bad_digest_rejected():
    raw = bytearray(fixture()); raw[30] ^= 1
    with pytest.raises(ValueError, match='integrity'):
        decode_map(raw)


def test_bad_grid_extent_rejected():
    with pytest.raises(ValueError, match='image'):
        decode_map(fixture(width=4))
