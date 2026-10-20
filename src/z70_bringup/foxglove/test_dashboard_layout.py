"""Offline exported-layout integrity checks (no robot access)."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
LAYOUT = json.loads((HERE / 'z70-dashboard.json').read_text())
CONFIGS = LAYOUT['configById']


def test_tabs_reference_existing_panels_and_do_not_include_command_panels():
    visited = set()

    def walk(item):
        if isinstance(item, str):
            assert item in CONFIGS
            assert item not in visited
            visited.add(item)
            if item.startswith('Tab!'):
                for tab in CONFIGS[item]['tabs']:
                    walk(tab['layout'])
        else:
            assert item['direction'] in ('row', 'column')
            assert 0 < item['splitPercentage'] < 100
            walk(item['first'])
            walk(item['second'])

    walk(LAYOUT['layout'])
    assert visited == CONFIGS.keys()
    assert [t['title'] for t in CONFIGS['Tab!z70-dashboard']['tabs']] == [
        'Overview', 'Cameras', 'Arm', 'Odometry & calibration', 'Diagnostics']
    assert not any(k.split('!')[0] in ('Publish', 'Teleop', 'ServiceCall') for k in CONFIGS)


def test_3d_and_image_configs_use_real_fields_and_sources():
    scene = CONFIGS['3D!Robot-map-and-depth']
    assert scene['layers']['z70-urdf']['sourceType'] == 'topic'
    assert scene['layers']['z70-urdf']['topic'] == '/robot_description'
    assert scene['followTf'] == 'odom'
    assert scene['fixedFrame'] == 'map'
    assert scene['layers']['z70-grid']['frameId'] == 'odom'
    assert scene['topics']['/z70/odom_path']['type'] == 'line'
    assert scene['topics']['/z70/odom_path']['lineWidth'] == 0.02
    assert scene['topics']['/z70/floor_mosaic']['cameraInfoTopic'] == '/z70/camera_info'
    assert scene['topics']['/map']['visible'] is True
    assert scene['cameraState']['distance'] == 8.0
    dock = scene['topics']['/z70/map/dock_pose']
    assert dock['visible'] is True
    assert dock['type'] == 'arrow'
    assert dock['color'] == '#34d399'
    assert dock['arrowScale'] == [0.35, 0.08, 0.08]
    assert dock['showCovariance'] is False
    assert scene['topics']['/camera/front/depth/points']['colorField'] == 'z'
    for camera in ('front', 'gripper'):
        assert any(c.get('imageMode', {}).get('imageTopic') == f'/camera/{camera}/image_raw/compressed'
                   for c in CONFIGS.values())
    for removed in ('left', 'right', 'back'):
        assert not any(c.get('imageMode', {}).get('imageTopic') == f'/camera/{removed}/image_raw/compressed'
                       for c in CONFIGS.values())
    assert CONFIGS['RawMessages!Detected-objects']['topicPath'] == '/z70/objects/json'
    assert CONFIGS['RawMessages!Camera-state']['topicPath'] == '/z70/camera/state'
    assert CONFIGS['Indicator!Active-lens']['path'] == '/z70/camera/state.selected_camera'
    assert CONFIGS['3D!Arm-closeup']['followTf'] == 'base_footprint'


def test_spatial_context_is_visible_in_overview():
    assert CONFIGS['RawMessages!Robot-status']['topicPath'] == '/z70/state'
    assert CONFIGS['RawMessages!Spatial-context']['topicPath'] == '/z70/map/spatial_context'
    overview = CONFIGS['Tab!z70-dashboard']['tabs'][0]['layout']
    assert 'RawMessages!Spatial-context' in json.dumps(overview)


def test_floor_mosaic_is_visible_in_odometry_tab():
    mosaic = CONFIGS['Image!Floor-mosaic']['imageMode']
    assert mosaic['imageTopic'] == '/z70/floor_mosaic'
    assert mosaic['imageSchemaName'] == 'sensor_msgs/msg/Image'
    odometry = CONFIGS['Tab!z70-dashboard']['tabs'][3]['layout']
    assert 'Image!Floor-mosaic' in json.dumps(odometry)
    trail = CONFIGS['3D!Odometry-trail']
    assert trail['cameraState']['distance'] == 4.5
    assert trail['topics']['/z70/map/dock_pose']['visible'] is False
