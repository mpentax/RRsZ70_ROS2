"""Generate z70-dashboard.json for Foxglove Studio (ws://127.0.0.1:8765)."""
import json
from pathlib import Path

configs = {}


def panel(kind, name, config):
    key = f'{kind}!{name}'
    configs[key] = {**config, 'title': name.replace('-', ' ')}
    return key


def split(direction, first, second, percent=50):
    return dict(direction=direction, first=first, second=second, splitPercentage=percent)


def row(a, b, percent=50): return split('row', a, b, percent)
def col(a, b, percent=50): return split('column', a, b, percent)


def raw(name, topic):
    return panel('RawMessages', name, dict(
        diffEnabled=False, diffMethod='custom', diffTopicPath='',
        showFullMessageForDiff=False, topicPath=topic, fontSize=12, defaultExpanded=True,
    ))


def image(name, camera):
    return panel('Image', name, dict(imageMode=dict(
        imageTopic=f'/camera/{camera}/image_raw/compressed',
        calibrationTopic=f'/camera/{camera}/camera_info',
        synchronize=False, flipHorizontal=False, flipVertical=False, rotation=0,
        annotations={}, colorMode='gradient', gradient=['#000000', '#ffffff'],
        colorMap='turbo', valueMin=0, valueMax=1,
        imageSchemaName='sensor_msgs/msg/CompressedImage',
    )))


def mosaic_image(name, topic):
    return panel('Image', name, dict(imageMode=dict(
        imageTopic=topic, calibrationTopic='/z70/camera_info',
        synchronize=False, flipHorizontal=False, flipVertical=False, rotation=0,
        annotations={}, colorMode='gradient', gradient=['#000000', '#ffffff'],
        colorMap='turbo', valueMin=0, valueMax=1,
        imageSchemaName='sensor_msgs/msg/Image',
    )))


def plot(name, series, unit='', bounds=None):
    palette = ['#38bdf8', '#a78bfa', '#34d399', '#fb923c']
    cfg = dict(
        paths=[dict(
            value=path + '.data', enabled=True, timestampMethod='receiveTime',
            label=label, color=palette[i % len(palette)], lineSize=2, showLine=True,
        ) for i, (path, label) in enumerate(series)],
        showXAxisLabels=True, showYAxisLabels=True, showLegend=True,
        legendDisplay='top', showPlotValuesInLegend=True, isSynced=True,
        xAxisVal='timestamp', followingViewWidth=60, yAxisLabel=unit,
    )
    if bounds:
        cfg['minYValue'], cfg['maxYValue'] = bounds
    return panel('Plot', name, cfg)


def indicator(name, topic, states, font_size=20, fallback='UNKNOWN / unavailable', suffix='.data'):
    return panel('Indicator', name, dict(
        path=topic + suffix, style='background', fontSize=font_size,
        rules=[dict(rawValue=value, operator='=', color=color, label=label) for value, label, color in states],
        fallbackColor='#475569', fallbackLabel=fallback,
    ))


def scene(name, overview=True, distance=None, follow_tf='odom'):
    dist = (8.0 if overview else 4.5) if distance is None else distance
    topics = {
        '/z70/odom_path': dict(visible=True, type='line', lineWidth=.02, gradient=['#38bdf8', '#34d399']),
        '/odom': dict(visible=True, type='arrow', arrowScale=[.3, .05, .05], color='#38bdf8', showCovariance=False),
        '/map': dict(visible=overview, colorMode='map', alpha=.7, frameLocked=True),
        '/z70/map/safe_grid': dict(visible=False, colorMode='map', alpha=.4, frameLocked=True),
        '/z70/map/dynamic_obstacles': dict(visible=False, colorMode='map', alpha=.5, frameLocked=True),
        '/z70/map/markers': dict(visible=overview),
        '/z70/world/markers': dict(visible=overview),
        '/z70/garments/markers': dict(visible=overview),
        '/z70/objects': dict(visible=True),
        '/z70/floor_mosaic': dict(visible=False, cubeOutline=False, pointShape='circle', frameLocked=True, cameraInfoTopic='/z70/camera_info', distance=.01),
        '/z70/floor_mosaic_mask': dict(visible=False, cubeOutline=False, pointShape='circle', frameLocked=True, cameraInfoTopic='/z70/camera_info', distance=.01),
        '/camera/front/depth/points': dict(visible=overview, pointSize=2, cubeOutline=False, pointShape='circle', colorMode='colormap', colorField='z', colorMap='turbo', minValue=0, maxValue=2),
        '/z70/map/robot_pose': dict(visible=overview, type='arrow', arrowScale=[.3, .05, .05], color='#fbbf24', showCovariance=False),
        '/z70/map/dock_pose': dict(visible=overview, type='arrow', arrowScale=[.35, .08, .08], color='#34d399', showCovariance=False),
    }
    return panel('3D', name, dict(
        scene=dict(
            backgroundColor='#101827', meshUpAxis='z_up',
            transforms=dict(visible=False, editable=False, showLabel=False, axisSize=.05),
        ),
        layers={
            'z70-grid': dict(visible=True, instanceId='z70-grid', layerId='foxglove.Grid', label='Floor · 1 m grid', frameId='odom', size=10, divisions=10, lineWidth=1, color='#334155'),
            'z70-urdf': dict(visible=True, instanceId='z70-urdf', layerId='foxglove.Urdf', label='Saros Z70 · estimated arm', sourceType='topic', topic='/robot_description', displayMode='visual', showOutlines=True, opacity=1),
        },
        topics=topics,
        cameraState=dict(distance=dist, perspective=True, phi=50, target=[0, 0, 0], targetOffset=[0, 0, 0], targetOrientation=[0, 0, 0, 1], thetaOffset=135, fovy=45, near=.01, far=100),
        followMode='follow-none', followTf=follow_tf, fixedFrame='map' if overview else 'odom',
    ))


battery = panel('Gauge', 'Battery-%', dict(
    path='/z70/battery.data', style='bar', minValue=0, maxValue=100,
    tickInterval=20, showTicks=True, colorMode='colormap', colorMap='red-yellow-green', mirrorColorAtOrigin=False,
))
state = indicator('Robot-state', '/z70/state_name', [(n, n, c) for n, c in [
    ('IDLE', '#0f766e'), ('CHARGER_DISCONNECTED', '#0e7490'), ('CHARGING', '#0f766e'),
    ('CHARGING_COMPLETE', '#0f766e'), ('STARTING', '#0e7490'), ('CLEANING', '#0369a1'),
    ('SEGMENT_CLEANING', '#0369a1'), ('ZONED_CLEANING', '#0369a1'), ('SPOT_CLEANING', '#0369a1'),
    ('GOING_TO_TARGET', '#0369a1'), ('PATROL', '#0369a1'), ('RETURNING_HOME', '#b45309'),
    ('DOCKING', '#b45309'), ('ARM_RESETTING', '#b45309'), ('REMOTE_PICKUP', '#6d28d9'),
    ('REMOTE_CONTROL_ACTIVE', '#6d28d9'), ('PROGRAM_MODE', '#6d28d9'), ('MANUAL_MODE', '#6d28d9'),
    ('ERROR', '#b91c1c'), ('CHARGING_PROBLEM', '#b91c1c'), ('DEVICE_OFFLINE', '#b91c1c'),
    ('PAUSED', '#a16207'),
]])
connection = indicator('Robot-link', '/z70/connected', [('true', 'CONNECTED', '#0f766e'), ('false', 'DISCONNECTED', '#b91c1c')])
error = indicator('Error', '/z70/error_code', [('0', 'NO ERROR', '#0f766e')])
summary = panel('DiagnosticSummary', 'Health', dict(minLevel=0, topicToRender='/diagnostics', sortByLevel=True, secondsUntilStale=5))
overview = row(scene('Robot-map-and-depth'), col(image('Front-live', 'front'),
    col(row(battery, state), col(row(connection, error), row(raw('Robot-status', '/z70/state'),
        raw('Spatial-context', '/z70/map/spatial_context'), 50), 35), 30), 47), 62)

active_lens = indicator(
    'Active-lens', '/z70/camera/state.selected_camera',
    [('1', 'FRONT LENS · 1600×1200', '#0f766e'), ('2', 'GRIPPER LENS · 1600×896', '#6d28d9')],
    font_size=18, fallback='UNKNOWN LENS', suffix='',
)
cameras = col(
    row(image('Front-camera', 'front'), image('Gripper-camera', 'gripper'), 55),
    row(raw('Detected-objects', '/z70/objects/json'),
        col(active_lens, row(raw('Camera-state', '/z70/camera/state'),
                             raw('Front-metadata', '/camera/front/metadata'), 50), 28), 55), 62)
arm_state = indicator('Arm-estimate-state', '/z70/arm/state', [(n, n, c) for n, c in [
    ('UNKNOWN', '#b45309'), ('IN_ROBOT', '#475569'), ('WORKING_POS', '#0f766e'),
    ('IN_PROCESS_POS', '#6d28d9'), ('TRANSFER_POS', '#0e7490'),
]])
arm = row(scene('Arm-closeup', overview=False, distance=1.1, follow_tf='base_footprint'),
          row(col(plot('App-joint-estimates', [(f'/z70/arm/m{i}_deg', f'M{i}') for i in range(3, 7)], 'degrees'),
                  plot('Joint-uncertainty', [(f'/z70/arm/sigma_m{i}_deg', f'M{i} sigma') for i in range(3, 6)], 'sigma · degrees')),
              col(arm_state, col(raw('UNKNOWN-reason', '/z70/arm/unknown_reason'), raw('Arm-source-detail', '/z70/arm_estimate')), 25), 58), 38)
cal = '/z70/calibration/'
odometry = row(col(scene('Odometry-trail', False), mosaic_image('Floor-mosaic', '/z70/floor_mosaic'), 62), col(
    plot('VO-accept-reject', [(cal + 'vo_accepted', 'accepted'), (cal + 'vo_rejected', 'rejected')], 'frames'),
    col(plot('Chassis-gains', [(cal + 'yaw_positive_gain', 'yaw +'), (cal + 'yaw_negative_gain', 'yaw −')], 'gain'),
        col(plot('Camera-x', [(cal + 'camera_x_m', 'fit'), (cal + 'active_camera_x_m', 'active'), (cal + 'camera_x_sigma_m', 'fit sigma')], 'meters'),
            raw('Calibration-source', '/z70/chassis_calibration'), 65)), 30), 50)
diag = row(col(summary, panel('DiagnosticStatusPanel', 'Robot-diagnostics', dict(
    selectedHardwareId='Saros Z70', selectedName='Z70/Robot', topicToRender='/diagnostics', secondsUntilStale=5,
)), 42),
    col(raw('Diagnostics-raw', '/diagnostics'), col(plot('Battery-history', [('/z70/battery', 'battery')], '%', (0, 100)),
       plot('Last-RC-command-age', [('/z70/last_command/age_s', 'age')], 'seconds')), 50), 60)
configs['Tab!z70-dashboard'] = dict(activeTabIdx=0, tabs=[dict(title=t, layout=l) for t, l in [
    ('Overview', overview), ('Cameras', cameras), ('Arm', arm), ('Odometry & calibration', odometry), ('Diagnostics', diag),
]])
layout = dict(configById=configs, globalVariables={}, userNodes={}, playbackConfig={'speed': 1}, layout='Tab!z70-dashboard')
Path(__file__).with_name('z70-dashboard.json').write_text(json.dumps(layout, indent=2, allow_nan=False) + '\n')
print(f'Generated {len(configs)} panel configs in z70-dashboard.json')
