"""Build the layout using upstream Foxglove SDK panel configuration types.

Run with Python 3.12 after downloading upstream layouts/__init__.py as
foxglove_layout_types.py onto PYTHONPATH (see README Schema sources).
This generator emits the widely-supported exported-layout/mosaic envelope.
"""
import json
from pathlib import Path
import foxglove_layout_types as f

configs = {}

def panel(kind, name, config):
    key = kind + '!' + name
    configs[key] = json.loads(config.to_json())
    configs[key]['title'] = name.replace('-', ' ')
    return key

def split(direction, first, second, percent=50):
    return dict(direction=direction, first=first, second=second, splitPercentage=percent)

def row(a, b, percent=50): return split('row', a, b, percent)
def col(a, b, percent=50): return split('column', a, b, percent)

def raw(name, topic):
    return panel('RawMessages', name, f.RawMessagesConfig(topic_path=topic, default_expanded=True, font_size=12))

def image(name, camera):
    return panel('Image', name, f.ImageConfig(image_mode=f.ImageModeConfig(
        image_topic=f'/camera/{camera}/image_raw/compressed',
        calibration_topic=f'/camera/{camera}/camera_info',
        image_schema_name='sensor_msgs/msg/CompressedImage', synchronize=False)))

def mosaic_image(name, topic):
    return panel('Image', name, f.ImageConfig(image_mode=f.ImageModeConfig(
        image_topic=topic, calibration_topic='/z70/camera_info',
        image_schema_name='sensor_msgs/msg/Image', synchronize=False)))


def plot(name, series, unit='', bounds=None):
    palette = ['#38bdf8', '#a78bfa', '#34d399', '#fb923c']
    return panel('Plot', name, f.PlotConfig(
        paths=[f.PlotSeries(value=path + '.data', label=label, color=palette[i % len(palette)],
            enabled=True, timestamp_method='receiveTime', show_line=True, line_size=2)
            for i,(path,label) in enumerate(series)],
        show_legend=True, legend_display='top', show_plot_values_in_legend=True,
        show_x_axis_labels=True, show_y_axis_labels=True, x_axis_val='timestamp',
        following_view_width=60, is_synced=True, y_axis_label=unit,
        min_y_value=bounds[0] if bounds else None, max_y_value=bounds[1] if bounds else None))

def indicator(name, topic, states):
    return panel('Indicator', name, f.IndicatorConfig(path=topic + '.data', style='background',
        font_size=20, fallback_label='UNKNOWN / unavailable', fallback_color='#475569',
        rules=[f.IndicatorRule(raw_value=value, operator='=', label=label, color=color)
               for value,label,color in states]))

def scene(name, overview=True, distance=None, follow_tf='odom'):
    dist = (8.0 if overview else 4.5) if distance is None else distance
    topics = {'/z70/odom_path': f.BaseRendererPosesTopicSettings(visible=True, type='line',
                  gradient=('#38bdf8','#34d399'), line_width=.02),
              '/odom': f.BaseRendererPoseTopicSettings(visible=True, type='arrow', color='#38bdf8',
                  arrow_scale=(.3,.05,.05), show_covariance=False),
              '/map': f.BaseRendererRosOccupancyGridTopicSettings(visible=overview, color_mode='map', alpha=.7, frame_locked=True),
              '/z70/map/markers': f.BaseRendererRosMarkerTopicSettings(visible=overview),
              '/z70/objects': f.BaseRendererRosMarkerTopicSettings(visible=True),
              '/z70/floor_mosaic': f.BaseRendererImageTopicSettings(visible=False,
                  camera_info_topic='/z70/camera_info', distance=.01, frame_locked=True),
              '/z70/floor_mosaic_mask': f.BaseRendererImageTopicSettings(visible=False,
                  camera_info_topic='/z70/camera_info', distance=.01, frame_locked=True),
              '/camera/front/depth/points': f.BaseRendererPointCloudTopicSettings(visible=overview, point_size=2,
                  color_mode='colormap', color_field='z', color_map='turbo', min_value=0, max_value=2),
              '/z70/map/robot_pose': f.BaseRendererPoseTopicSettings(visible=overview, type='arrow',
                  color='#fbbf24', arrow_scale=(.3,.05,.05), show_covariance=False),
              '/z70/map/dock_pose': f.BaseRendererPoseTopicSettings(visible=overview, type='arrow',
                  color='#34d399', arrow_scale=(.35,.08,.08), show_covariance=False)}
    return panel('3D', name, f.ThreeDeeConfig(
        fixed_frame='map' if overview else 'odom', follow_tf=follow_tf, follow_mode='follow-none',
        camera_state=f.ThreeDeeCameraState(distance=dist, perspective=True, phi=50, theta_offset=135,
            target=(0,0,0), target_offset=(0,0,0), target_orientation=(0,0,0,1), near=.01, far=100, fovy=45),
        scene=f.BaseRendererSceneSettings(background_color='#101827', mesh_up_axis='z_up',
            transforms=f.BaseRendererTransforms(visible=False, show_label=False, axis_size=.05, editable=False)),
        topics=topics,
        layers={
            'z70-grid': f.BaseRendererGridLayerSettings(instance_id='z70-grid', layer_id='foxglove.Grid',
                label='Floor · 1 m grid', visible=True, frame_id='odom', size=10, divisions=10, color='#334155', line_width=1),
            'z70-urdf': f.BaseRendererUrdfLayerSettings(instance_id='z70-urdf', layer_id='foxglove.Urdf',
                label='Saros Z70 · estimated arm', visible=True, source_type='topic', topic='/robot_description',
                display_mode='visual', opacity=1, show_outlines=True)}))

battery = panel('Gauge', 'Battery-%', f.GaugeConfig(path='/z70/battery.data', style='bar',
                min_value=0, max_value=100, color_mode='colormap', color_map='red-yellow-green',
                show_ticks=True, tick_interval=20))
state = indicator('Robot-state', '/z70/state_name', [(n,n,c) for n,c in [
    ('IDLE','#0f766e'), ('CHARGER_DISCONNECTED','#0e7490'), ('CHARGING','#0f766e'),
    ('CHARGING_COMPLETE','#0f766e'), ('STARTING','#0e7490'), ('CLEANING','#0369a1'),
    ('SEGMENT_CLEANING','#0369a1'), ('ZONED_CLEANING','#0369a1'), ('SPOT_CLEANING','#0369a1'),
    ('GOING_TO_TARGET','#0369a1'), ('PATROL','#0369a1'), ('RETURNING_HOME','#b45309'),
    ('DOCKING','#b45309'), ('ARM_RESETTING','#b45309'), ('REMOTE_PICKUP','#6d28d9'),
    ('REMOTE_CONTROL_ACTIVE','#6d28d9'), ('PROGRAM_MODE','#6d28d9'), ('MANUAL_MODE','#6d28d9'),
    ('ERROR','#b91c1c'), ('CHARGING_PROBLEM','#b91c1c'), ('DEVICE_OFFLINE','#b91c1c'),
    ('PAUSED','#a16207')]])
connection = indicator('Robot-link', '/z70/connected', [('true','CONNECTED','#0f766e'), ('false','DISCONNECTED','#b91c1c')])
error = indicator('Error', '/z70/error_code', [('0','NO ERROR','#0f766e')])
summary = panel('DiagnosticSummary', 'Health', f.DiagnosticsSummaryConfig(topic_to_render='/diagnostics',
                min_level=0, sort_by_level=True, seconds_until_stale=5))
overview = row(scene('Robot-map-and-depth'), col(image('Front-live','front'),
    col(row(battery, state), col(row(connection,error),row(raw('Robot-status','/z70/state'),
        raw('Spatial-context','/z70/map/spatial_context'),50),35),30),47),62)

active_lens = panel('Indicator', 'Active-lens', f.IndicatorConfig(
    path='/z70/camera/state.selected_camera', style='background', font_size=18,
    fallback_label='UNKNOWN LENS', fallback_color='#475569',
    rules=[f.IndicatorRule(raw_value='1', operator='=', label='FRONT LENS · 1600×1200', color='#0f766e'),
           f.IndicatorRule(raw_value='2', operator='=', label='GRIPPER LENS · 1600×896', color='#6d28d9')]))
cameras = col(
    row(image('Front-camera','front'), image('Gripper-camera','gripper'), 55),
    row(raw('Detected-objects','/z70/objects/json'),
        col(active_lens, row(raw('Camera-state','/z70/camera/state'),
                             raw('Front-metadata','/camera/front/metadata'), 50), 28), 55), 62)
arm_state = indicator('Arm-estimate-state','/z70/arm/state',[(n,n,c) for n,c in [
    ('UNKNOWN','#b45309'),('IN_ROBOT','#475569'),('WORKING_POS','#0f766e'),
    ('IN_PROCESS_POS','#6d28d9'),('TRANSFER_POS','#0e7490')]])
arm = row(scene('Arm-closeup', overview=False, distance=1.1, follow_tf='base_footprint'),
          row(col(plot('App-joint-estimates',[(f'/z70/arm/m{i}_deg',f'M{i}') for i in range(3,7)],'degrees'),
                  plot('Joint-uncertainty',[(f'/z70/arm/sigma_m{i}_deg',f'M{i} sigma') for i in range(3,6)],'sigma · degrees')),
              col(arm_state,col(raw('UNKNOWN-reason','/z70/arm/unknown_reason'),raw('Arm-source-detail','/z70/arm_estimate')),25),58),38)
cal = '/z70/calibration/'
odometry = row(col(scene('Odometry-trail',False),mosaic_image('Floor-mosaic','/z70/floor_mosaic'),62),col(
    plot('VO-accept-reject',[(cal+'vo_accepted','accepted'),(cal+'vo_rejected','rejected')],'frames'),
    col(plot('Chassis-gains',[(cal+'yaw_positive_gain','yaw +'),(cal+'yaw_negative_gain','yaw −')],'gain'),
        col(plot('Camera-x',[(cal+'camera_x_m','fit'),(cal+'active_camera_x_m','active'),(cal+'camera_x_sigma_m','fit sigma')],'meters'),
            raw('Calibration-source','/z70/chassis_calibration'),65)),30),50)
diag = row(col(summary,panel('DiagnosticStatusPanel','Robot-diagnostics',f.DiagnosticsDetailConfig(
    selected_hardware_id='Saros Z70', selected_name='Z70/Robot', topic_to_render='/diagnostics',seconds_until_stale=5)),42),
    col(raw('Diagnostics-raw','/diagnostics'),col(plot('Battery-history',[('/z70/battery','battery')],'%',(0,100)),
       plot('Last-RC-command-age',[('/z70/last_command/age_s','age')],'seconds')),50),60)
configs['Tab!z70-dashboard'] = dict(activeTabIdx=0, tabs=[dict(title=t,layout=l) for t,l in [
    ('Overview',overview),('Cameras',cameras),('Arm',arm),('Odometry & calibration',odometry),('Diagnostics',diag)]])
layout = dict(configById=configs, globalVariables={}, userNodes={}, playbackConfig={'speed':1},layout='Tab!z70-dashboard')
Path(__file__).with_name('z70-dashboard.json').write_text(json.dumps(layout,indent=2,allow_nan=False)+'\n')
print(f'Generated {len(configs)} panel configs in z70-dashboard.json')
