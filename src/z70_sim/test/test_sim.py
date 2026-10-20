"""Offline integration and unit tests for the MuJoCo Z70 simulator and ROS 2 driver bridge."""
from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from z70_driver.driver import Driver
from z70_interfaces.action import PatrolCollect
from z70_interfaces.msg import FrameMetadata
from z70_manipulation import perception, planner as placement
from z70_manipulation.executor import Executor
from z70_manipulation.garment_delivery import TowingQualificationConfig, loaded_route
from z70_manipulation.perception import GarmentTracker, geofence, observe
from z70_manipulation.garment_node import observations_message
from z70_manipulation.garment_node import GarmentModel
from z70_manipulation.goals import BUILTIN_GOALS, GOAL_CONTEXT_PRIMITIVES, GoalContext
from z70_manipulation.observations import Observations
from z70_manipulation.patrol import PatrolCollector, PatrolLedger
from z70_mapping.boundary import BoundaryConfig, compute_safe_interior
from z70_mapping.decoder import decode_map
from z70_mapping.dynamic_layer import SessionWorldMap
from z70_mapping.node import dynamic_grid_message, world_markers_message
from z70_mapping.node import MappingNode
from z70_odometry.node import FloorCamera, FloorOdometry
from z70_sim import (
    APARTMENT_ROOMS,
    DEFAULT_GARMENTS,
    WASHING_MACHINE_DROP_M,
    SimWorld,
    create_sim_server,
    encode_v1_map,
)
from z70_sim.server import ensure_token_file
from z70_web_gateway.gateway import read_token


def wait_for(predicate, seconds: float = 5.0) -> None:
    deadline = time.monotonic() + seconds
    while not predicate():
        assert time.monotonic() < deadline, 'condition deadline'
        time.sleep(0.02)


class DirectSimClient:
    def __init__(self, w: SimWorld, obs_adapter=None):
        self.w = w
        self.observations = obs_adapter
        self.commands = (
            NS(rpc=self.rpc, stop=lambda: None, cancelled=NS(is_set=lambda: False))
            if obs_adapter is not None
            else None
        )
        self.frame_received_at = 0.0
        self.frame_exposure_known = False

    def status(self):
        return self.w.status_dict()

    def arm_estimate(self):
        if self.observations is not None:
            return self.observations.arm_estimate()
        return {
            'state': 'WORKING_POS' if self.w.deployed else 'IN_ROBOT',
            'joints': dict(self.w.joints),
        }

    def frame(self):
        jpeg, cam, _, _ = self.w.render_jpeg()
        self.frame_received_at = time.time()
        return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cam

    def select(self, camera: str):
        self.w.camera = camera
        return {'camera': camera}

    def rpc(self, method: str, params, timeout_ms: int = 5000):
        kind, val = self.w.handle_rpc(method, params)
        if kind == 'ok':
            return {'dispatch': 'sent', 'response': 'received', 'result': val}
        return {'dispatch': 'unknown', 'response': 'transport_error', 'transportReason': 'timeout'}


def test_mujoco_cameras_marker_68_floor_vo_and_v1_map():
    world = SimWorld()

    # 1. Front camera HSV object detection and floor projection
    jpeg_f, cam_f, wf, hf = world.render_jpeg('front')
    assert (cam_f, wf, hf) == ('front', 1600, 1200)
    img_f = cv2.imdecode(np.frombuffer(jpeg_f, np.uint8), cv2.IMREAD_COLOR)
    objs = perception.front_objects(img_f)
    colors = {o['color']: o for o in objs}
    assert {'yellow', 'purple', 'blue'} <= set(colors)
    assert colors['yellow']['position'][0] == pytest.approx(0.64, abs=0.05)
    assert colors['yellow']['position'][1] == pytest.approx(0.0, abs=0.02)

    # 2. Deploy arm, move to grasp pose (250, 95, 0), and verify ArUco ID68 & joint fit
    world.handle_rpc('app_pick_up_enter_exit', {'flag': 1})
    world.handle_rpc('app_arm_out_compartment', {})
    world.joints['m3'], world.joints['m4'], world.joints['m5'] = 250.0, 95.0, 0.0
    jpeg_arm, _, _, _ = world.render_jpeg('front')
    img_arm = cv2.imdecode(np.frombuffer(jpeg_arm, np.uint8), cv2.IMREAD_COLOR)
    jaw = perception.front_jaw(img_arm)
    assert jaw['marker_id'] == 68 and jaw['rms_px'] < 0.5
    fit = placement.observed_joints(jaw['hand_pose_base'])
    assert fit is not None
    assert fit['joints'] == pytest.approx([250.0, 95.0, 0.0], abs=1.0)

    # 3. Switch to gripper camera at WORKING_POS and verify 1600x896 rendering
    world.joints['m3'], world.joints['m4'], world.joints['m5'] = 150.0, 45.0, 0.0
    kind, _ = world.handle_rpc('app_change_camera', {'flag': 1})
    assert kind == 'timeout' and world.camera == 'gripper'
    jpeg_g, cam_g, wg, hg = world.render_jpeg()
    assert (cam_g, wg, hg) == ('gripper', 1600, 896)
    img_g = cv2.imdecode(np.frombuffer(jpeg_g, np.uint8), cv2.IMREAD_COLOR)
    assert any(b['color'] == 'yellow' for b in perception.detect(img_g))

    # 4. Floor visual odometry over textured MuJoCo floor
    world.handle_rpc('app_change_camera', {'flag': 0})
    world.handle_rpc('app_arm_in_compartment', {})
    jpeg_0, _, _, _ = world.render_jpeg('front')
    world.handle_rpc('app_rc_move', [{'velocity': 0.06, 'omega': 0.0, 'duration': 500, 'seqnum': 1}])
    jpeg_1, _, _, _ = world.render_jpeg('front')
    vo = FloorOdometry(FloorCamera.load(), camera_x_m=0.275)
    motion = vo.estimate(
        cv2.imdecode(np.frombuffer(jpeg_0, np.uint8), cv2.IMREAD_COLOR),
        cv2.imdecode(np.frombuffer(jpeg_1, np.uint8), cv2.IMREAD_COLOR),
    )
    assert motion is not None
    assert motion.dx == pytest.approx(0.06 * 0.65 * 0.5, abs=0.003)
    assert motion.inliers >= 200

    # 5. V1 map encoding and decoding
    raw = encode_v1_map(robot_pose=(0.25, -0.10, 0.5))
    decoded = decode_map(raw)
    assert (decoded.width, decoded.height) == (120, 120)
    assert sorted(decoded.rooms) == [1, 2]
    assert decoded.robot[:2] == pytest.approx((0.25, -0.10), abs=0.005)


def test_closed_loop_pick_against_mujoco_world(tmp_path: Path):
    world = SimWorld()
    client = DirectSimClient(world)

    def observe(camera: str, color: str):
        img, tag = client.frame()
        blobs = perception.front_objects(img) if tag == 'front' else perception.detect(img)
        return [b for b in blobs if b['color'] == color]

    ex = Executor(
        client,
        observe,
        tmp_path / 'pick_run',
        sleep=lambda _: None,
        odom=lambda: {'x': world.pose[0], 'y': world.pose[1], 'yaw': world.pose[2]},
    )
    result = ex.pick('yellow')
    assert result['verified'] is True
    assert not world.deployed


def test_sim_server_and_ros_driver_integration(tmp_path: Path):
    token_file = tmp_path / 'service-token'
    token_file.write_text('sim-test-bearer-token\n')
    server = create_sim_server(token='sim-test-bearer-token')
    rclpy.init(domain_id=97)
    driver = Driver(
        [
            Parameter('origin', value=server.origin),
            Parameter('token_file', value=str(token_file)),
            Parameter('arm_estimate_path', value=''),
            Parameter('camera_hz', value=2.0),
            Parameter('state_hz', value=5.0),
            Parameter('map_refresh_s', value=60.0),
        ]
    )
    mapping = MappingNode()
    consumer = rclpy.create_node('sim_test_consumer')
    obs = Observations(consumer, timeout=4.0)
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (driver, mapping, consumer):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        wait_for(lambda: 'state' in driver.samples and 'camera' in driver.samples and 'map' in driver.samples)
        status = obs.status()
        assert status['state'] == 3 and status['battery'] == 100
        img, tag = obs.frame()
        assert tag == 'front' and img.shape == (1200, 1600, 3)
        nav_snap = obs.navigation_snapshot(refresh=False)
        assert nav_snap.active_floor_verified and nav_snap.current_pose_verified
        assert nav_snap.labels == {
            1: 'Corridor',
            2: 'Kitchen',
            4: 'Living room',
            5: 'Bedroom',
            6: 'Bathroom',
        }

        # Deploy arm via driver_rpc and switch to gripper camera via SelectCamera
        driver.driver_rpc('app_pick_up_enter_exit', {'flag': 2}, 3000)
        driver.driver_rpc('app_arm_out_compartment', {}, 3000)
        def select_gripper():
            try:
                obs.select('gripper')
                return True
            except LookupError as exc:
                if str(exc) == 'BUSY':
                    return False
                raise
        wait_for(select_gripper)
        wait_for(lambda: server.world.camera == 'gripper')
    finally:
        if obs is not None:
            obs.close()
        driver.close()
        executor.shutdown()
        for node in (consumer, mapping, driver):
            node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=3.0)
        server.stop()


def test_patrol_collect_and_drag_garments_to_washing_machine_in_sim(tmp_path: Path):
    world = SimWorld(objects=DEFAULT_GARMENTS, map_preset='apartment')
    model = GarmentModel('sim_hsv')
    tracker = GarmentTracker()
    labels = {r['segment_id']: r['name'] for r in APARTMENT_ROOMS}
    seq = 0
    last_pose: tuple[float, float, float] | None = None

    class SimGarmentObservations:
        def __init__(self, w: SimWorld):
            self.w = w
            self.world = SessionWorldMap()
            self.last_msg = None
            self.last_img = None
            self.last_pose_used = None
            self.last_frame_seq = None

        def status(self):
            return self.w.status_dict()

        def arm_estimate(self):
            return dict(
                state='IN_PROCESS_POS' if self.w.deployed else 'IN_ROBOT',
                joints=dict(self.w.joints),
            )

        def world_map(self) -> SessionWorldMap:
            return self.world

        def dynamic_obstacles(self, pose_xy=None):
            if self.world.width > 0 and self.world.height > 0:
                return self.world.obstacle_mask(protect_xy=pose_xy)
            return None

        def record_world_event(self, action: str, **fields):
            self.world.apply_event(dict(action=action, **fields))

        def navigation_snapshot(self, refresh: bool = False, max_age_s: float = 5.0):
            raw, mseq, _ = self.w.raw_map(refresh=refresh)
            decoded = decode_map(raw)
            self.world.bind_session(b'\x01' * 16, 0, decoded.origin, decoded.width, decoded.height, decoded.resolution)
            return NS(
                map=decoded,
                labels=labels,
                sequence=mseq,
                map_index=0,
                map_sequence=mseq,
                session_id=b'\x01' * 16,
                active_floor_verified=True,
                current_pose_verified=True,
                cache_age_s=0.0,
                evidence=None,
                dynamic_obstacles=self.dynamic_obstacles(decoded.robot[:2] if decoded.robot else None),
                world=self.world,
            )

        def garment_barrier(self):
            return dict(garment=seq, image=seq)

        def _step_perception(self):
            nonlocal seq, last_pose
            seq += 1
            jpeg, _, w, h = self.w.render_jpeg('front')
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            detections, quality = model.infer(img)
            snap = self.navigation_snapshot(refresh=False)
            b = compute_safe_interior(snap.map, snap.labels, BoundaryConfig(), tuple(self.w.pose), dynamic_obstacles=snap.dynamic_obstacles)
            grid = NS(origin=b.origin, resolution=b.resolution, safe_grid=b.safe_grid)
            pose = (float(self.w.pose[0]), float(self.w.pose[1]), float(self.w.pose[2]))
            if last_pose is None:
                delta = (0.0, 0.0, 0.0)
            else:
                ox, oy, oyaw = last_pose
                dx, dy = pose[0] - ox, pose[1] - oy
                delta = (
                    math.cos(oyaw) * dx + math.sin(oyaw) * dy,
                    -math.sin(oyaw) * dx + math.cos(oyaw) * dy,
                    math.atan2(math.sin(pose[2] - oyaw), math.cos(pose[2] - oyaw)),
                )
            last_pose = pose
            items = []
            for d in detections:
                item = observe(
                    d['mask'],
                    self.w.front_cam,
                    category=d['category'],
                    confidence=d['confidence'],
                    camera_x_m=0.275,
                    calibration_id='sim-front',
                    qualified_policy=dict(qualified_small_lift=True, allow_tow=True),
                )
                if item['contact_valid']:
                    item['inside_geofence'], item['geofence_verified'] = geofence(
                        item['floor_contact_m'], pose, grid
                    )
                items.append(item)
            items = tracker.update(items, delta)
            for item in items:
                item['reachable_for_grasp'] = bool(
                    item['reachable_for_grasp']
                    and item['confirmed']
                    and item['inside_geofence']
                    and item['geofence_verified']
                )
            meta = FrameMetadata()
            meta.width, meta.height = w, h
            meta.calibration_id = 'sim-front'
            meta.evidence.valid = True
            meta.evidence.sequence = seq
            meta.evidence.session_id.uuid = [1] * 16
            msg = observations_message(items, meta, quality, True)
            self.world.ingest_observations(msg, pose)
            self.last_msg = msg
            self.last_img = img
            return img, msg

        def garment_snapshot(self, after=None, max_age_s: float = 6.0):
            prior = after.get('garment') if isinstance(after, dict) else after
            if self.last_msg is not None and prior is None and self.last_pose_used == tuple(self.w.pose):
                return self.last_msg
            _, msg = self._step_perception()
            if not any(g.reachable_for_grasp for g in msg.garments) and msg.garments:
                _, msg = self._step_perception()
            self.last_msg = msg
            self.last_pose_used = tuple(self.w.pose)
            return msg

        def garment_frame(self, after=None):
            prior = after.get('image') if isinstance(after, dict) else after
            if (
                self.last_msg is not None
                and (prior is None or self.last_msg.evidence.sequence != prior)
                and self.last_frame_seq != self.last_msg.evidence.sequence
                and self.last_pose_used == tuple(self.w.pose)
            ):
                img, msg = self.last_img, self.last_msg
            else:
                img, msg = self._step_perception()
                self.last_msg = msg
                self.last_pose_used = tuple(self.w.pose)
            self.last_frame_seq = msg.evidence.sequence
            assert img is not None
            visible = float(img.mean()) > 8.0 and float(np.percentile(img, 95) - np.percentile(img, 5)) > 12.0
            return dict(
                image=img,
                source_id=('sim', msg.evidence.sequence),
                evidence=msg.evidence,
                coverage_valid=visible,
            )

    obs = SimGarmentObservations(world)
    client = DirectSimClient(world, obs)
    ex = Executor(
        client,
        lambda *_: [],
        tmp_path / 'patrol_exec',
        sleep=lambda _: None,
        odom=lambda: {'x': world.pose[0], 'y': world.pose[1], 'yaw': world.pose[2]},
        calibration={'forward': {'gain': 0.65}, 'yaw_positive': {'gain': 0.70}, 'yaw_negative': {'gain': 0.65}},
    )
    ex.max_transit_pulse_ms = 5000.0  # type: ignore[attr-defined]
    ledger = PatrolLedger(tmp_path / 'patrol_ledger')
    towing_cfg = TowingQualificationConfig(
        towing_qualified=True,
        qualified_categories=('towel', 'shirt', 'pants'),
        artifact_uri='sim:mujoco-qualified',
    )
    patrol = PatrolCollector(
        obs,
        ex,
        ledger,
        boundary_config=BoundaryConfig(allowed_segments=(1, 2, 4, 5, 6)),
        towing_config=towing_cfg,
        sleep=lambda _: None,
    )
    goal = NS(
        drop_x_m=WASHING_MACHINE_DROP_M[0],
        drop_y_m=WASHING_MACHINE_DROP_M[1],
        max_duration_s=600.0,
        max_items=2,
        allow_towing=True,
        dry_run=False,
    )
    res = patrol.execute(goal)
    assert res['items_attempted'] == 2
    assert res['items_delivered'] == 2
    assert res['items_failed'] == 0
    assert all(item['status'] == 'DELIVERED' for item in ledger.data['items'].values())
    for g in DEFAULT_GARMENTS:
        pos = world._get_object_pos(str(g['name']))
        assert math.dist(pos[:2], WASHING_MACHINE_DROP_M) <= 0.40
    assert not world.deployed
    summary = obs.world.summary()
    assert len(summary['completed']) == 2
    assert len(summary['by_mobility']['pickupable']) >= 1
    assert len(summary['by_mobility']['draggable']) >= 1


def test_session_dynamic_map_opportunistic_sighting_obstacle_avoidance_and_reset(tmp_path: Path):
    world = SimWorld(objects=DEFAULT_GARMENTS, map_preset='apartment')
    labels = {r['segment_id']: r['name'] for r in APARTMENT_ROOMS}
    raw, _, _ = world.raw_map(refresh=True)
    m = decode_map(raw)
    wm = SessionWorldMap()
    assert wm.bind_session(b'\x01' * 16, 0, m.origin, m.width, m.height, m.resolution)

    # 1. Opportunistic sighting while performing another task:
    # Suppose the robot spotted a pickupable sock in the Bedroom (x=1.55, y=3.65)
    # and a draggable towel in the Corridor (x=1.18, y=-0.075) while driving past.
    wm.upsert_object(1.55, 3.65, category='sock', mobility='pickupable', relevance='objective', width_m=0.05, depth_m=0.06, track_id='bed_sock')
    wm.upsert_object(1.18, -0.075, category='towel', mobility='draggable', relevance='objective', width_m=0.06, depth_m=0.22, track_id='corridor_towel')
    assert len(wm.pending_objectives()) == 2
    assert len(wm.pending_objectives(allowed_mobility=('pickupable',))) == 1

    # Verify PatrolCollector prioritizes the waypoint near the sighted Bedroom pickupable objective (tier 0)
    # when allow_towing=False (ignoring the closer draggable Corridor towel), and prioritizes the closer
    # Corridor draggable towel when allow_towing=True.
    ledger = PatrolLedger(tmp_path / 'priority_ledger')
    b0 = compute_safe_interior(m, labels, BoundaryConfig(allowed_segments=(1, 2, 4, 5, 6)), (0.0, 0.0, 0.0))
    ledger.waypoints(b0.waypoints)
    patrol = PatrolCollector(NS(world_map=lambda: wm), None, ledger, world=wm)
    patrol.position = [0.0, 0.0]
    best_no_tow = min(ledger.data['waypoints'].keys(), key=lambda k: patrol._waypoint_priority(k, allow_towing=False))
    assert ledger.data['waypoints'][best_no_tow]['segment_id'] == 5
    best_with_tow = min(ledger.data['waypoints'].keys(), key=lambda k: patrol._waypoint_priority(k, allow_towing=True))
    assert ledger.data['waypoints'][best_with_tow]['segment_id'] == 1

    # 2. Stationary / deferred obstacle & dynamic obstacle avoidance:
    # Place a stationary obstacle on the direct corridor path between (0.0, 0.0) and Kitchen drop (-3.85, -0.30).
    base_loaded = compute_safe_interior(
        m, labels, BoundaryConfig(allowed_segments=(1, 2, 4, 5, 6), deployed_arm_extension_m=0.10), (0.0, 0.0, 0.0)
    )
    direct_route = loaded_route(base_loaded, (0.0, 0.0), WASHING_MACHINE_DROP_M)
    assert direct_route is not None and len(direct_route) > 4
    mid_pt = direct_route[len(direct_route) // 2]
    wm.mark_obstacle(mid_pt[0], mid_pt[1], radius_m=0.12, category='shoe_rack', mobility='stationary', relevance='ignored', detail='STATIONARY_OBSTACLE')

    dyn_grid = dynamic_grid_message(m, wm, 1000)
    assert max(dyn_grid.data) == 100
    markers = world_markers_message(wm, 1000)
    assert any(mk.ns == 'world_obstacles' for mk in markers.markers)
    assert any(mk.ns == 'world_objectives' for mk in markers.markers)

    updated_loaded = compute_safe_interior(
        m, labels, BoundaryConfig(allowed_segments=(1, 2, 4, 5, 6), deployed_arm_extension_m=0.10),
        (0.0, 0.0, 0.0), dynamic_obstacles=wm.obstacle_mask(protect_xy=(0.0, 0.0))
    )
    gx = int(math.floor((mid_pt[0] - m.origin[0]) / m.resolution))
    gy = int(math.floor((mid_pt[1] - m.origin[1]) / m.resolution))
    assert not updated_loaded.reachable_mask[gy, gx]

    # 3. GoalContext integration & session reset:
    fake_obs = NS(
        world_map=lambda: wm,
        dynamic_obstacles=lambda pose_xy=None: wm.obstacle_mask(protect_xy=pose_xy),
        record_world_event=lambda action, **kw: wm.apply_event(dict(action=action, **kw)),
        navigation_snapshot=lambda refresh=False, max_age_s=5.0: NS(
            map=m, labels=labels, evidence=None, dynamic_obstacles=wm.obstacle_mask(protect_xy=(0.0, 0.0))
        ),
    )
    fake_server = NS(
        observations=fake_obs,
        navigation_config=BoundaryConfig(allowed_segments=(1, 2, 4, 5, 6)),
        gains={},
        odom=lambda: {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    )
    ctx = GoalContext(
        fake_server,
        NS(request=PatrolCollect.Goal(), is_cancel_requested=False),
        PatrolCollect,
        None,
        tmp_path,
        create_executor=False,
    )
    assert len(ctx.world_objects()['obstacles']) == 1
    assert len(ctx.world_objects()['objectives']) == 2
    assert len(ctx.world_objects()['by_mobility']['stationary']) == 1
    _, ctx_boundary = ctx.safe_interior()
    assert not ctx_boundary.reachable_mask[gy, gx]

    # Session change automatically clears the session-scoped world map
    assert wm.bind_session(b'\x02' * 16, 0, m.origin, m.width, m.height, m.resolution)
    assert len(ctx.world_objects()['entries']) == 0
    _, clean_boundary = ctx.safe_interior()
    assert clean_boundary.reachable_mask[gy, gx]
    assert {'world_objects', 'update_world_object', 'mark_obstacle', 'clear_world'} <= {p['name'] for p in GOAL_CONTEXT_PRIMITIVES}

    # 4. Simulated session edge case: patrol with allow_towing=False skips the large towel,
    # records it as a SKIPPED_OBSTACLE in the session world map, and delivers the small sock.
    obs_no_tow = NS(
        world_map=lambda: wm,
        dynamic_obstacles=lambda pose_xy=None: wm.obstacle_mask(protect_xy=pose_xy),
        record_world_event=lambda action, **kw: wm.apply_event(dict(action=action, **kw)),
        navigation_snapshot=lambda refresh=False, max_age_s=5.0: NS(
            map=decode_map(world.raw_map(refresh=refresh)[0]), labels=labels, sequence=1, map_index=0, map_sequence=1, session_id=b'\x02' * 16,
            active_floor_verified=True, current_pose_verified=True, cache_age_s=0.0, evidence=None,
            dynamic_obstacles=wm.obstacle_mask(protect_xy=tuple(world.pose[:2])), world=wm,
        ),
        status=world.status_dict,
        arm_estimate=lambda: dict(state='IN_ROBOT', joints=dict(world.joints)),
        garment_barrier=lambda: dict(garment=1, image=1),
        garment_frame=lambda **_: dict(coverage_valid=True),
    )
    towel_obs = NS(
        track_id=NS(uuid=list(b'\xaa' * 16)),
        category='towel',
        contact_valid=True,
        inside_geofence=True,
        geofence_verified=True,
        grasp_valid=True,
        reachable_for_grasp=True,
        transport_suitability=2,
        footprint_width_m=0.22,
        footprint_depth_m=0.06,
        floor_contact_m=NS(x=0.35, y=0.0),
    )
    obs_no_tow.garment_snapshot = lambda **_: NS(
        garments=[towel_obs],
        model_available=True,
        geofence_active=True,
        evidence=NS(session_id=NS(uuid=[2] * 16), sequence=2),
    )
    ex_stub = NS(
        client=NS(commands=NS(
            rpc=lambda method, params, timeout_ms=5000: (
                world.handle_rpc(method, params) and {'dispatch': 'sent', 'response': 'received', 'result': ['ok']}
            ),
            stop=lambda: None,
            cancelled=NS(is_set=lambda: False),
        )),
        gripped=False,
        hazard=lambda: None,
        odom=lambda: {'x': world.pose[0], 'y': world.pose[1], 'yaw': world.pose[2]},
        end_rc=lambda: None,
        pulse=lambda **_: None,
    )
    skip_ledger = PatrolLedger(tmp_path / 'skip_ledger')
    skip_patrol = PatrolCollector(
        obs_no_tow,
        ex_stub,
        skip_ledger,
        boundary_config=BoundaryConfig(allowed_segments=(1, 2, 4, 5, 6), waypoint_spacing_m=20.0),
        sleep=lambda _: None,
        world=wm,
    )
    skip_res = skip_patrol.execute(NS(
        drop_x_m=WASHING_MACHINE_DROP_M[0],
        drop_y_m=WASHING_MACHINE_DROP_M[1],
        max_duration_s=60.0,
        max_items=2,
        allow_towing=False,
        dry_run=False,
    ))
    assert skip_res['items_skipped'] == 1
    assert len(wm.obstacle_entries()) == 1
    obs_entry = wm.obstacle_entries()[0]
    assert obs_entry.mobility == 'draggable'
    assert obs_entry.relevance == 'deferred'
    assert obs_entry.is_obstacle is True
    assert obs_entry.detail == 'UNSUPPORTED_GARMENT_SKIP'


def test_ensure_token_file_matches_gateway_token_contract(tmp_path: Path):
    path = tmp_path / '.local/service-token'
    token = ensure_token_file(path)
    assert len(token) == 43
    assert read_token(str(path)) == token
    assert ensure_token_file(path) == token
