"""ROS image/TF perception and single-owner manipulation/navigation action servers."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from z70_description.arm_model import check as check_arm_limits
from z70_interfaces.action import (
    DeliverGarment, PatrolCollect, PinAndGo, Pick, Stack,
    DeployArm, AlignToTarget, MoveArm, GraspAndLift, ReleaseAndStow,
)
from z70_interfaces.msg import ArmEstimate, CalibrationState, CommandOutcome
from z70_odometry.calibration_state import gains_from_state
from z70_driver.frame_subscription import MatchedFrames
from z70_mapping.boundary import BoundaryConfig
from . import perception, planner
from .executor import Client, log_directory
from .goals import GoalContext, GoalSpec, load_custom_goals
from .observations import Observations, LATCHED

RGB = {'yellow': (1., 1., 0.), 'orange': (1., .4, 0.), 'purple': (.7, 0., 1.), 'blue': (0., .2, 1.), 'green': (0., 1., .2)}


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Perception(Node):
    def __init__(self, name='z70_perception', parameter_overrides=None, subscribe=True):
        super().__init__(name, parameter_overrides=parameter_overrides or [])
        self.repo = Path(self.declare_parameter('repo', str(Path(os.environ.get('Z70_REPO', Path.cwd())))).value)
        config = self.declare_parameter('colors_file', '').value
        self.thresholds = yaml.safe_load(Path(config).read_text()) if config else perception.colors()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.arm_trusted = False
        self.create_subscription(ArmEstimate, '/z70/arm_estimate', self.arm, LATCHED)
        self.pub_json = self.create_publisher(String, '/z70/objects/json', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/z70/objects', 10)
        self.K = planner.GRIPPER_MATRIX.copy()
        self.D = planner.GRIPPER_D.copy()
        self.create_subscription(CameraInfo, '/camera/gripper/camera_info', self.info, 10)
        if subscribe:
            self.frames = [MatchedFrames(self, camera, lambda msg, c=camera: self.image(msg, c), compressed=True)
                           for camera in ('front', 'gripper')]

    def arm(self, msg):
        self.arm_trusted = msg.evidence.valid and not msg.evidence.stale and msg.validity.value in (1, 2) and not msg.restored

    def info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

    def project(self, image, camera, stamp=None):
        if camera == 'front':
            objects = perception.front_objects(image, thresholds=self.thresholds)
        else:
            objects = perception.detect(image, self.thresholds)
            if self.arm_trusted:
                tr = self.tf.lookup_transform('base_footprint', 'gripper_camera_optical_frame', stamp or Time(), timeout=Duration(seconds=.5))
                p = tr.transform.translation
                C = np.array([p.x, p.y, p.z])
                R = rotation(tr.transform.rotation)
                K = self.K.copy()
                K[0, :] *= image.shape[1] / 1600
                K[1, :] *= image.shape[0] / 896
                for b in objects:
                    point = perception.ray_floor(b['contact_px'], K, self.D, C, R)
                    if point:
                        b['position'] = point
                        b['size_m'] = .05
                        b['camera_distance_m'] = float(np.linalg.norm(np.array(point) - C))
                        b['camera_height_m'] = float(C[2])
            for b in objects:
                b['camera'] = 'gripper'
        self.publish(objects, camera, stamp)
        return objects

    def publish(self, objects, camera, stamp=None):
        now = self.get_clock().now().to_msg()
        self.pub_json.publish(String(data=json.dumps(dict(camera=camera, frame_id='base_footprint', stamp_ns=self.get_clock().now().nanoseconds, objects=objects))))
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = 'base_footprint'
        clear.header.stamp = now
        markers = [clear]
        for index, b in enumerate(objects):
            if 'position' not in b:
                continue
            m = Marker()
            m.header.frame_id = 'base_footprint'
            m.header.stamp = now
            m.ns = camera
            m.id = index
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, b['position'])
            m.pose.position.z += b.get('size_m', .05) / 2
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = float(b.get('size_m', .05))
            m.color.r, m.color.g, m.color.b = RGB[b['color']]
            m.color.a = .9
            m.text = b['color']
            markers.append(m)
        self.pub_markers.publish(MarkerArray(markers=markers))

    def image(self, msg, camera):
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        try:
            self.project(image, camera, Time.from_msg(msg.header.stamp))
        except TransformException:
            self.get_logger().warning('camera TF unavailable; next observation will retry', throttle_duration_sec=5)


class PickServer(Perception):
    def __init__(self, parameter_overrides=None):
        super().__init__('z70_pick_server', parameter_overrides, subscribe=False)
        self.log_root = self.declare_parameter('log_root', '').value
        self.safety_stop = False
        self.odom_value = None
        self.odom_at = 0.
        self.lock = threading.Lock()
        self.active_handle = None
        self.navigation_active = False
        self.monitor = None
        self.client = None
        self.gains = {}
        self.deadline = None
        self.create_subscription(Bool, '/z70/safety_stop', self.stop_signal, 10)
        self.create_subscription(Odometry, '/odom', self.odometry, 30)
        self.group = ReentrantCallbackGroup()
        self.observations = Observations(self)
        if hasattr(self.observations, 'node'):
            from .commands import Commands
            Commands(self.observations)
        self.observations.cancelled = lambda: self.client is not None and self.client.commands is not None and self.client.commands.cancelled.is_set()
        self.create_subscription(CalibrationState, '/z70/chassis_calibration', self.calibration, LATCHED)
        self.create_timer(.05, self.watchdog, callback_group=self.group)
        self.navigation_config = BoundaryConfig(allowed_segments=tuple(
            self.declare_parameter('pin_and_go_allowed_segments', [1, 2, 3, 4, 5, 6]).value))
        self.patrol_directory = Path(self.declare_parameter('patrol_ledger_directory', str(self.repo / '.local/patrol')).value)
        self.custom_goals_directory = Path(self.declare_parameter('custom_goals_directory', str(self.repo / '.local/goals')).value)

        self.action_servers = []
        self.goal_specs = {}
        self.action = self.register_goal(GoalSpec(
            'pick', Pick, '/z70/pick', 285.,
            lambda s, r: r.color in s.thresholds,
            self._goal_pick,
            fields=('color', 'dry_run'),
            description='Autonomous object pickup by HSV color with visual alignment, coordinated descent, grasp, lift, and stow.',
        ))
        self.stack_action = self.register_goal(GoalSpec(
            'stack', Stack, '/z70/stack', lambda r: 60. if r.dry_run else 540.,
            lambda s, r: len({r.base_color, r.middle_color, r.top_color}) == 3 and all(c in s.thresholds for c in (r.base_color, r.middle_color, r.top_color)),
            self._goal_stack,
            fields=('base_color', 'middle_color', 'top_color', 'dry_run'),
            description='Multi-tier 3-object stacking with visual alignment, controlled descent, in-place jaw release, and stability verification.',
        ))
        self.pin_and_go_action = self.register_goal(GoalSpec(
            'pin_and_go', PinAndGo, '/z70/nav/pin_and_go', lambda r: r.max_duration_s,
            lambda s, r: all(math.isfinite(v) and -100 <= v <= 100 for v in (r.target_x_m, r.target_y_m)) and math.isfinite(r.max_duration_s) and 1 <= r.max_duration_s <= 600,
            self._goal_pin_and_go,
            needs_executor=lambda _: False,
            navigation=True,
            fields=('target_x_m', 'target_y_m', 'max_duration_s', 'dry_run'),
            description='Interior-geofenced Pin-and-Go point navigation on the verified vendor SLAM map.',
        ))
        self.deliver_garment_action = self.register_goal(GoalSpec(
            'deliver_garment', DeliverGarment, '/z70/deliver_garment', lambda r: r.max_duration_s,
            self._valid_garment_goal,
            lambda ctx, r: self._goal_garment(ctx, r, DeliverGarment, patrol=False),
            needs_executor=lambda r: not r.dry_run,
            navigation=True,
            fields=('category', 'drop_x_m', 'drop_y_m', 'max_duration_s', 'allow_towing', 'dry_run'),
            description='Single-garment perception, staged deployment, pickup, loaded corridor transit, and drop-zone delivery.',
        ))
        self.patrol_collect_action = self.register_goal(GoalSpec(
            'patrol_collect', PatrolCollect, '/z70/patrol_collect', lambda r: r.max_duration_s,
            self._valid_garment_goal,
            lambda ctx, r: self._goal_garment(ctx, r, PatrolCollect, patrol=True),
            needs_executor=lambda r: not r.dry_run,
            navigation=True,
            fields=('drop_x_m', 'drop_y_m', 'max_duration_s', 'max_items', 'allow_towing', 'dry_run'),
            description='Autonomous multi-waypoint garment patrol, collection, and drop-zone delivery with persistent ledger state.',
        ))
        self.curated_actions = [
            self.register_goal(GoalSpec(
                'deploy_arm', DeployArm, '/z70/arm/deploy', 90.,
                lambda s, r: True,
                lambda ctx, r: self._goal_curated(
                    ctx, lambda ex: ex.deploy(settle=r.settle),
                    lambda outcome, ok, data: DeployArm.Result(outcome=outcome, estimate=ctx.arm_msg(), ready=bool(ok and data.get('ready'))),
                ),
                read_only=lambda _: False,
                fields=('settle',),
                description='Unstow the arm into WORKING_POS (150, 45, 0, 90) and optionally wait for obstruction check settle.',
            )),
            self.register_goal(GoalSpec(
                'align_target', AlignToTarget, '/z70/align_target', 120.,
                lambda s, r: r.color in s.thresholds,
                lambda ctx, r: self._goal_curated(
                    ctx, lambda ex: ex.align_front(r.color),
                    lambda outcome, ok, data: AlignToTarget.Result(
                        outcome=outcome,
                        aligned=bool(ok and 'position' in data),
                        target_position=[float(x) for x in data.get('position', [0., 0., 0.])[:3]],
                        bearing_error_deg=float(data.get('bearing_error_deg', 0.)),
                        range_error_m=float(data.get('range_error_m', 0.)),
                    ),
                ),
                read_only=lambda _: False,
                fields=('color',),
                description='Visually servo the chassis using front camera observations to center and range a target color.',
            )),
            self.register_goal(GoalSpec(
                'move_arm', MoveArm, '/z70/arm/move', 120.,
                lambda s, r: (all(math.isfinite(x) for x in (r.m3_deg, r.m4_deg, r.m5_deg))
                              and check_arm_limits(r.m3_deg, r.m4_deg, r.m5_deg) == 'SUCCESS'
                              and planner.safe((r.m3_deg, r.m4_deg, r.m5_deg, 90))),
                lambda ctx, r: self._goal_curated(
                    ctx, lambda ex: ex.move_arm((r.m3_deg, r.m4_deg, r.m5_deg)),
                    lambda outcome, ok, data: MoveArm.Result(
                        outcome=outcome,
                        estimate=ctx.arm_msg(),
                        steps_executed=int(data.get('steps_executed', 0)) if ok else 0,
                    ),
                ),
                read_only=lambda _: False,
                fields=('m3_deg', 'm4_deg', 'm5_deg'),
                description='Plan and execute collision-checked, coordinated multi-axis 10-degree arm steps to target joint angles.',
            )),
            self.register_goal(GoalSpec(
                'grasp_and_lift', GraspAndLift, '/z70/arm/grasp_and_lift', 90.,
                lambda s, r: 0 <= r.lift_steps <= 10 and (not r.verify_color or r.verify_color in s.thresholds),
                lambda ctx, r: self._goal_curated(
                    ctx, lambda ex: ex.grasp_and_lift(lift_steps=r.lift_steps, verify_color=r.verify_color),
                    lambda outcome, ok, data: GraspAndLift.Result(
                        outcome=outcome,
                        estimate=ctx.arm_msg(),
                        verified_hold=bool(ok and data.get('verified')),
                    ),
                ),
                read_only=lambda _: False,
                fields=('lift_steps', 'verify_color'),
                description='Close jaws, execute upward hand steps, and optionally verify target elevation in the front camera.',
            )),
            self.register_goal(GoalSpec(
                'release_and_stow', ReleaseAndStow, '/z70/arm/release_and_stow', 90.,
                lambda s, r: True,
                lambda ctx, r: self._goal_curated(
                    ctx, lambda ex: ex.release_and_stow(put_down=r.put_down),
                    lambda outcome, ok, data: ReleaseAndStow.Result(
                        outcome=outcome,
                        estimate=ctx.arm_msg(),
                        stowed=bool(ok and data.get('stowed')),
                    ),
                ),
                read_only=lambda _: False,
                fields=('put_down',),
                description='Optionally open jaws in place, stow the arm into the compartment, and anchor IN_ROBOT.',
            )),
        ]
        load_custom_goals(self, self.custom_goals_directory)

    @staticmethod
    def _valid_garment_goal(_server, request):
        return (all(math.isfinite(v) and -100 <= v <= 100 for v in (request.drop_x_m, request.drop_y_m))
                and math.isfinite(request.max_duration_s) and 1 <= request.max_duration_s <= 3600
                and (not hasattr(request, 'max_items') or 1 <= request.max_items <= 1000))

    def register_goal(self, spec: GoalSpec) -> ActionServer:
        server = ActionServer(
            self,
            spec.action_cls,
            spec.topic,
            execute_callback=lambda handle, sp=spec: self._run_goal(handle, sp),
            goal_callback=lambda req, sp=spec: GoalResponse.ACCEPT if sp.validate(self, req) and self.lock.acquire(blocking=False) else GoalResponse.REJECT,
            cancel_callback=self.cancel,
            callback_group=self.group,
        )
        self.action_servers.append(server)
        self.goal_specs[spec.name] = spec
        return server

    def _run_goal(self, handle, spec: GoalSpec):
        self.active_handle = handle
        self.navigation_active = spec.navigation
        client = None
        try:
            req = handle.request
            client = Client(read_only=spec.read_only(req), observations=self.observations)
            self.client = client
            timeout = spec.timeout_s(req) if callable(spec.timeout_s) else float(spec.timeout_s)
            self.deadline = time.monotonic() + timeout
            base_dir = log_directory(self.repo)
            directory = (Path(self.log_root) if spec.navigation else Path(self.log_root) / base_dir.name) if self.log_root else base_dir
            ctx = GoalContext(self, handle, spec.action_cls, client, directory, create_executor=spec.needs_executor(req))
            try:
                if client.commands:
                    client.commands.begin()
                success, result = spec.run(ctx, req)
                if handle.is_cancel_requested:
                    handle.canceled()
                elif success:
                    handle.succeed()
                else:
                    handle.abort()
                return result
            finally:
                if client.commands:
                    try:
                        client.commands.end()
                    except (LookupError, TimeoutError, InterruptedError):
                        pass
                client.close()
        except Exception as exc:
            self.get_logger().warning(f'{spec.name} failed: {type(exc).__name__}: {exc}')
            if spec.name == 'pin_and_go' and client and client.commands:
                try:
                    client.commands.stop()
                except (LookupError, TimeoutError):
                    pass
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            res = spec.action_cls.Result()
            if hasattr(res, 'outcome'):
                res.outcome = CommandOutcome(error_code='OBSERVATION_GAP')
            if hasattr(res, 'status_code'):
                res.status_code = 'OBSERVATION_GAP'
            if hasattr(res, 'detail'):
                res.detail = 'Shared driver observation or command unavailable'
            return res
        finally:
            self.navigation_active = False
            self.active_handle = None
            self.client = None
            self.deadline = None
            self.lock.release()

    # --- Goal Handlers ---

    def _goal_curated(self, ctx: GoalContext, fn, build_result):
        try:
            data = fn(ctx.executor)
            success = True
        except Exception as e:
            data = dict(error=type(e).__name__, reason=str(e))
            ctx.executor.log('failure', **data)
            success = False
        return success, build_result(ctx.outcome(success), success, data)

    def _goal_pick(self, ctx: GoalContext, req):
        try:
            report = ctx.executor.pick(req.color, dry_run=req.dry_run)
            success = bool(report.get('verified', report.get('dry_run') and report.get('target') is not None))
        except Exception as e:
            report = dict(error=type(e).__name__, reason=str(e), log_dir=str(ctx.directory))
            ctx.executor.log('failure', **report)
            success = False
        return success, Pick.Result(
            outcome=ctx.outcome(success),
            verified_hold=bool(report.get('verified')),
            dry_run_valid=bool(req.dry_run and success),
            evidence_uri=ctx.directory.name,
        )

    def _goal_stack(self, ctx: GoalContext, req):
        ctx.executor.feedback = lambda _phase: ctx.feedback(phase=3, current_tier=1, reason='NONE')
        try:
            report = ctx.executor.stack(req.base_color, req.middle_color, req.top_color, dry_run=req.dry_run)
            success = bool(report.get('stack_verified', report.get('dry_run') and report.get('all_targets_visible')))
        except Exception as e:
            report = dict(error=type(e).__name__, reason=str(e), log_dir=str(ctx.directory))
            ctx.executor.log('failure', **report)
            success = False
        tiers_done = 3 if (req.dry_run and success) else int(report.get('tiers_completed', 0))
        return success, Stack.Result(
            outcome=ctx.outcome(success),
            tiers_completed=tiers_done,
            stack_verified=bool(report.get('stack_verified')),
            dry_run_valid=bool(req.dry_run and success),
            evidence_uri=ctx.directory.name,
        )

    def _goal_pin_and_go(self, ctx: GoalContext, req):
        data = ctx.navigate(req.target_x_m, req.target_y_m, req.max_duration_s, dry_run=req.dry_run)
        result = PinAndGo.Result(**data)
        if ctx.commands and ctx.commands.latest_outcome is not None:
            result.outcome = ctx.commands.latest_outcome
        return bool(result.arrived or result.dry_run_valid), result

    def _goal_garment(self, ctx: GoalContext, req, action_cls, patrol=False):
        from .garment_delivery import GarmentDeliverer
        from .patrol import PatrolCollector, PatrolLedger
        if patrol:
            runner = PatrolCollector(
                self.observations, ctx.executor, PatrolLedger(self.patrol_directory),
                boundary_config=self.navigation_config, feedback=ctx.feedback,
            )
        else:
            runner = GarmentDeliverer(
                self.observations, ctx.executor,
                boundary_config=self.navigation_config, feedback=ctx.feedback,
            )
        # Keep runner's latest_observation synced onto ctx for feedback publishing
        orig_feedback = ctx.feedback
        def synced_feedback(**fields):
            ctx.latest_observation = runner.latest_observation
            orig_feedback(**fields)
        runner.feedback = synced_feedback
        data = runner.execute(req)
        success = (data.get('delivered', False) or data.get('completed', False) or data.get('dry_run_valid', False)
                   or data.get('status_code') == 'PARTIAL_COMPLETION_SMALL_ITEMS_ONLY')
        result = action_cls.Result(**data)
        result.outcome = ctx.outcome(success)
        return success, result

    # --- Node Helpers & Subscriptions ---

    def calibration(self, msg):
        self.gains = gains_from_state(msg)

    def watchdog(self):
        expired = self.deadline is not None and time.monotonic() >= self.deadline
        navigation_hazard = self.navigation_active and self.safety_stop
        if (expired or navigation_hazard) and self.client and self.client.commands and not self.client.commands.cancelled.is_set():
            self.client.commands.cancelled.set()
            threading.Thread(target=self.client.commands.stop, daemon=True).start()

    def cancel(self, handle):
        if self.active_handle is not None and handle != self.active_handle:
            return CancelResponse.REJECT
        if self.client and self.client.commands:
            self.client.commands.cancelled.set()
            threading.Thread(target=self.client.commands.stop, daemon=True).start()
        return CancelResponse.ACCEPT

    def stop_signal(self, msg):
        self.safety_stop = msg.data

    def odometry(self, msg):
        p = msg.pose.pose
        q = p.orientation
        if msg.pose.covariance[0] > 1000:
            return
        self.odom_value = dict(x=p.position.x, y=p.position.y, yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.odom_at = time.monotonic()

    def odom(self):
        return self.odom_value if time.monotonic() - self.odom_at < 2. else None

    def hazard(self):
        if self.safety_stop:
            return 'person/pet/obstacle or operator stop signal'
        if self.active_handle and self.active_handle.is_cancel_requested:
            return 'operator cancelled task'
        if self.client and self.client.commands and self.client.commands.cancelled.is_set():
            return 'task revoked'
        if self.deadline is not None and time.monotonic() >= self.deadline:
            return 'task deadline'
        return None

    def _current_arm_msg(self):
        with getattr(self.observations, 'condition', threading.Lock()):
            arm = getattr(self.observations, 'arm', None)
            return deepcopy(arm) if arm is not None else ArmEstimate()

    def map_pose(self, source='base_footprint'):
        tr = self.tf.lookup_transform('map', source, Time(), timeout=Duration(seconds=.5))
        age = (self.get_clock().now().nanoseconds - Time.from_msg(tr.header.stamp).nanoseconds) / 1e9
        if not 0 <= age <= 2.:
            raise LookupError('stale map -> odom -> base TF')
        p = tr.transform.translation
        q = tr.transform.rotation
        return (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def run(node_type):
    rclpy.init()
    node = node_type()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def perception_main():
    run(Perception)


def server_main():
    run(PickServer)
