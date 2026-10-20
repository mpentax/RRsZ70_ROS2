"""Composable ROS 2 Goal execution context and declarative GoalSpec registry.

Provides LLM-authored and curated ROS 2 goals with a single typed context (`GoalContext`)
for reading robot state, arm kinematics, map/spatial context, and object/garment perception,
and for invoking navigation, base pulses, and multi-axis arm primitives.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
import math
from pathlib import Path
import re
from types import SimpleNamespace
from typing import Any, Callable
import time

from tf2_ros import TransformException
from z70_description.arm_model import check as check_arm_limits
from z70_interfaces.action import (
    DeliverGarment, PatrolCollect, PinAndGo, Pick, Stack,
    DeployArm, AlignToTarget, MoveArm, GraspAndLift, ReleaseAndStow,
    BasePulse, ArmStep, HandStep, ArmOperation,
)
from z70_interfaces.msg import CommandOutcome
from z70_mapping.boundary import compute_safe_interior
from . import planner
from .executor import Executor
from .navigation import PinAndGoNavigator

COLOR_RE = re.compile(r'[a-zA-Z0-9_-]{1,32}')


def _int_in(value, low, high) -> bool:
    return type(value) is int and low <= value <= high


def _num_in(value, low, high) -> bool:
    return type(value) in (int, float) and math.isfinite(value) and low <= value <= high


def _is_color(value) -> bool:
    return isinstance(value, str) and bool(COLOR_RE.fullmatch(value))


GOAL_CONTEXT_PRIMITIVES: tuple[dict[str, str], ...] = (
    {'name': 'status', 'signature': 'status() -> dict', 'summary': 'Read the latest robot status telemetry dict.'},
    {'name': 'arm', 'signature': 'arm() -> dict', 'summary': 'Read the current arm joint and pose estimate dict.'},
    {'name': 'map_pose', 'signature': "map_pose(source='base_footprint') -> tuple[float, float, float]", 'summary': 'Lookup (x_m, y_m, yaw_rad) of a frame in the map.'},
    {'name': 'safe_interior', 'signature': 'safe_interior(refresh=False, deployed_arm_extension_m=0.)', 'summary': 'Return (map_snapshot, safe_interior_grid) for navigation and corridor planning.'},
    {'name': 'spatial_context', 'signature': 'spatial_context(refresh: bool = False) -> dict', 'summary': 'Return ego-centric map context: current_room, rooms (centroids/distances/bearings), dock pose, and 8-sector clearance_sectors_m.'},
    {'name': 'plan_route', 'signature': 'plan_route(target_x_m: float, target_y_m: float, deployed_arm_extension_m: float = 0., refresh: bool = False) -> list[tuple[float, float]] | None', 'summary': 'Plan a collision-checked A* route on the eroded safe interior from the current robot pose to (target_x_m, target_y_m).'},
    {'name': 'objects', 'signature': "objects(camera='front', color=None) -> list[dict]", 'summary': 'Detect and project HSV objects in base_footprint from a fresh camera frame.'},
    {'name': 'wrist_pose', 'signature': 'wrist_pose() -> dict | None', 'summary': 'Estimate the elevated ArUco ID68 wrist/jaw pose and grasp reference in base_footprint from the front camera.'},
    {'name': 'garments', 'signature': "garments(category='any', after=None) -> list", 'summary': 'Return verified reachable garments from the garment observation pipeline.'},
    {'name': 'navigate', 'signature': 'navigate(target_x_m: float, target_y_m: float, max_duration_s: float = 60., dry_run: bool = False) -> dict', 'summary': 'Navigate the stowed chassis to (target_x_m, target_y_m) via PinAndGo.'},
    {'name': 'pick', 'signature': 'pick(color: str, dry_run: bool = False) -> dict', 'summary': 'Execute a full autonomous object pickup by HSV color.'},
    {'name': 'stack', 'signature': 'stack(base_color: str, middle_color: str, top_color: str, dry_run: bool = False) -> dict', 'summary': 'Execute a 3-object stack by HSV color.'},
    {'name': 'deliver_garment', 'signature': "deliver_garment(category: str = 'any', drop_x_m: float = 0., drop_y_m: float = 0., max_duration_s: float = 300., allow_towing: bool = False, dry_run: bool = False) -> dict", 'summary': 'Detect, grasp, and deliver a single floor garment to the drop zone.'},
    {'name': 'patrol_collect', 'signature': 'patrol_collect(drop_x_m: float = 0., drop_y_m: float = 0., max_duration_s: float = 600., max_items: int = 5, allow_towing: bool = False, dry_run: bool = False) -> dict', 'summary': 'Patrol safe interior waypoints to collect and deliver garments to the drop zone.'},
    {'name': 'deploy_arm', 'signature': 'deploy_arm(settle=True) -> dict', 'summary': 'Enter manual arm mode and unstow the arm to WORKING_POS.'},
    {'name': 'align_target', 'signature': 'align_target(color: str) -> dict', 'summary': 'Visually servo the chassis toward a target color using front camera observations.'},
    {'name': 'arm_safe', 'signature': 'arm_safe(m3_deg: float, m4_deg: float, m5_deg: float = 0., m6_deg: float = 90.) -> bool', 'summary': 'Check whether an arm joint state (m3, m4, m5, m6) satisfies firmware limits and CAD floor/body collision bounds.'},
    {'name': 'plan_grasp', 'signature': 'plan_grasp(x_m: float, y_m: float, size_m: float = 0.066) -> dict | None', 'summary': 'Compute the closest reachable (M3, M4) grasp state and coordinated descent steps for a floor point (x_m, y_m) in base_footprint.'},
    {'name': 'move_arm', 'signature': 'move_arm(m3_deg: float, m4_deg: float, m5_deg: float = 0.) -> dict', 'summary': 'Execute a collision-checked coordinated multi-axis arm trajectory.'},
    {'name': 'arm_step', 'signature': 'arm_step(m3: int = 0, m4: int = 0, m5: int = 0) -> dict', 'summary': 'Execute a single 10-degree multi-axis joint step [m3, m4, m5] (0=hold, 1=+10deg, 2=-10deg).'},
    {'name': 'hand_step', 'signature': 'hand_step(direction: int) -> dict', 'summary': 'Execute a single 40 mm Cartesian IK hand step (1=up, 2=down, 3=forward, 4=back).'},
    {'name': 'open_jaws', 'signature': 'open_jaws()', 'summary': 'Open gripper jaws in place without lifting or stowing the arm.'},
    {'name': 'grasp_and_lift', 'signature': "grasp_and_lift(lift_steps: int = 2, verify_color: str = '') -> dict", 'summary': 'Close the gripper jaws, step the hand upward, and verify grasp elevation.'},
    {'name': 'release_and_stow', 'signature': 'release_and_stow(put_down: bool = True) -> dict', 'summary': 'Open jaws in place, stow the arm into the robot compartment, and anchor IN_ROBOT.'},
    {'name': 'pulse', 'signature': 'pulse(velocity: float = 0., omega: float = 0., duration: int = 300)', 'summary': 'Send a bounded linear or angular chassis velocity pulse.'},
    {'name': 'rpc', 'signature': 'rpc(method: str, params=None, timeout_ms: int = 5000) -> dict', 'summary': 'Invoke a raw vendor RPC method through the ROS driver scheduler.'},
    {'name': 'feedback', 'signature': 'feedback(**fields)', 'summary': 'Publish ROS 2 action feedback with latest observation and outcome stamps.'},
    {'name': 'outcome', 'signature': 'outcome(success: bool) -> CommandOutcome', 'summary': 'Build the terminal CommandOutcome for the goal result.'},
)

GOAL_DESCRIPTIONS: dict[str, str] = {
    'pick': 'Autonomous object pickup by HSV color with visual alignment, coordinated descent, grasp, lift, and stow.',
    'stack': 'Multi-tier 3-object stacking with visual alignment, controlled descent, in-place jaw release, and stability verification.',
    'pin_and_go': 'Interior-geofenced Pin-and-Go point navigation on the verified vendor SLAM map.',
    'deliver_garment': 'Single-garment perception, staged deployment, pickup, loaded corridor transit, and drop-zone delivery.',
    'patrol_collect': 'Autonomous multi-waypoint garment patrol, collection, and drop-zone delivery with persistent ledger state.',
    'deploy_arm': 'Unstow the arm into WORKING_POS (150, 45, 0, 90) and optionally wait for obstruction check settle.',
    'align_target': 'Visually servo the chassis using front camera observations to center and range a target color.',
    'move_arm': 'Plan and execute collision-checked, coordinated multi-axis 10-degree arm steps to target joint angles.',
    'grasp_and_lift': 'Close jaws, execute upward hand steps, and optionally verify target elevation in the front camera.',
    'release_and_stow': 'Optionally open jaws in place, stow the arm into the compartment, and anchor IN_ROBOT.',
    'base_pulse': 'Execute a single bounded linear or angular chassis velocity pulse.',
    'arm_step': 'Execute a single 10-degree joint step on M3..M5 or a coordinated M3+M4 step (34/43).',
    'hand_step': 'Execute a single 40 mm Cartesian IK hand step (up, down, forward, back).',
    'arm_operation': 'Invoke a discrete arm mode or lifecycle operation (unstow, stow, recovery-open, grip, put-down, mode entry/exit, recover).',
}


@dataclass
class GoalSpec:
    """Declarative specification for a ROS 2 Action goal server."""
    name: str
    action_cls: type
    topic: str
    timeout_s: float | Callable[[Any], float]
    validate: Callable[[Any, Any], bool]
    run: Callable[['GoalContext | SimpleNamespace', Any], tuple[bool, Any]]
    read_only: Callable[[Any], bool] = lambda req: bool(getattr(req, 'dry_run', False))
    needs_executor: Callable[[Any], bool] = lambda req: True
    navigation: bool = False
    fields: tuple[str, ...] = ()
    description: str = ''
    validate_payload: Callable[[dict], bool] = lambda _: True

    def describe(self) -> dict:
        return {
            'name': self.name,
            'topic': self.topic,
            'action': self.action_cls.__name__,
            'fields': list(self.fields),
            'navigation': self.navigation,
            'description': self.description or GOAL_DESCRIPTIONS.get(self.name, ''),
        }


def simulate_goal(spec: GoalSpec, request, **overrides) -> tuple[bool, Any]:
    """Run `spec.run(ctx, request)` against a lightweight in-memory mock `GoalContext`."""
    defaults = dict(
        request=request,
        action_cls=spec.action_cls,
        directory=Path('.'),
        latest_observation=None,
        commands=None,
        executor=None,
        status=lambda: {'state': 3, 'error_code': 0, 'battery': 100, 'common_status': 51202},
        arm=lambda: {'state': 'IN_ROBOT', 'joints': {'m3': 270., 'm4': 180., 'm5': 0., 'm6': 0.}},
        arm_msg=lambda: None,
        odom=lambda: {'x': 0., 'y': 0., 'yaw': 0.},
        map_pose=lambda source='base_footprint': (0., 0., 0.),
        map_snapshot=lambda refresh=False, max_age_s=5.: None,
        safe_interior=lambda refresh=False, deployed_arm_extension_m=0.: (None, None),
        spatial_context=lambda refresh=False: {'robot_pose_map': {'x': 0., 'y': 0., 'yaw': 0.}, 'current_room': None, 'rooms': [], 'dock': None, 'clearance_sectors_m': {}},
        plan_route=lambda target_x_m, target_y_m, deployed_arm_extension_m=0., refresh=False: [(float(target_x_m), float(target_y_m))],
        objects=lambda camera='front', color=None: [],
        wrist_pose=lambda: {'position': [0.35, 0., 0.08], 'marker_id': 68},
        observe_color=lambda camera, color: [],
        garments=lambda category='any', after=None: [],
        navigate=lambda target_x_m, target_y_m, max_duration_s=60., dry_run=False: {
            'arrived': not dry_run, 'dry_run_valid': bool(dry_run), 'status_code': 'ARRIVED', 'detail': '',
        },
        pick=lambda color, dry_run=False: {
            'verified': not dry_run, 'dry_run': bool(dry_run), 'target': {'color': color},
        },
        stack=lambda base_color, middle_color, top_color, dry_run=False: {
            'stack_verified': not dry_run, 'dry_run': bool(dry_run), 'all_targets_visible': True, 'tiers_completed': 3,
        },
        deliver_garment=lambda category='any', drop_x_m=0., drop_y_m=0., max_duration_s=300., allow_towing=False, dry_run=False: {
            'delivered': not dry_run, 'dry_run_valid': bool(dry_run), 'status_code': 'DELIVERED', 'detail': '',
        },
        patrol_collect=lambda drop_x_m=0., drop_y_m=0., max_duration_s=600., max_items=5, allow_towing=False, dry_run=False: {
            'completed': not dry_run, 'dry_run_valid': bool(dry_run), 'status_code': 'COMPLETED', 'detail': '',
        },
        deploy_arm=lambda settle=True: {'ready': True},
        align_target=lambda color: {'position': [0.35, 0., 0.04], 'bearing_error_deg': 0., 'range_error_m': 0.},
        arm_safe=lambda m3_deg, m4_deg, m5_deg=0., m6_deg=90.: planner.safe((m3_deg, m4_deg, m5_deg, m6_deg)),
        plan_grasp=lambda x_m, y_m, size_m=.066: {'state': [250, 95], 'error_m': 0., 'steps': []},
        move_arm=lambda m3_deg, m4_deg, m5_deg=0.: {'steps_executed': 1},
        arm_step=lambda m3=0, m4=0, m5=0: {'m3': int(m3), 'm4': int(m4), 'm5': int(m5)},
        hand_step=lambda direction: {'direction': int(direction)},
        open_jaws=lambda: None,
        grasp_and_lift=lambda lift_steps=2, verify_color='': {'verified': True},
        release_and_stow=lambda put_down=True: {'stowed': True},
        pulse=lambda velocity=0., omega=0., duration=300: {'ok': True},
        rpc=lambda method, params=None, timeout_ms=5000: {'result': ['ok']},
        stop=lambda: None,
        hazard=lambda: None,
        is_cancelled=lambda: False,
        sleep=lambda duration: None,
        log=lambda stage, **fields: None,
        feedback=lambda **fields: None,
        outcome=lambda success: CommandOutcome(error_code='NONE' if success else 'OBSERVATION_GAP'),
    )
    defaults.update(overrides)
    return spec.run(SimpleNamespace(**defaults), request)


class GoalContext:
    """Unified execution context passed to every ROS 2 goal handler."""

    def __init__(self, server, handle, action_cls, client, directory: Path, create_executor=True):
        self.server = server
        self.handle = handle
        self.request = handle.request
        self.action_cls = action_cls
        self.client = client
        self.commands = client.commands if client else None
        self.observations = server.observations
        self.directory = directory
        self.latest_observation = None
        self.executor = (
            Executor(
                client,
                self.observe_color,
                directory,
                sleep=self.sleep,
                hazard=self.hazard,
                odom=self.odom,
                feedback=lambda phase: self.feedback(phase=3, reason='NONE'),
                calibration=server.gains,
            )
            if create_executor
            else None
        )

    # --- State & Perception ---

    def status(self) -> dict:
        return self.observations.status() if hasattr(self.observations, 'status') else self.observations.get('/api/state')['telemetry']['status']['value']

    def arm(self) -> dict:
        return self.observations.arm_estimate() if hasattr(self.observations, 'arm_estimate') else self.observations.get('/api/arm/estimate')

    def arm_msg(self):
        return self.server._current_arm_msg()

    def odom(self) -> dict | None:
        return self.server.odom()

    def map_pose(self, source='base_footprint') -> tuple[float, float, float]:
        return self.server.map_pose(source)

    def map_snapshot(self, refresh=False, max_age_s=5.):
        snap = self.observations.navigation_snapshot(refresh=refresh, max_age_s=max_age_s)
        self.latest_observation = snap.evidence
        return snap

    def safe_interior(self, refresh=False, deployed_arm_extension_m=0.):
        snap = self.map_snapshot(refresh=refresh)
        cfg = replace(self.server.navigation_config, deployed_arm_extension_m=deployed_arm_extension_m)
        return snap, compute_safe_interior(snap.map, snap.labels, cfg, snap.map.robot)

    def spatial_context(self, refresh: bool = False) -> dict:
        from z70_mapping.geometry import spatial_context as compute_spatial_context
        snap = self.map_snapshot(refresh=refresh)
        try:
            pose = self.map_pose()
        except LookupError:
            pose = snap.map.robot
        return compute_spatial_context(snap.map, snap.labels, pose)

    def plan_route(self, target_x_m: float, target_y_m: float, deployed_arm_extension_m: float = 0., refresh: bool = False) -> list[tuple[float, float]] | None:
        from .garment_delivery import loaded_route
        snap, boundary = self.safe_interior(refresh=refresh, deployed_arm_extension_m=deployed_arm_extension_m)
        if snap.map.robot is None:
            return None
        return loaded_route(boundary, snap.map.robot[:2], (float(target_x_m), float(target_y_m)))

    def frame(self):
        return self.client.frame()

    def objects(self, camera='front', color=None) -> list[dict]:
        image, tag = self.client.frame()
        if tag != camera:
            raise LookupError('frame camera tag mismatch')
        if camera == 'gripper':
            estimate = self.arm()
            gripped = bool(self.executor and self.executor.gripped)
            self.server.arm_trusted = not gripped and estimate.get('state') not in ('UNKNOWN', 'IN_ROBOT')
        try:
            detected = self.server.project(image, camera)
        except TransformException:
            raise LookupError('camera TF missing') from None
        return [b for b in detected if color is None or b['color'] == color]

    def observe_color(self, camera, color) -> list[dict]:
        return self.objects(camera=camera, color=color)

    def wrist_pose(self) -> dict | None:
        from . import perception
        image, tag = self.client.frame()
        if tag != 'front':
            raise LookupError('wrist_pose requires front camera frame')
        try:
            return perception.front_jaw(image)
        except LookupError:
            return None

    def garments(self, category='any', after=None) -> list:
        batch = self.observations.garment_snapshot(after=after)
        self.latest_observation = batch.evidence
        return [
            g for g in batch.garments
            if (category in ('', 'any') or g.category == category)
            and all(getattr(g, f) for f in ('reachable_for_grasp', 'inside_geofence', 'geofence_verified', 'grasp_valid', 'contact_valid'))
        ]

    # --- Motion & Manipulation Primitives ---

    def navigate(self, target_x_m: float, target_y_m: float, max_duration_s: float = 60., dry_run: bool = False) -> dict:
        nav = PinAndGoNavigator(
            self.observations,
            self.commands,
            boundary_config=self.server.navigation_config,
            cancelled=self.is_cancelled,
            hazard=lambda: self.server.safety_stop,
            odom=self.odom,
            feedback=self.feedback,
        )
        res = nav.execute(SimpleNamespace(
            target_x_m=float(target_x_m),
            target_y_m=float(target_y_m),
            max_duration_s=float(max_duration_s),
            dry_run=bool(dry_run),
        ))
        self.latest_observation = nav.latest_observation
        return res

    def pick(self, color: str, dry_run: bool = False) -> dict:
        return self.executor.pick(color, dry_run=dry_run)

    def stack(self, base_color: str, middle_color: str, top_color: str, dry_run: bool = False) -> dict:
        self.executor.feedback = lambda _phase: self.feedback(phase=3, current_tier=1, reason='NONE')
        return self.executor.stack(base_color, middle_color, top_color, dry_run=dry_run)

    def _run_garment_runner(self, runner, req) -> dict:
        orig_feedback = self.feedback
        def synced_feedback(**fields):
            self.latest_observation = runner.latest_observation
            orig_feedback(**fields)
        runner.feedback = synced_feedback
        data = runner.execute(req)
        self.latest_observation = runner.latest_observation
        return data

    def deliver_garment(self, category: str = 'any', drop_x_m: float = 0., drop_y_m: float = 0.,
                        max_duration_s: float = 300., allow_towing: bool = False, dry_run: bool = False) -> dict:
        from .garment_delivery import GarmentDeliverer
        runner = GarmentDeliverer(
            self.observations, self.executor,
            boundary_config=self.server.navigation_config, feedback=self.feedback,
        )
        return self._run_garment_runner(runner, SimpleNamespace(
            category=str(category), drop_x_m=float(drop_x_m), drop_y_m=float(drop_y_m),
            max_duration_s=float(max_duration_s), allow_towing=bool(allow_towing), dry_run=bool(dry_run),
        ))

    def patrol_collect(self, drop_x_m: float = 0., drop_y_m: float = 0., max_duration_s: float = 600.,
                       max_items: int = 5, allow_towing: bool = False, dry_run: bool = False) -> dict:
        from .patrol import PatrolCollector, PatrolLedger
        runner = PatrolCollector(
            self.observations, self.executor, PatrolLedger(self.server.patrol_directory),
            boundary_config=self.server.navigation_config, feedback=self.feedback,
        )
        return self._run_garment_runner(runner, SimpleNamespace(
            drop_x_m=float(drop_x_m), drop_y_m=float(drop_y_m), max_duration_s=float(max_duration_s),
            max_items=int(max_items), allow_towing=bool(allow_towing), dry_run=bool(dry_run),
        ))

    def deploy_arm(self, settle=True) -> dict:
        return self.executor.deploy(settle=settle)

    def align_target(self, color: str) -> dict:
        return self.executor.align_front(color)

    def arm_safe(self, m3_deg: float, m4_deg: float, m5_deg: float = 0., m6_deg: float = 90.) -> bool:
        return planner.safe((float(m3_deg), float(m4_deg), float(m5_deg), float(m6_deg)))

    def plan_grasp(self, x_m: float, y_m: float, size_m: float = .066) -> dict | None:
        joints = self.arm().get('joints') or {}
        start = (round(float(joints.get('m3', 190)) / 10) * 10, round(float(joints.get('m4', 65)) / 10) * 10)
        try:
            return planner.grasp_goal([float(x_m), float(y_m), 0.], start, size_m=float(size_m))
        except ValueError:
            return None

    def move_arm(self, m3_deg: float, m4_deg: float, m5_deg: float = 0.) -> dict:
        return self.executor.move_arm((m3_deg, m4_deg, m5_deg))

    def arm_step(self, m3: int = 0, m4: int = 0, m5: int = 0) -> dict:
        if any(v not in (0, 1, 2) for v in (m3, m4, m5)) or not any((m3, m4, m5)):
            raise ValueError('arm_step axes must be in (0, 1, 2) with at least one nonzero axis')
        return self.rpc('app_arm_move', {'m': [int(m3), int(m4), int(m5)]}, timeout_ms=3000)

    def hand_step(self, direction: int) -> dict:
        if direction not in (1, 2, 3, 4):
            raise ValueError('hand_step direction must be in (1, 2, 3, 4)')
        return self.rpc('app_arm_direction_move', {'direction': int(direction)}, timeout_ms=3000)

    def open_jaws(self):
        self.executor.open_jaws()

    def grasp_and_lift(self, lift_steps: int = 2, verify_color: str = '') -> dict:
        return self.executor.grasp_and_lift(lift_steps=lift_steps, verify_color=verify_color)

    def release_and_stow(self, put_down: bool = True) -> dict:
        return self.executor.release_and_stow(put_down=put_down)

    def pulse(self, velocity: float = 0., omega: float = 0., duration: int = 300):
        return self.executor.pulse(velocity=velocity, omega=omega, duration=duration)

    def rpc(self, method: str, params=None, timeout_ms: int = 5000) -> dict:
        return self.client.rpc(method, [] if params is None else params, timeout_ms=timeout_ms)

    def stop(self):
        if self.commands:
            self.commands.stop()

    def log(self, stage: str, **fields):
        if self.executor:
            self.executor.log(stage, **fields)

    # --- Lifecycle, Cancellation & Feedback ---

    def hazard(self):
        return self.server.hazard()

    def is_cancelled(self) -> bool:
        return bool(self.handle.is_cancel_requested or (self.commands and self.commands.cancelled.is_set()))

    def sleep(self, duration: float):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            if self.hazard():
                raise InterruptedError('CANCELLED_OR_DEADLINE')
            time.sleep(min(.05, max(0., until - time.monotonic())))

    def feedback(self, **fields):
        msg = self.action_cls.Feedback(**fields)
        if hasattr(msg, 'latest_observation') and self.latest_observation is not None:
            msg.latest_observation = self.latest_observation
        if hasattr(msg, 'latest_outcome') and self.commands and self.commands.latest_outcome is not None:
            msg.latest_outcome = self.commands.latest_outcome
        self.handle.publish_feedback(msg)

    def outcome(self, success: bool) -> CommandOutcome:
        if self.commands and self.commands.latest_outcome is not None:
            return deepcopy(self.commands.latest_outcome)
        return CommandOutcome(error_code='NONE' if success else 'OBSERVATION_GAP')


# --- Canonical Built-in Goal Handlers & Registry ---

def _valid_garment_goal(_server, request) -> bool:
    return (all(math.isfinite(v) and -100 <= v <= 100 for v in (request.drop_x_m, request.drop_y_m))
            and math.isfinite(request.max_duration_s) and 1 <= request.max_duration_s <= 3600
            and (not hasattr(request, 'max_items') or 1 <= request.max_items <= 1000))


def _goal_curated(ctx: 'GoalContext | SimpleNamespace', fn, build_result):
    try:
        data = fn(ctx)
        success = True
    except Exception as e:
        data = dict(error=type(e).__name__, reason=str(e))
        ctx.log('failure', **data)
        success = False
    return success, build_result(ctx.outcome(success), success, data)


def _goal_pick(ctx: 'GoalContext | SimpleNamespace', req):
    try:
        report = ctx.pick(req.color, dry_run=req.dry_run)
        success = bool(report.get('verified') or (report.get('dry_run') and report.get('target') is not None))
    except Exception as e:
        report = dict(error=type(e).__name__, reason=str(e), log_dir=str(ctx.directory))
        ctx.log('failure', **report)
        success = False
    return success, Pick.Result(
        outcome=ctx.outcome(success),
        verified_hold=bool(report.get('verified')),
        dry_run_valid=bool(req.dry_run and success),
        evidence_uri=ctx.directory.name,
    )


def _goal_stack(ctx: 'GoalContext | SimpleNamespace', req):
    try:
        report = ctx.stack(req.base_color, req.middle_color, req.top_color, dry_run=req.dry_run)
        success = bool(report.get('stack_verified') or (report.get('dry_run') and report.get('all_targets_visible')))
    except Exception as e:
        report = dict(error=type(e).__name__, reason=str(e), log_dir=str(ctx.directory))
        ctx.log('failure', **report)
        success = False
    tiers_done = 3 if (req.dry_run and success) else int(report.get('tiers_completed', 0))
    return success, Stack.Result(
        outcome=ctx.outcome(success),
        tiers_completed=tiers_done,
        stack_verified=bool(report.get('stack_verified')),
        dry_run_valid=bool(req.dry_run and success),
        evidence_uri=ctx.directory.name,
    )


def _goal_pin_and_go(ctx: 'GoalContext | SimpleNamespace', req):
    data = ctx.navigate(req.target_x_m, req.target_y_m, req.max_duration_s, dry_run=req.dry_run)
    result = PinAndGo.Result(**data)
    if ctx.commands and ctx.commands.latest_outcome is not None:
        result.outcome = ctx.commands.latest_outcome
    return bool(result.arrived or result.dry_run_valid), result


def _goal_garment(ctx: 'GoalContext | SimpleNamespace', req, action_cls, patrol=False):
    if patrol:
        data = ctx.patrol_collect(
            drop_x_m=req.drop_x_m, drop_y_m=req.drop_y_m, max_duration_s=req.max_duration_s,
            max_items=req.max_items, allow_towing=req.allow_towing, dry_run=req.dry_run,
        )
    else:
        data = ctx.deliver_garment(
            category=req.category, drop_x_m=req.drop_x_m, drop_y_m=req.drop_y_m,
            max_duration_s=req.max_duration_s, allow_towing=req.allow_towing, dry_run=req.dry_run,
        )
    success = (data.get('delivered', False) or data.get('completed', False) or data.get('dry_run_valid', False)
               or data.get('status_code') == 'PARTIAL_COMPLETION_SMALL_ITEMS_ONLY')
    result = action_cls.Result(**data)
    result.outcome = ctx.outcome(success)
    return success, result


BUILTIN_GOALS: dict[str, GoalSpec] = {
    spec.name: spec for spec in (
        GoalSpec(
            'pick', Pick, '/z70/pick', 285.,
            lambda s, r: r.color in s.thresholds,
            _goal_pick,
            fields=('color', 'dry_run'),
            description=GOAL_DESCRIPTIONS['pick'],
            validate_payload=lambda p: _is_color(p['color']) and type(p['dry_run']) is bool,
        ),
        GoalSpec(
            'stack', Stack, '/z70/stack', lambda r: 60. if r.dry_run else 540.,
            lambda s, r: len({r.base_color, r.middle_color, r.top_color}) == 3 and all(c in s.thresholds for c in (r.base_color, r.middle_color, r.top_color)),
            _goal_stack,
            fields=('base_color', 'middle_color', 'top_color', 'dry_run'),
            description=GOAL_DESCRIPTIONS['stack'],
            validate_payload=lambda p: (all(_is_color(p[k]) for k in ('base_color', 'middle_color', 'top_color'))
                                        and len({p['base_color'], p['middle_color'], p['top_color']}) == 3
                                        and type(p['dry_run']) is bool),
        ),
        GoalSpec(
            'pin_and_go', PinAndGo, '/z70/nav/pin_and_go', lambda r: r.max_duration_s,
            lambda s, r: all(math.isfinite(v) and -100 <= v <= 100 for v in (r.target_x_m, r.target_y_m)) and math.isfinite(r.max_duration_s) and 1 <= r.max_duration_s <= 600,
            _goal_pin_and_go,
            needs_executor=lambda _: False,
            navigation=True,
            fields=('target_x_m', 'target_y_m', 'max_duration_s', 'dry_run'),
            description=GOAL_DESCRIPTIONS['pin_and_go'],
            validate_payload=lambda p: (_num_in(p['target_x_m'], -100., 100.) and _num_in(p['target_y_m'], -100., 100.)
                                        and _num_in(p['max_duration_s'], 1., 600.) and type(p['dry_run']) is bool),
        ),
        GoalSpec(
            'deliver_garment', DeliverGarment, '/z70/deliver_garment', lambda r: r.max_duration_s,
            _valid_garment_goal,
            lambda ctx, r: _goal_garment(ctx, r, DeliverGarment, patrol=False),
            needs_executor=lambda r: not r.dry_run,
            navigation=True,
            fields=('category', 'drop_x_m', 'drop_y_m', 'max_duration_s', 'allow_towing', 'dry_run'),
            description=GOAL_DESCRIPTIONS['deliver_garment'],
            validate_payload=lambda p: (_num_in(p['drop_x_m'], -100., 100.) and _num_in(p['drop_y_m'], -100., 100.)
                                        and _num_in(p['max_duration_s'], 1., 3600.) and type(p['allow_towing']) is bool
                                        and type(p['dry_run']) is bool and isinstance(p['category'], str) and len(p['category']) <= 32),
        ),
        GoalSpec(
            'patrol_collect', PatrolCollect, '/z70/patrol_collect', lambda r: r.max_duration_s,
            _valid_garment_goal,
            lambda ctx, r: _goal_garment(ctx, r, PatrolCollect, patrol=True),
            needs_executor=lambda r: not r.dry_run,
            navigation=True,
            fields=('drop_x_m', 'drop_y_m', 'max_duration_s', 'max_items', 'allow_towing', 'dry_run'),
            description=GOAL_DESCRIPTIONS['patrol_collect'],
            validate_payload=lambda p: (_num_in(p['drop_x_m'], -100., 100.) and _num_in(p['drop_y_m'], -100., 100.)
                                        and _num_in(p['max_duration_s'], 1., 3600.) and type(p['allow_towing']) is bool
                                        and type(p['dry_run']) is bool and _int_in(p['max_items'], 1, 1000)),
        ),
        GoalSpec(
            'deploy_arm', DeployArm, '/z70/arm/deploy', 90.,
            lambda s, r: True,
            lambda ctx, r: _goal_curated(
                ctx, lambda c: c.deploy_arm(settle=r.settle),
                lambda outcome, ok, data: DeployArm.Result(outcome=outcome, estimate=ctx.arm_msg(), ready=bool(ok and data.get('ready'))),
            ),
            read_only=lambda _: False,
            fields=('settle',),
            description=GOAL_DESCRIPTIONS['deploy_arm'],
            validate_payload=lambda p: type(p['settle']) is bool,
        ),
        GoalSpec(
            'align_target', AlignToTarget, '/z70/align_target', 120.,
            lambda s, r: r.color in s.thresholds,
            lambda ctx, r: _goal_curated(
                ctx, lambda c: c.align_target(r.color),
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
            description=GOAL_DESCRIPTIONS['align_target'],
            validate_payload=lambda p: _is_color(p['color']),
        ),
        GoalSpec(
            'move_arm', MoveArm, '/z70/arm/move', 120.,
            lambda s, r: (all(math.isfinite(x) for x in (r.m3_deg, r.m4_deg, r.m5_deg))
                          and check_arm_limits(r.m3_deg, r.m4_deg, r.m5_deg) == 'SUCCESS'
                          and planner.safe((r.m3_deg, r.m4_deg, r.m5_deg, 90))),
            lambda ctx, r: _goal_curated(
                ctx, lambda c: c.move_arm(r.m3_deg, r.m4_deg, r.m5_deg),
                lambda outcome, ok, data: MoveArm.Result(
                    outcome=outcome,
                    estimate=ctx.arm_msg(),
                    steps_executed=int(data.get('steps_executed', 0)) if ok else 0,
                ),
            ),
            read_only=lambda _: False,
            fields=('m3_deg', 'm4_deg', 'm5_deg'),
            description=GOAL_DESCRIPTIONS['move_arm'],
            validate_payload=lambda p: _num_in(p['m3_deg'], 90, 270) and _num_in(p['m4_deg'], 35, 170) and _num_in(p['m5_deg'], 0, 155),
        ),
        GoalSpec(
            'grasp_and_lift', GraspAndLift, '/z70/arm/grasp_and_lift', 90.,
            lambda s, r: 0 <= r.lift_steps <= 10 and (not r.verify_color or r.verify_color in s.thresholds),
            lambda ctx, r: _goal_curated(
                ctx, lambda c: c.grasp_and_lift(lift_steps=r.lift_steps, verify_color=r.verify_color),
                lambda outcome, ok, data: GraspAndLift.Result(
                    outcome=outcome,
                    estimate=ctx.arm_msg(),
                    verified_hold=bool(ok and data.get('verified')),
                ),
            ),
            read_only=lambda _: False,
            fields=('lift_steps', 'verify_color'),
            description=GOAL_DESCRIPTIONS['grasp_and_lift'],
            validate_payload=lambda p: (_int_in(p['lift_steps'], 0, 10) and isinstance(p['verify_color'], str)
                                        and (p['verify_color'] == '' or _is_color(p['verify_color']))),
        ),
        GoalSpec(
            'release_and_stow', ReleaseAndStow, '/z70/arm/release_and_stow', 90.,
            lambda s, r: True,
            lambda ctx, r: _goal_curated(
                ctx, lambda c: c.release_and_stow(put_down=r.put_down),
                lambda outcome, ok, data: ReleaseAndStow.Result(
                    outcome=outcome,
                    estimate=ctx.arm_msg(),
                    stowed=bool(ok and data.get('stowed')),
                ),
            ),
            read_only=lambda _: False,
            fields=('put_down',),
            description=GOAL_DESCRIPTIONS['release_and_stow'],
            validate_payload=lambda p: type(p['put_down']) is bool,
        ),
    )
}


def _valid_pulse_payload(p: dict) -> bool:
    d = p.get('duration')
    return (isinstance(d, dict) and set(d) == {'sec', 'nanosec'}
            and _int_in(d['sec'], 0, 0) and _int_in(d['nanosec'], 1, 800000000)
            and _num_in(p['linear_mps'], -.1, .1) and _num_in(p['angular_radps'], -.5, .5)
            and not (p['linear_mps'] and p['angular_radps']))


DRIVER_GOALS: dict[str, GoalSpec] = {
    spec.name: spec for spec in (
        GoalSpec(
            'base_pulse', BasePulse, '/z70/base/pulse', 5.,
            lambda _s, _r: True,
            lambda ctx, r: (True, BasePulse.Result(outcome=ctx.outcome(True))),
            read_only=lambda _: False,
            fields=('linear_mps', 'angular_radps', 'duration'),
            description=GOAL_DESCRIPTIONS['base_pulse'],
            validate_payload=_valid_pulse_payload,
        ),
        GoalSpec(
            'arm_step', ArmStep, '/z70/arm/step', 5.,
            lambda _s, r: r.joint in (3, 4, 5, 34, 43) and r.direction in (-1, 1),
            lambda ctx, _r: (True, ArmStep.Result(outcome=ctx.outcome(True), estimate=ctx.arm_msg())),
            read_only=lambda _: False,
            fields=('joint', 'direction'),
            description=GOAL_DESCRIPTIONS['arm_step'],
            validate_payload=lambda p: (type(p['joint']) is int and p['joint'] in (3, 4, 5, 34, 43)
                                        and type(p['direction']) is int and p['direction'] in (-1, 1)),
        ),
        GoalSpec(
            'hand_step', HandStep, '/z70/arm/hand_step', 5.,
            lambda _s, r: 1 <= r.direction <= 4,
            lambda ctx, _r: (True, HandStep.Result(outcome=ctx.outcome(True), estimate=ctx.arm_msg())),
            read_only=lambda _: False,
            fields=('direction',),
            description=GOAL_DESCRIPTIONS['hand_step'],
            validate_payload=lambda p: _int_in(p['direction'], 1, 4),
        ),
        GoalSpec(
            'arm_operation', ArmOperation, '/z70/arm/operation', 15.,
            lambda _s, r: 1 <= r.operation <= 9,
            lambda ctx, _r: (True, ArmOperation.Result(outcome=ctx.outcome(True), estimate=ctx.arm_msg())),
            read_only=lambda _: False,
            fields=('operation',),
            description=GOAL_DESCRIPTIONS['arm_operation'],
            validate_payload=lambda p: _int_in(p['operation'], 1, 9),
        ),
    )
}

ALL_GOALS: dict[str, GoalSpec] = {**BUILTIN_GOALS, **DRIVER_GOALS}
