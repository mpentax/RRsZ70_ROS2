"""Composable ROS 2 Goal execution context and declarative GoalSpec registry.

Provides LLM-authored and curated ROS 2 goals with a single typed context (`GoalContext`)
for reading robot state, arm kinematics, map/spatial context, and object/garment perception,
and for invoking navigation, base pulses, and multi-axis arm primitives.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable
import time

from tf2_ros import TransformException
from z70_interfaces.msg import CommandOutcome
from z70_mapping.boundary import compute_safe_interior
from .executor import Executor
from .navigation import PinAndGoNavigator

GOAL_CONTEXT_PRIMITIVES: tuple[dict[str, str], ...] = (
    {'name': 'status', 'signature': 'status() -> dict', 'summary': 'Read the latest robot status telemetry dict.'},
    {'name': 'arm', 'signature': 'arm() -> dict', 'summary': 'Read the current arm joint and pose estimate dict.'},
    {'name': 'map_pose', 'signature': "map_pose(source='base_footprint') -> tuple[float, float, float]", 'summary': 'Lookup (x_m, y_m, yaw_rad) of a frame in the map.'},
    {'name': 'safe_interior', 'signature': 'safe_interior(refresh=False, deployed_arm_extension_m=0.)', 'summary': 'Return (map_snapshot, safe_interior_grid) for navigation and corridor planning.'},
    {'name': 'objects', 'signature': "objects(camera='front', color=None) -> list[dict]", 'summary': 'Detect and project HSV objects in base_footprint from a fresh camera frame.'},
    {'name': 'garments', 'signature': "garments(category='any', after=None) -> list", 'summary': 'Return verified reachable garments from the garment observation pipeline.'},
    {'name': 'navigate', 'signature': 'navigate(target_x_m: float, target_y_m: float, max_duration_s: float = 60., dry_run: bool = False) -> dict', 'summary': 'Navigate the stowed chassis to (target_x_m, target_y_m) via PinAndGo.'},
    {'name': 'pick', 'signature': 'pick(color: str, dry_run: bool = False) -> dict', 'summary': 'Execute a full autonomous object pickup by HSV color.'},
    {'name': 'stack', 'signature': 'stack(base_color: str, middle_color: str, top_color: str, dry_run: bool = False) -> dict', 'summary': 'Execute a 3-object stack by HSV color.'},
    {'name': 'deliver_garment', 'signature': "deliver_garment(category: str = 'any', drop_x_m: float = 0., drop_y_m: float = 0., max_duration_s: float = 300., allow_towing: bool = False, dry_run: bool = False) -> dict", 'summary': 'Detect, grasp, and deliver a single floor garment to the drop zone.'},
    {'name': 'patrol_collect', 'signature': 'patrol_collect(drop_x_m: float = 0., drop_y_m: float = 0., max_duration_s: float = 600., max_items: int = 5, allow_towing: bool = False, dry_run: bool = False) -> dict', 'summary': 'Patrol safe interior waypoints to collect and deliver garments to the drop zone.'},
    {'name': 'deploy_arm', 'signature': 'deploy_arm(settle=True) -> dict', 'summary': 'Enter manual arm mode and unstow the arm to WORKING_POS.'},
    {'name': 'align_target', 'signature': 'align_target(color: str) -> dict', 'summary': 'Visually servo the chassis toward a target color using front camera observations.'},
    {'name': 'move_arm', 'signature': 'move_arm(m3_deg: float, m4_deg: float, m5_deg: float = 0.) -> dict', 'summary': 'Execute a collision-checked coordinated multi-axis arm trajectory.'},
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
        objects=lambda camera='front', color=None: [],
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
        move_arm=lambda m3_deg, m4_deg, m5_deg=0.: {'steps_executed': 1},
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

    def move_arm(self, m3_deg: float, m4_deg: float, m5_deg: float = 0.) -> dict:
        return self.executor.move_arm((m3_deg, m4_deg, m5_deg))

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
