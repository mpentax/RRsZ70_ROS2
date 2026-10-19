"""Composable ROS 2 Goal execution context and declarative GoalSpec registry.

Provides LLM-authored and curated ROS 2 goals with a single typed context (`GoalContext`)
for reading robot state, arm kinematics, map/spatial context, and object/garment perception,
and for invoking navigation, base pulses, and multi-axis arm primitives.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable
import time

from tf2_ros import TransformException
from z70_interfaces.msg import CommandOutcome
from z70_mapping.boundary import compute_safe_interior
from .executor import Executor
from .navigation import PinAndGoNavigator


@dataclass
class GoalSpec:
    """Declarative specification for a ROS 2 Action goal server."""
    name: str
    action_cls: type
    topic: str
    timeout_s: float | Callable[[Any], float]
    validate: Callable[[Any, Any], bool]
    run: Callable[['GoalContext', Any], tuple[bool, Any]]
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
            'description': self.description,
        }


def load_custom_goals(server, directory: Path) -> list[str]:
    """Load any custom ROS 2 goal plugins (`*.py`) from `directory` onto `server`."""
    loaded = []
    if not directory or not directory.is_dir():
        return loaded
    for path in sorted(directory.glob('*.py')):
        if path.name.startswith('_'):
            continue
        spec = importlib.util.spec_from_file_location(f'z70_custom_goal_{path.stem}', path)
        if spec is None or spec.loader is None:
            continue
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        if hasattr(mod, 'register') and callable(mod.register):
            mod.register(server)
            loaded.append(path.stem)
        elif hasattr(mod, 'GOAL_SPECS'):
            for goal_spec in mod.GOAL_SPECS:
                server.register_goal(goal_spec)
            loaded.append(path.stem)
    return loaded


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
        if hasattr(self.observations, 'status'):
            return self.observations.status()
        return self.observations.get('/api/state')['telemetry']['status']['value']

    def arm(self) -> dict:
        if hasattr(self.observations, 'arm_estimate'):
            return self.observations.arm_estimate()
        return self.observations.get('/api/arm/estimate')

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
