"""Composable ROS 2 Goal execution context and declarative GoalSpec registry.

Provides LLM-authored and curated ROS 2 goals with a single typed context (`GoalContext`)
for reading robot state, arm kinematics, map/spatial context, and object/garment perception,
and for invoking navigation, base pulses, and multi-axis arm primitives.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
from types import SimpleNamespace
from typing import Any, Callable
import time

from tf2_ros import TransformException
from z70_interfaces.action import (
    DeliverGarment, PatrolCollect, PinAndGo, Pick, Stack,
    DeployArm, AlignToTarget, MoveArm, GraspAndLift, ReleaseAndStow,
    BasePulse, ArmStep, HandStep, ArmOperation,
)
from z70_description.arm_model import HAND_STEP_M, vendor_check, vendor_forward, vendor_inverse
from z70_driver.commands import ARM_OPERATION_RPCS, arm_step_vector
from z70_driver.driver import app_to_urdf
from z70_interfaces.msg import ArmEstimate, CommandOutcome
from z70_mapping.boundary import SessionWorldMap, check_corridor_containment, compute_safe_interior, mobility_from_suitability
from z70_mapping.node import spatial_context as compute_spatial_context
from . import perception, planner
from .executor import Executor, read_arm, read_status
from .garment_delivery import GarmentDeliverer, PatrolCollector, PatrolLedger, PinAndGoNavigator, dynamic_obstacles, graspable_garments, loaded_route, track_key
from .perception import transform_xy

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
    {'name': 'spatial_context', 'signature': 'spatial_context(refresh: bool = False) -> dict', 'summary': 'Return ego-centric map context: current_room, rooms (centroids/distances/bearings), dock pose, 8-sector clearance_sectors_m, and world_objects.'},
    {'name': 'room_waypoint', 'signature': 'room_waypoint(room: str | int, refresh: bool = False) -> dict | None', 'summary': 'Return the highest-clearance reachable safe-interior waypoint {segment_id, name, x, y, clearance_m} for a room name or segment ID.'},
    {'name': 'plan_route', 'signature': 'plan_route(target_x_m: float, target_y_m: float, deployed_arm_extension_m: float = 0., refresh: bool = False) -> list[tuple[float, float]] | None', 'summary': 'Plan a collision-checked A* route on the eroded safe interior from the current robot pose to (target_x_m, target_y_m).'},
    {'name': 'corridor_safe', 'signature': 'corridor_safe(start_x_m: float, start_y_m: float, end_x_m: float, end_y_m: float, radius_m: float = 0., deployed_arm_extension_m: float = 0., refresh: bool = False) -> bool', 'summary': 'Check whether a straight-line swept disk from (start_x_m, start_y_m) to (end_x_m, end_y_m) stays inside the reachable safe interior.'},
    {'name': 'select_camera', 'signature': "select_camera(camera: str) -> dict", 'summary': "Switch the active camera lens ('front' or 'gripper') via the ROS driver selector."},
    {'name': 'objects', 'signature': "objects(camera='front', color=None) -> list[dict]", 'summary': 'Detect and project HSV objects in base_footprint from a fresh camera frame.'},
    {'name': 'wrist_pose', 'signature': 'wrist_pose() -> dict | None', 'summary': 'Estimate the elevated ArUco ID68 wrist/jaw pose and grasp reference in base_footprint from the front camera.'},
    {'name': 'fit_wrist_joints', 'signature': 'fit_wrist_joints() -> dict | None', 'summary': 'Fit [M3, M4, M5] joint angles (degrees) to the front-camera ArUco ID68 wrist marker pose.'},
    {'name': 'garments', 'signature': "garments(category='any', after=None) -> list", 'summary': 'Return verified reachable garments from the garment observation pipeline.'},
    {'name': 'world_objects', 'signature': 'world_objects() -> dict', 'summary': 'Return the session-scoped dynamic world map summary (objects classified by mobility: pickupable/draggable/stationary and relevance: objective/completed/deferred/ignored).'},
    {'name': 'update_world_object', 'signature': "update_world_object(x_m: float, y_m: float, category: str = 'object', mobility: str = 'pickupable', relevance: str = 'objective', radius_m: float = 0.12, detail: str = '') -> dict", 'summary': 'Register or update a session-scoped world object in the map frame with explicit mobility (pickupable/draggable/stationary) and objective relevance (objective/completed/deferred/ignored).'},
    {'name': 'mark_obstacle', 'signature': "mark_obstacle(x_m: float, y_m: float, radius_m: float = 0.12, category: str = 'obstacle', detail: str = 'OBSTACLE') -> dict", 'summary': 'Record a session-scoped stationary or deferred obstacle at (x_m, y_m) in the map frame so safe_interior and route planners avoid it.'},
    {'name': 'clear_world', 'signature': 'clear_world() -> dict', 'summary': 'Clear all session-scoped world objects and dynamic obstacles from the active session world map.'},
    {'name': 'navigate', 'signature': 'navigate(target_x_m: float, target_y_m: float, max_duration_s: float = 60., dry_run: bool = False) -> dict', 'summary': 'Navigate the stowed chassis to (target_x_m, target_y_m) via PinAndGo.'},
    {'name': 'navigate_room', 'signature': 'navigate_room(room: str | int, max_duration_s: float = 60., dry_run: bool = False) -> dict', 'summary': 'Resolve a room name or segment ID to its safest interior waypoint and navigate there via PinAndGo.'},
    {'name': 'pick', 'signature': 'pick(color: str, dry_run: bool = False) -> dict', 'summary': 'Execute a full autonomous object pickup by HSV color.'},
    {'name': 'stack', 'signature': 'stack(base_color: str, middle_color: str, top_color: str, dry_run: bool = False) -> dict', 'summary': 'Execute a 3-object stack by HSV color.'},
    {'name': 'deliver_garment', 'signature': "deliver_garment(category: str = 'any', drop_x_m: float = 0., drop_y_m: float = 0., max_duration_s: float = 300., allow_towing: bool = False, dry_run: bool = False) -> dict", 'summary': 'Detect, grasp, and deliver a single floor garment to the drop zone.'},
    {'name': 'patrol_collect', 'signature': 'patrol_collect(drop_x_m: float = 0., drop_y_m: float = 0., max_duration_s: float = 600., max_items: int = 5, allow_towing: bool = False, dry_run: bool = False) -> dict', 'summary': 'Patrol safe interior waypoints to collect and deliver garments to the drop zone.'},
    {'name': 'deploy_arm', 'signature': 'deploy_arm(settle=True) -> dict', 'summary': 'Enter manual arm mode and unstow the arm to WORKING_POS.'},
    {'name': 'align_target', 'signature': 'align_target(color: str) -> dict', 'summary': 'Visually servo the chassis toward a target color using front camera observations.'},
    {'name': 'arm_safe', 'signature': 'arm_safe(m3_deg: float, m4_deg: float, m5_deg: float = 0., m6_deg: float = 90.) -> bool', 'summary': 'Check whether an arm joint state (m3, m4, m5, m6) satisfies firmware limits and CAD floor/body collision bounds.'},
    {'name': 'arm_kinematics', 'signature': 'arm_kinematics(m3_deg: float, m4_deg: float, m5_deg: float = 0., m6_deg: float = 90.) -> dict', 'summary': 'Return forward kinematics (operational_point_m, grasp_point_m, tip_height_m, tip_floor_bound_m, vendor_endpoint_xz_m) and collision safety for a joint state.'},
    {'name': 'plan_approach', 'signature': 'plan_approach(x_m: float, y_m: float) -> dict | None', 'summary': 'Compute the chassis yaw (rad) and forward distance (m) needed to align a base_footprint floor point (x_m, y_m) to the arm grasp point.'},
    {'name': 'plan_grasp', 'signature': 'plan_grasp(x_m: float, y_m: float, size_m: float = 0.066) -> dict | None', 'summary': 'Compute the closest reachable (M3, M4) grasp state and coordinated descent steps for a floor point (x_m, y_m) in base_footprint.'},
    {'name': 'plan_stack_tier', 'signature': 'plan_stack_tier(tier: int, size_m: float = 0.066) -> dict | None', 'summary': 'Plan the placement arm state, required clearance height, lift steps, and trajectory steps for tier 1..3 of a vertical stack.'},
    {'name': 'plan_trajectory', 'signature': 'plan_trajectory(m3_deg: float, m4_deg: float, m5_deg: float = 0.) -> list[dict] | None', 'summary': 'Plan a collision-checked coordinated 3-axis 10-degree joint trajectory from the current arm joints to (m3_deg, m4_deg, m5_deg).'},
    {'name': 'move_arm', 'signature': 'move_arm(m3_deg: float, m4_deg: float, m5_deg: float = 0.) -> dict', 'summary': 'Execute a collision-checked coordinated multi-axis arm trajectory.'},
    {'name': 'arm_step', 'signature': 'arm_step(m3: int = 0, m4: int = 0, m5: int = 0) -> dict', 'summary': 'Execute a single 10-degree multi-axis joint step [m3, m4, m5] (0=hold, 1=+10deg, 2=-10deg).'},
    {'name': 'hand_step', 'signature': 'hand_step(direction: int) -> dict', 'summary': 'Execute a single 40 mm Cartesian IK hand step (1=up, 2=down, 3=forward, 4=back).'},
    {'name': 'open_jaws', 'signature': 'open_jaws()', 'summary': 'Open gripper jaws in place without lifting or stowing the arm.'},
    {'name': 'set_arm_anchor', 'signature': "set_arm_anchor(anchor: str, reason: str = '') -> dict", 'summary': "Re-anchor the driver arm estimator ('stowed', 'deployed', or 'unknown') after visual confirmation."},
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

    def __post_init__(self):
        if not self.description:
            self.description = GOAL_DESCRIPTIONS.get(self.name, '')

    def describe(self) -> dict:
        goal_cls = getattr(self.action_cls, 'Goal', None)
        result_cls = getattr(self.action_cls, 'Result', None)
        raw_types = dict(goal_cls.get_fields_and_field_types()) if hasattr(goal_cls, 'get_fields_and_field_types') else {}
        field_types = {k: raw_types[k] for k in self.fields if k in raw_types} if self.fields else raw_types
        result_fields = list(result_cls.get_fields_and_field_types().keys()) if hasattr(result_cls, 'get_fields_and_field_types') else []
        default_req = SimpleNamespace(
            dry_run=False,
            max_duration_s=120. if self.name == 'pin_and_go' else 300. if self.name == 'deliver_garment' else 600.,
        )
        timeout_s = float(self.timeout_s(default_req) if callable(self.timeout_s) else self.timeout_s)
        return {
            'kind': self.name,
            'name': self.name,
            'topic': self.topic,
            'action': self.action_cls.__name__,
            'fields': list(self.fields),
            'field_types': field_types,
            'result_fields': result_fields,
            'timeout_s': timeout_s,
            'navigation': self.navigation,
            'description': self.description,
        }


def simulate_goal(spec: GoalSpec, request, **overrides) -> tuple[bool, Any]:
    """Run `spec.run(ctx, request)` against a stateful in-memory mock `GoalContext`."""
    trace = overrides.pop('_trace', None)
    if not isinstance(trace, list):
        trace = []
    arm_override = isinstance(overrides.get('arm'), dict)
    sim_arm = deepcopy(overrides.pop('arm')) if arm_override else {
        'state': 'WORKING_POS', 'joints': {'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.},
    }
    arm_unrestored = bool(arm_override and sim_arm.get('state') != 'IN_ROBOT')
    sim_pose = [0., 0., 0.]
    sim_world = SessionWorldMap()
    sc_base: dict[str, Any] = {'current_room': None, 'rooms': [], 'dock': None, 'clearance_sectors_m': {}}
    if isinstance(overrides.get('spatial_context'), dict):
        sc_init = deepcopy(overrides.pop('spatial_context'))
        if isinstance(sc_init.get('world_objects'), dict):
            sim_world.load_entries(sc_init.pop('world_objects'))
        rp = sc_init.get('robot_pose_map')
        if isinstance(rp, dict) and all(rp.get(k) is not None for k in ('x', 'y')):
            sim_pose = [float(rp['x']), float(rp['y']), float(rp.get('yaw') or 0.)]
        sc_base.update(sc_init)
    if isinstance(overrides.get('world_objects'), dict):
        sim_world.load_entries(overrides.pop('world_objects'))
    if isinstance(overrides.get('odom'), dict) and all(k in overrides['odom'] for k in ('x', 'y')):
        od = overrides.pop('odom')
        sim_pose = [float(od['x']), float(od['y']), float(od.get('yaw') or 0.)]
    if isinstance(overrides.get('map_pose'), (list, tuple)) and len(overrides['map_pose']) >= 3:
        mp = overrides.pop('map_pose')
        sim_pose = [float(mp[0]), float(mp[1]), float(mp[2])]

    def _rec(primitive: str, **info):
        trace.append({'primitive': primitive, **info})

    def _deploy_arm(settle=True):
        nonlocal arm_unrestored
        arm_unrestored = True
        sim_arm.update(state='WORKING_POS', joints={'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        _rec('deploy_arm', settle=bool(settle))
        return {'ready': True}

    def _move_arm(m3_deg, m4_deg, m5_deg=0.):
        nonlocal arm_unrestored
        arm_unrestored = True
        j = sim_arm.get('joints') or {}
        cur = [150., 45., 0.] if sim_arm.get('state') == 'IN_ROBOT' else [
            float(j.get('m3', 150.)), float(j.get('m4', 45.)), float(j.get('m5', 0.)),
        ]
        target = [round(float(m3_deg)), round(float(m4_deg)), round(float(m5_deg))]
        steps = planner.trajectory(cur, target, m6=float(j.get('m6', 90.)))
        sim_arm['state'] = 'IN_PROCESS_POS'
        sim_arm['joints'] = {'m3': float(target[0]), 'm4': float(target[1]), 'm5': float(target[2]), 'm6': float(j.get('m6', 90.))}
        _rec('move_arm', target=target, steps_executed=len(steps))
        return {'state': target, 'steps_executed': len(steps), 'steps': steps}

    def _arm_step(m3=0, m4=0, m5=0):
        nonlocal arm_unrestored
        if any(v not in (0, 1, 2) for v in (m3, m4, m5)) or not any((m3, m4, m5)):
            raise ValueError('arm_step axes must be in (0, 1, 2) with at least one nonzero axis')
        arm_unrestored = True
        if sim_arm.get('state') == 'IN_ROBOT':
            sim_arm.update(state='WORKING_POS', joints={'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        j = sim_arm.setdefault('joints', {'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        cur = (float(j.get('m3', 150.)), float(j.get('m4', 45.)), float(j.get('m5', 0.)), float(j.get('m6', 90.)))
        nxt = [
            cur[0] + (10. if m3 == 1 else -10. if m3 == 2 else 0.),
            cur[1] + (10. if m4 == 1 else -10. if m4 == 2 else 0.),
            cur[2] + (10. if m5 == 1 else -10. if m5 == 2 else 0.),
            cur[3],
        ]
        if not planner.safe(nxt) or not planner.safe_edge(cur, nxt):
            raise ValueError(f'unsafe arm_step transition {cur[:3]} -> {nxt[:3]}')
        j['m3'], j['m4'], j['m5'] = nxt[0], nxt[1], nxt[2]
        sim_arm['state'] = 'IN_PROCESS_POS'
        _rec('arm_step', m3=int(m3), m4=int(m4), m5=int(m5))
        return {'m3': int(m3), 'm4': int(m4), 'm5': int(m5)}

    def _hand_step(direction):
        nonlocal arm_unrestored
        if direction not in (1, 2, 3, 4):
            raise ValueError('hand_step direction must be in (1, 2, 3, 4)')
        arm_unrestored = True
        if sim_arm.get('state') == 'IN_ROBOT':
            sim_arm.update(state='WORKING_POS', joints={'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        j = sim_arm.setdefault('joints', {'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        delta = {1: (0, 1), 2: (0, -1), 3: (1, 0), 4: (-1, 0)}[int(direction)]
        m3, m4, m5 = float(j.get('m3', 150.)), float(j.get('m4', 45.)), float(j.get('m5', 0.))
        x, y = vendor_forward(m3, m4)[1]
        sol = vendor_inverse(x + delta[0] * HAND_STEP_M, y + delta[1] * HAND_STEP_M, (m3, m4))
        if sol is None or vendor_check(sol[0], sol[1], m5) != 'SUCCESS' or not planner.safe((sol[0], sol[1], m5, float(j.get('m6', 90.)))):
            raise ValueError(f'unreachable hand_step direction {direction}')
        j['m3'], j['m4'] = float(sol[0]), float(sol[1])
        sim_arm['state'] = 'IN_PROCESS_POS'
        _rec('hand_step', direction=int(direction))
        return {'direction': int(direction)}

    def _open_jaws():
        sim_arm.setdefault('joints', {})['m6'] = 90.
        _rec('open_jaws')

    def _grasp_and_lift(lift_steps=2, verify_color=''):
        nonlocal arm_unrestored
        arm_unrestored = True
        if sim_arm.get('state') == 'IN_ROBOT':
            sim_arm.update(state='WORKING_POS', joints={'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        j = sim_arm.setdefault('joints', {'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        j['m6'] = 0.
        m3, m4, m5 = float(j.get('m3', 150.)), float(j.get('m4', 45.)), float(j.get('m5', 0.))
        for _ in range(max(0, int(lift_steps))):
            x, y = vendor_forward(m3, m4)[1]
            sol = vendor_inverse(x, y + HAND_STEP_M, (m3, m4))
            if sol is not None and vendor_check(sol[0], sol[1], m5) == 'SUCCESS':
                m3, m4 = float(sol[0]), float(sol[1])
        j['m3'], j['m4'] = m3, m4
        sim_arm['state'] = 'IN_PROCESS_POS'
        verified = bool(defaults['objects']('front', verify_color)) if (verify_color and 'objects' in overrides) else True
        _rec('grasp_and_lift', lift_steps=int(lift_steps), verify_color=str(verify_color))
        return {'gripped': True, 'lift_steps': int(lift_steps), 'verified': verified}

    def _release_and_stow(put_down=True):
        nonlocal arm_unrestored
        arm_unrestored = False
        sim_arm.update(state='IN_ROBOT', joints={'m3': 0., 'm4': 0., 'm5': 0., 'm6': 0.})
        _rec('release_and_stow', put_down=bool(put_down))
        return {'stowed': True}

    def _set_arm_anchor(anchor, reason=''):
        nonlocal arm_unrestored
        accepted = anchor in ('unknown', 'stowed', 'deployed')
        if anchor == 'stowed':
            arm_unrestored = False
            sim_arm.update(state='IN_ROBOT', joints={'m3': 0., 'm4': 0., 'm5': 0., 'm6': 0.})
        elif anchor == 'deployed':
            arm_unrestored = True
            sim_arm.update(state='WORKING_POS', joints={'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        elif anchor == 'unknown':
            arm_unrestored = True
            sim_arm['state'] = 'UNKNOWN'
        _rec('set_arm_anchor', anchor=str(anchor), reason=str(reason))
        return {'accepted': accepted, 'anchor': anchor}

    def _arm_msg():
        st = sim_arm.get('state', 'IN_ROBOT')
        m = ArmEstimate()
        m.evidence.valid = True
        if st == 'IN_ROBOT':
            m.validity.value = 3; m.validity.reason = 'COMMAND_DERIVED_STOW'
        elif st != 'UNKNOWN':
            m.validity.value = 1; m.validity.reason = 'COMMAND_DEAD_RECKONING'
            j = sim_arm.get('joints') or {}
            q = app_to_urdf(j.get('m3', 150.), j.get('m4', 45.), j.get('m5', 0.), j.get('m6', 90.))
            m.joints.name = list(q)[:3]
            m.joints.position = list(q.values())[:3]
        else:
            m.validity.value = 0; m.validity.reason = 'UNRECONCILED_OR_UNKNOWN'
        return m

    def _pulse(velocity=0., omega=0., duration=300):
        dt = float(duration) / 1000.
        sim_pose[2] = math.atan2(math.sin(sim_pose[2] + float(omega) * dt), math.cos(sim_pose[2] + float(omega) * dt))
        sim_pose[0] += float(velocity) * dt * math.cos(sim_pose[2])
        sim_pose[1] += float(velocity) * dt * math.sin(sim_pose[2])
        _rec('pulse', velocity=float(velocity), omega=float(omega), duration=int(duration))
        return {'ok': True}

    def _rpc(method, params=None, timeout_ms=5000):
        nonlocal arm_unrestored
        m = str(method)
        if m == 'app_arm_out_compartment':
            arm_unrestored = True
            sim_arm.update(state='WORKING_POS', joints={'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.})
        elif m in ('app_arm_in_compartment', 'app_arm_recover'):
            arm_unrestored = False
            sim_arm.update(state='IN_ROBOT', joints={'m3': 0., 'm4': 0., 'm5': 0., 'm6': 0.})
        elif m in ('app_open_gripper', 'app_put_down'):
            sim_arm.setdefault('joints', {})['m6'] = 90.
        elif m == 'app_grip':
            sim_arm.setdefault('joints', {})['m6'] = 0.
        _rec('rpc', method=m, params=params)
        return {'result': ['ok']}

    def _seg_dist(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        denom = dx * dx + dy * dy
        t = max(0., min(1., ((px - ax) * dx + (py - ay) * dy) / denom)) if denom > 1e-12 else 0.
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    def _corridor_safe(start_x_m, start_y_m, end_x_m, end_y_m, radius_m=0., deployed_arm_extension_m=0., refresh=False):
        pad = float(radius_m) + float(deployed_arm_extension_m)
        for e in sim_world.entries.values():
            if e.relevance in ('deferred', 'ignored') or e.mobility in ('stationary', 'draggable'):
                if _seg_dist(e.x, e.y, float(start_x_m), float(start_y_m), float(end_x_m), float(end_y_m)) <= e.radius_m + pad:
                    return False
        return True

    def _plan_route(target_x_m, target_y_m, deployed_arm_extension_m=0., refresh=False):
        tx, ty = float(target_x_m), float(target_y_m)
        if not _corridor_safe(tx, ty, tx, ty, deployed_arm_extension_m=deployed_arm_extension_m):
            return None
        if _corridor_safe(sim_pose[0], sim_pose[1], tx, ty, deployed_arm_extension_m=deployed_arm_extension_m):
            return [(tx, ty)]
        dx, dy = tx - sim_pose[0], ty - sim_pose[1]
        norm = math.hypot(dx, dy) or 1.
        nx, ny = -dy / norm, dx / norm
        for e in sim_world.entries.values():
            if e.relevance in ('deferred', 'ignored') or e.mobility in ('stationary', 'draggable'):
                off = e.radius_m + float(deployed_arm_extension_m) + 0.15
                for sign in (1., -1.):
                    wx, wy = round(e.x + sign * nx * off, 3), round(e.y + sign * ny * off, 3)
                    if (_corridor_safe(sim_pose[0], sim_pose[1], wx, wy, deployed_arm_extension_m=deployed_arm_extension_m)
                            and _corridor_safe(wx, wy, tx, ty, deployed_arm_extension_m=deployed_arm_extension_m)):
                        return [(wx, wy), (tx, ty)]
        return None

    def _navigate(target_x_m, target_y_m, max_duration_s=60., dry_run=False):
        _rec('navigate', target_x_m=float(target_x_m), target_y_m=float(target_y_m), dry_run=bool(dry_run))
        if arm_unrestored and sim_arm.get('state', 'IN_ROBOT') != 'IN_ROBOT':
            return {
                'arrived': False, 'dry_run_valid': False, 'status_code': 'ARM_NOT_STOWED',
                'final_position_m': [sim_pose[0], sim_pose[1]],
                'position_error_m': math.hypot(float(target_x_m) - sim_pose[0], float(target_y_m) - sim_pose[1]),
                'detail': 'ARM_NOT_STOWED',
            }
        if not defaults['corridor_safe'](target_x_m, target_y_m, target_x_m, target_y_m):
            return {
                'arrived': False, 'dry_run_valid': False, 'status_code': 'TARGET_OUTSIDE_INTERIOR',
                'final_position_m': [sim_pose[0], sim_pose[1]],
                'position_error_m': math.hypot(float(target_x_m) - sim_pose[0], float(target_y_m) - sim_pose[1]),
                'detail': 'TARGET_OUTSIDE_INTERIOR',
            }
        if not dry_run:
            sim_pose[0], sim_pose[1] = float(target_x_m), float(target_y_m)
        return {
            'arrived': not dry_run, 'dry_run_valid': bool(dry_run),
            'status_code': 'DRY_RUN_VALID' if dry_run else 'ARRIVED',
            'final_position_m': [sim_pose[0], sim_pose[1]], 'position_error_m': 0., 'detail': '',
        }

    def _room_waypoint(room, refresh=False):
        sc = defaults['spatial_context'](refresh=refresh) if callable(defaults.get('spatial_context')) else {}
        rooms = (sc or {}).get('rooms') or []
        if rooms:
            q = str(room).strip().lower()
            seg_str = q.removeprefix('segment ').strip()
            for r in rooms:
                if (isinstance(room, int) and not isinstance(room, bool) and r.get('segment_id') == room) or str(r.get('segment_id')) == seg_str or str(r.get('name', '')).strip().lower() == q:
                    if r.get('reachable') is False:
                        return None
                    c = r.get('safe_waypoint') or r.get('centroid') or [1.0, 1.0]
                    return {'segment_id': int(r['segment_id']), 'name': str(r.get('name', room)), 'x': float(c[0]), 'y': float(c[1]), 'clearance_m': 0.8}
            return None
        return {
            'segment_id': int(room) if isinstance(room, int) or str(room).isdigit() else 1,
            'name': str(room), 'x': 1.0, 'y': 1.0, 'clearance_m': 0.8,
        }

    def _navigate_room(room, max_duration_s=60., dry_run=False):
        wp = defaults['room_waypoint'](room, refresh=True)
        _rec('navigate_room', room=room, max_duration_s=float(max_duration_s), dry_run=bool(dry_run))
        if wp is None:
            return {'arrived': False, 'dry_run_valid': False, 'status_code': 'ROOM_UNREACHABLE', 'room': room, 'detail': 'ROOM_UNREACHABLE'}
        res = defaults['navigate'](wp['x'], wp['y'], max_duration_s=max_duration_s, dry_run=dry_run)
        return {**res, 'room': wp['name'], 'segment_id': wp['segment_id'], 'target_x_m': float(wp['x']), 'target_y_m': float(wp['y'])}

    def _update_world_object(x_m, y_m, category='object', mobility='pickupable', relevance='objective', radius_m=0.12, detail=''):
        entry = sim_world.upsert_object(
            float(x_m), float(y_m), category=str(category), mobility=str(mobility),
            relevance=str(relevance), radius_m=float(radius_m), detail=str(detail),
        )
        _rec('update_world_object', x_m=float(x_m), y_m=float(y_m), category=str(category), mobility=str(mobility), relevance=str(relevance))
        return entry.to_dict() if entry else {}

    def _mark_obstacle(x_m, y_m, radius_m=0.12, category='obstacle', detail='OBSTACLE'):
        entry = sim_world.mark_obstacle(float(x_m), float(y_m), radius_m=float(radius_m), category=str(category), detail=str(detail))
        _rec('mark_obstacle', x_m=float(x_m), y_m=float(y_m), radius_m=float(radius_m), category=str(category))
        return entry.to_dict()

    def _clear_world():
        sim_world.clear()
        _rec('clear_world')
        return {'cleared': True}

    def _pick(color, dry_run=False):
        nonlocal arm_unrestored
        if 'objects' in overrides:
            seen = defaults['objects']('front', color)
            target = seen[0] if seen else None
        else:
            target = {'color': str(color), 'position': [0.35, 0., 0.03]}
        ok = target is not None
        if ok and not dry_run:
            arm_unrestored = False
            sim_arm.update(state='IN_ROBOT', joints={'m3': 0., 'm4': 0., 'm5': 0., 'm6': 0.})
            raw_pos = target.get('position') if isinstance(target, dict) else None
            pos = raw_pos if isinstance(raw_pos, (list, tuple)) and len(raw_pos) >= 2 else [0.35, 0., 0.03]
            px, py = transform_xy((float(pos[0]), float(pos[1])), (sim_pose[0], sim_pose[1], sim_pose[2]))
            sim_world.mark_completed(px, py, category=str(color), detail='PICKED')
        _rec('pick', color=str(color), dry_run=bool(dry_run))
        return {'verified': ok and not dry_run, 'dry_run': bool(dry_run), 'target': target}

    def _stack(base_color, middle_color, top_color, dry_run=False):
        nonlocal arm_unrestored
        if 'objects' in overrides:
            all_vis = all(bool(defaults['objects']('front', c)) for c in (base_color, middle_color, top_color))
        else:
            all_vis = True
        if all_vis and not dry_run:
            arm_unrestored = False
            sim_arm.update(state='IN_ROBOT', joints={'m3': 0., 'm4': 0., 'm5': 0., 'm6': 0.})
            px, py = transform_xy((0.35, 0.), (sim_pose[0], sim_pose[1], sim_pose[2]))
            sim_world.mark_completed(px, py, category=f'stack:{base_color}+{middle_color}+{top_color}', detail='STACKED')
        _rec('stack', base_color=str(base_color), middle_color=str(middle_color), top_color=str(top_color), dry_run=bool(dry_run))
        return {'stack_verified': all_vis and not dry_run, 'dry_run': bool(dry_run), 'all_targets_visible': all_vis, 'tiers_completed': 3 if all_vis else 0}

    def _align_target(color):
        if 'objects' in overrides:
            seen = defaults['objects']('front', color)
            if not seen:
                raise LookupError(f'target {color} not visible')
            pos = list(seen[0].get('position', [0.43, 0., 0.04]))
        else:
            pos = [0.43, 0., 0.04]
        app = planner.approach(pos)
        sim_pose[2] = math.atan2(math.sin(sim_pose[2] + float(app['yaw'])), math.cos(sim_pose[2] + float(app['yaw'])))
        sim_pose[0] += float(app['forward']) * math.cos(sim_pose[2])
        sim_pose[1] += float(app['forward']) * math.sin(sim_pose[2])
        _rec('align_target', color=str(color))
        return {'position': pos, 'bearing_error_deg': 0., 'range_error_m': 0.}

    def _deliver_garment(category='any', drop_x_m=0., drop_y_m=0., max_duration_s=300., allow_towing=False, dry_run=False):
        nonlocal arm_unrestored
        _rec('deliver_garment', category=str(category), drop_x_m=float(drop_x_m), drop_y_m=float(drop_y_m), dry_run=bool(dry_run))
        if 'garments' in overrides:
            seen = defaults['garments'](category)
            if not seen:
                return {'delivered': False, 'stowed': False, 'dry_run_valid': False, 'status_code': 'NO_CONFIRMED_GARMENT', 'final_position_m': [sim_pose[0], sim_pose[1]], 'detail': 'NO_CONFIRMED_GARMENT'}
            g0 = seen[0]
            suit = int(g0.get('transport_suitability', 1) if isinstance(g0, dict) else getattr(g0, 'transport_suitability', 1))
            if suit != 1:
                code = 'TOWING_UNQUALIFIED_SKIP' if (suit == 2 and allow_towing) else 'UNSUPPORTED_GARMENT_SKIP'
                return {'delivered': False, 'stowed': False, 'dry_run_valid': False, 'status_code': code, 'final_position_m': [sim_pose[0], sim_pose[1]], 'detail': code}
        if defaults['plan_route'](drop_x_m, drop_y_m, deployed_arm_extension_m=0.10) is None:
            return {'delivered': False, 'stowed': False, 'dry_run_valid': False, 'status_code': 'LOADED_ROUTE_BLOCKED', 'final_position_m': [sim_pose[0], sim_pose[1]], 'detail': 'LOADED_ROUTE_BLOCKED'}
        if not dry_run:
            arm_unrestored = False
            sim_pose[0], sim_pose[1] = float(drop_x_m), float(drop_y_m)
            sim_arm.update(state='IN_ROBOT', joints={'m3': 0., 'm4': 0., 'm5': 0., 'm6': 0.})
            sim_world.mark_completed(float(drop_x_m), float(drop_y_m), category=str(category), detail='DELIVERED')
        return {
            'delivered': not dry_run, 'stowed': not dry_run, 'dry_run_valid': bool(dry_run),
            'status_code': 'DRY_RUN_VALID' if dry_run else 'DELIVERED',
            'final_position_m': [sim_pose[0], sim_pose[1]], 'detail': '',
        }

    def _patrol_collect(drop_x_m=0., drop_y_m=0., max_duration_s=600., max_items=5, allow_towing=False, dry_run=False):
        nonlocal arm_unrestored
        _rec('patrol_collect', drop_x_m=float(drop_x_m), drop_y_m=float(drop_y_m), max_items=int(max_items), dry_run=bool(dry_run))
        if defaults['plan_route'](drop_x_m, drop_y_m, deployed_arm_extension_m=0.10) is None:
            return {
                'completed': False, 'partial_completion': False, 'dry_run_valid': False,
                'waypoints_visited': 0, 'waypoints_cleared': 0, 'waypoints_deferred': 0,
                'items_attempted': 0, 'items_delivered': 0, 'items_skipped': 0, 'items_failed': 0,
                'status_code': 'TARGET_OUTSIDE_INTERIOR', 'detail': 'TARGET_OUTSIDE_INTERIOR',
            }
        all_rooms = [r for r in (sc_base.get('rooms') or []) if isinstance(r, dict)]
        rooms = [r for r in all_rooms if r.get('reachable', True)]
        lost_rooms = len(all_rooms) - len(rooms)
        wp_cnt = max(1, len(rooms))
        seen = defaults['garments']('any') if 'garments' in overrides else []
        delivered = skipped = 0
        for g in seen:
            suit = int(g.get('transport_suitability', 1) if isinstance(g, dict) else getattr(g, 'transport_suitability', 1))
            cat = str((g.get('category') if isinstance(g, dict) else getattr(g, 'category', '')) or 'garment')
            if suit == 1 and delivered < int(max_items):
                delivered += 1
                if not dry_run:
                    sim_world.mark_completed(float(drop_x_m), float(drop_y_m), category=cat, detail='DELIVERED')
            else:
                skipped += 1
                if not dry_run:
                    sim_world.upsert_object(sim_pose[0], sim_pose[1], category=cat, mobility='draggable', relevance='deferred', detail='UNSUPPORTED_GARMENT_SKIP')
        if not dry_run:
            arm_unrestored = False
            sim_arm.update(state='IN_ROBOT', joints={'m3': 0., 'm4': 0., 'm5': 0., 'm6': 0.})
            if delivered > 0 or 'garments' not in overrides:
                sim_pose[0], sim_pose[1] = float(drop_x_m), float(drop_y_m)
        complete = not dry_run and skipped == 0 and lost_rooms == 0
        partial = bool(not dry_run and not complete)
        return {
            'completed': complete, 'partial_completion': partial, 'dry_run_valid': bool(dry_run),
            'waypoints_visited': 0 if dry_run else wp_cnt, 'waypoints_cleared': 0 if dry_run else wp_cnt, 'waypoints_deferred': 0,
            'items_attempted': 0 if dry_run else delivered, 'items_delivered': 0 if dry_run else delivered,
            'items_skipped': 0 if dry_run else skipped, 'items_failed': 0,
            'status_code': 'DRY_RUN_VALID' if dry_run else ('PATROL_CLEARED' if complete else 'PARTIAL_COMPLETION_SMALL_ITEMS_ONLY'),
            'detail': '',
        }

    def _select_camera(camera):
        if camera not in ('front', 'gripper'):
            raise ValueError("camera must be 'front' or 'gripper'")
        _rec('select_camera', camera=str(camera))
        return {'camera': str(camera)}

    def _plan_approach(x_m, y_m):
        try:
            return planner.approach([float(x_m), float(y_m), 0.])
        except ValueError:
            return None

    def _plan_grasp(x_m, y_m, size_m=.066):
        j = sim_arm.get('joints') or {}
        start = planner.snap_grid((j.get('m3', 150.), j.get('m4', 45.), j.get('m5', 0.)))[:2]
        try:
            return planner.grasp_goal([float(x_m), float(y_m), 0.], start, size_m=float(size_m))
        except ValueError:
            return None

    def _plan_stack_tier(tier, size_m=.066):
        j = sim_arm.get('joints') or {}
        start = planner.snap_grid((j.get('m3', 190.), j.get('m4', 65.), j.get('m5', 0.)))[:2]
        try:
            return planner.stack_tier_plan(int(tier), size_m=float(size_m), start=start)
        except ValueError:
            return None

    def _plan_trajectory(m3_deg, m4_deg, m5_deg=0.):
        j = sim_arm.get('joints') or {}
        start = (150, 45, 0) if sim_arm.get('state') == 'IN_ROBOT' else planner.snap_grid((
            j.get('m3', 150.), j.get('m4', 45.), j.get('m5', 0.),
        ))
        try:
            return planner.trajectory(start, (m3_deg, m4_deg, m5_deg))
        except ValueError:
            return None

    defaults: dict[str, Any] = dict(
        request=request,
        action_cls=spec.action_cls,
        directory=Path('.'),
        latest_observation=None,
        commands=None,
        executor=None,
        trace=trace,
        status=lambda: {'state': 3, 'error_code': 0, 'battery': 100, 'common_status': 51202},
        arm=lambda: deepcopy(sim_arm),
        arm_msg=_arm_msg,
        odom=lambda: {'x': sim_pose[0], 'y': sim_pose[1], 'yaw': sim_pose[2]},
        map_pose=lambda source='base_footprint': (sim_pose[0], sim_pose[1], sim_pose[2]),
        map_snapshot=lambda refresh=False, max_age_s=5.: None,
        safe_interior=lambda refresh=False, deployed_arm_extension_m=0.: (None, None),
        spatial_context=lambda refresh=False: {
            **sc_base,
            'robot_pose_map': {'x': sim_pose[0], 'y': sim_pose[1], 'yaw': sim_pose[2]},
            'world_objects': sim_world.summary(),
        },
        room_waypoint=_room_waypoint,
        plan_route=_plan_route,
        corridor_safe=_corridor_safe,
        select_camera=_select_camera,
        objects=lambda camera='front', color=None: [],
        wrist_pose=lambda: {'position': [0.35, 0., 0.08], 'marker_id': 68},
        fit_wrist_joints=lambda: {'joints': [190., 65., 0.], 'residual_m': 0., 'residual_deg': 0., 'uncertainty_m': 0.015},
        observe_color=lambda camera, color: [],
        garments=lambda category='any', after=None: [],
        world_objects=sim_world.summary,
        update_world_object=_update_world_object,
        mark_obstacle=_mark_obstacle,
        clear_world=_clear_world,
        navigate=_navigate,
        navigate_room=_navigate_room,
        pick=_pick,
        stack=_stack,
        deliver_garment=_deliver_garment,
        patrol_collect=_patrol_collect,
        deploy_arm=_deploy_arm,
        align_target=_align_target,
        arm_safe=lambda m3_deg, m4_deg, m5_deg=0., m6_deg=90.: planner.safe((m3_deg, m4_deg, m5_deg, m6_deg)),
        arm_kinematics=lambda m3_deg, m4_deg, m5_deg=0., m6_deg=90.: planner.arm_kinematics((m3_deg, m4_deg, m5_deg, m6_deg)),
        plan_approach=_plan_approach,
        plan_grasp=_plan_grasp,
        plan_stack_tier=_plan_stack_tier,
        plan_trajectory=_plan_trajectory,
        move_arm=_move_arm,
        arm_step=_arm_step,
        hand_step=_hand_step,
        open_jaws=_open_jaws,
        set_arm_anchor=_set_arm_anchor,
        grasp_and_lift=_grasp_and_lift,
        release_and_stow=_release_and_stow,
        pulse=_pulse,
        rpc=_rpc,
        stop=lambda: _rec('stop'),
        hazard=lambda: None,
        is_cancelled=lambda: False,
        sleep=lambda duration: None,
        log=lambda stage, **fields: None,
        feedback=lambda **fields: _rec('feedback', **fields),
        outcome=lambda success: CommandOutcome(error_code='NONE' if success else 'OBSERVATION_GAP'),
    )
    callable_keys = {p['name'] for p in GOAL_CONTEXT_PRIMITIVES}
    for k, v in overrides.items():
        if k == 'objects' and isinstance(v, list):
            defaults[k] = lambda camera='front', color=None, _v=v: [
                o for o in _v if isinstance(o, dict) and (color is None or o.get('color') == color)
            ]
            if 'observe_color' not in overrides:
                defaults['observe_color'] = lambda camera, color, _fn=defaults[k]: _fn(camera, color)
        elif k == 'garments' and isinstance(v, list):
            defaults[k] = lambda category='any', after=None, _v=v: [
                g for g in _v
                if category in ('', 'any') or (g.get('category') if isinstance(g, dict) else getattr(g, 'category', '')) == category
            ]
        else:
            defaults[k] = (lambda *_a, _v=v, **_kw: _v) if k in callable_keys and not callable(v) else v
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
        self.trace: list[dict] = []
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

    def _rec(self, primitive: str, **info) -> None:
        self.trace.append({'primitive': primitive, **info})

    def _ex(self) -> Executor:
        assert self.executor is not None
        return self.executor

    # --- State & Perception ---

    def status(self) -> dict:
        return read_status(self.observations)

    def arm(self) -> dict:
        return read_arm(self.observations)

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
        dyn = dynamic_obstacles(self.observations, snap, snap.map.robot[:2] if snap.map.robot else None)
        return snap, compute_safe_interior(snap.map, snap.labels, cfg, snap.map.robot, dynamic_obstacles=dyn)

    def spatial_context(self, refresh: bool = False) -> dict:
        snap, boundary = self.safe_interior(refresh=refresh)
        try:
            pose = self.map_pose()
        except LookupError:
            pose = snap.map.robot
        ctx = compute_spatial_context(snap.map, snap.labels, pose, boundary_result=boundary)
        ctx['world_objects'] = self.world_objects()
        return ctx

    def room_waypoint(self, room: str | int, refresh: bool = False) -> dict | None:
        snap, boundary = self.safe_interior(refresh=refresh)
        labels = snap.labels or {}
        target_seg = None
        if isinstance(room, int) and not isinstance(room, bool):
            target_seg = room
        else:
            q = str(room).strip().lower()
            seg_str = q.removeprefix('segment ').strip()
            if seg_str.isdigit():
                target_seg = int(seg_str)
            else:
                for seg, name in labels.items():
                    if str(name).strip().lower() == q:
                        target_seg = int(seg)
                        break
        if target_seg is None:
            return None
        candidates = [w for w in boundary.waypoints if int(w.get('segment_id', -1)) == target_seg]
        if not candidates:
            return None
        best = max(candidates, key=lambda w: float(w.get('clearance_m', 0.)))
        return {
            'segment_id': target_seg,
            'name': labels.get(target_seg, f'Segment {target_seg}'),
            'x': float(best['x']),
            'y': float(best['y']),
            'clearance_m': float(best.get('clearance_m', 0.)),
        }

    def plan_route(self, target_x_m: float, target_y_m: float, deployed_arm_extension_m: float = 0., refresh: bool = False) -> list[tuple[float, float]] | None:
        snap, boundary = self.safe_interior(refresh=refresh, deployed_arm_extension_m=deployed_arm_extension_m)
        if snap.map.robot is None:
            return None
        return loaded_route(boundary, snap.map.robot[:2], (float(target_x_m), float(target_y_m)))

    def corridor_safe(self, start_x_m: float, start_y_m: float, end_x_m: float, end_y_m: float,
                      radius_m: float = 0., deployed_arm_extension_m: float = 0., refresh: bool = False) -> bool:
        _, boundary = self.safe_interior(refresh=refresh, deployed_arm_extension_m=deployed_arm_extension_m)
        return check_corridor_containment(boundary, (float(start_x_m), float(start_y_m)), (float(end_x_m), float(end_y_m)), float(radius_m))

    def frame(self):
        return self.client.frame()

    def select_camera(self, camera: str) -> dict:
        res = self.observations.select(camera)
        self._rec('select_camera', camera=str(camera))
        return res

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
        image, tag = self.client.frame()
        if tag != 'front':
            raise LookupError('wrist_pose requires front camera frame')
        try:
            return perception.front_jaw(image)
        except LookupError:
            return None

    def fit_wrist_joints(self) -> dict | None:
        jaw = self.wrist_pose()
        return planner.observed_joints(jaw['hand_pose_base']) if jaw and 'hand_pose_base' in jaw else None

    def garments(self, category='any', after=None) -> list:
        batch = self.observations.garment_snapshot(after=after)
        self.latest_observation = batch.evidence
        wm = self.observations.world_map() if hasattr(self.observations, 'world_map') else None
        return graspable_garments(batch.garments, category, world=wm)

    def world_objects(self) -> dict:
        wm = self.observations.world_map() if hasattr(self.observations, 'world_map') else None
        return wm.summary() if wm is not None else {'session_bound': False, 'objectives': [], 'obstacles': [], 'completed': [], 'by_mobility': {'pickupable': [], 'draggable': [], 'stationary': []}, 'entries': []}

    def update_world_object(self, x_m: float, y_m: float, category: str = 'object', mobility: str = 'pickupable',
                            relevance: str = 'objective', radius_m: float = 0.12, detail: str = '') -> dict:
        if hasattr(self.observations, 'record_world_event'):
            self.observations.record_world_event(
                'upsert', x=float(x_m), y=float(y_m), radius_m=float(radius_m),
                category=str(category), mobility=str(mobility), relevance=str(relevance), detail=str(detail),
            )
        self._rec('update_world_object', x_m=float(x_m), y_m=float(y_m), category=str(category), mobility=str(mobility), relevance=str(relevance))
        return {'x': float(x_m), 'y': float(y_m), 'radius_m': float(radius_m), 'category': str(category), 'mobility': str(mobility), 'relevance': str(relevance), 'detail': str(detail)}

    def mark_obstacle(self, x_m: float, y_m: float, radius_m: float = 0.12, category: str = 'obstacle', detail: str = 'OBSTACLE') -> dict:
        self._rec('mark_obstacle', x_m=float(x_m), y_m=float(y_m), radius_m=float(radius_m), category=str(category))
        return self.update_world_object(x_m, y_m, category=category, mobility='stationary', relevance='deferred', radius_m=radius_m, detail=detail)

    def clear_world(self) -> dict:
        if hasattr(self.observations, 'record_world_event'):
            self.observations.record_world_event('clear')
        self._rec('clear_world')
        return {'cleared': True}

    # --- Motion & Manipulation Primitives ---

    def _run_with_feedback(self, runner, req) -> dict:
        orig_feedback = self.feedback
        def synced_feedback(**fields):
            self.latest_observation = runner.latest_observation
            orig_feedback(**fields)
        runner.feedback = synced_feedback
        data = runner.execute(req)
        self.latest_observation = runner.latest_observation
        return data

    def navigate(self, target_x_m: float, target_y_m: float, max_duration_s: float = 60., dry_run: bool = False) -> dict:
        self._rec('navigate', target_x_m=float(target_x_m), target_y_m=float(target_y_m), dry_run=bool(dry_run))
        nav = PinAndGoNavigator(
            self.observations,
            self.commands,
            boundary_config=self.server.navigation_config,
            cancelled=self.is_cancelled,
            hazard=lambda: self.server.safety_stop,
            odom=self.odom,
            feedback=self.feedback,
        )
        return self._run_with_feedback(nav, SimpleNamespace(
            target_x_m=float(target_x_m),
            target_y_m=float(target_y_m),
            max_duration_s=float(max_duration_s),
            dry_run=bool(dry_run),
        ))

    def navigate_room(self, room: str | int, max_duration_s: float = 60., dry_run: bool = False) -> dict:
        self._rec('navigate_room', room=room, dry_run=bool(dry_run))
        wp = self.room_waypoint(room, refresh=True)
        if wp is None:
            return {'arrived': False, 'dry_run_valid': False, 'status_code': 'ROOM_UNREACHABLE', 'room': room, 'detail': 'ROOM_UNREACHABLE'}
        res = self.navigate(wp['x'], wp['y'], max_duration_s=max_duration_s, dry_run=dry_run)
        return {**res, 'room': wp['name'], 'segment_id': wp['segment_id'], 'target_x_m': wp['x'], 'target_y_m': wp['y']}

    def _target_map_xy(self, rel_pos=None) -> tuple[float, float]:
        try:
            pose = self.map_pose()
        except Exception:
            pose = (0., 0., 0.)
        rx, ry = (float(rel_pos[0]), float(rel_pos[1])) if isinstance(rel_pos, (list, tuple)) and len(rel_pos) >= 2 else (0.35, 0.)
        return transform_xy((rx, ry), pose)

    def pick(self, color: str, dry_run: bool = False) -> dict:
        self._rec('pick', color=str(color), dry_run=bool(dry_run))
        res = self._ex().pick(color, dry_run=dry_run)
        if not dry_run and res.get('verified') and hasattr(self.observations, 'record_world_event'):
            tpos = (res.get('target') or {}).get('position') if isinstance(res.get('target'), dict) else None
            mx, my = self._target_map_xy(tpos)
            self.observations.record_world_event('completed', x=mx, y=my, category=str(color), mobility='pickupable', detail='PICKED')
        return res

    def stack(self, base_color: str, middle_color: str, top_color: str, dry_run: bool = False) -> dict:
        self._rec('stack', base_color=str(base_color), middle_color=str(middle_color), top_color=str(top_color), dry_run=bool(dry_run))
        ex = self._ex()
        ex.feedback = lambda _phase: self.feedback(phase=3, current_tier=1, reason='NONE')
        res = ex.stack(base_color, middle_color, top_color, dry_run=dry_run)
        if not dry_run and res.get('stack_verified') and hasattr(self.observations, 'record_world_event'):
            mx, my = self._target_map_xy(res.get('tower_spot'))
            self.observations.record_world_event('completed', x=mx, y=my, category=f'stack:{base_color}+{middle_color}+{top_color}', mobility='pickupable', detail='STACKED')
        return res

    def deliver_garment(self, category: str = 'any', drop_x_m: float = 0., drop_y_m: float = 0.,
                        max_duration_s: float = 300., allow_towing: bool = False, dry_run: bool = False) -> dict:
        self._rec('deliver_garment', category=str(category), drop_x_m=float(drop_x_m), drop_y_m=float(drop_y_m), dry_run=bool(dry_run))
        runner = GarmentDeliverer(
            self.observations, self.executor,
            boundary_config=self.server.navigation_config,
            towing_config=getattr(self.server, 'towing_config', None),
            feedback=self.feedback,
        )
        res = self._run_with_feedback(runner, SimpleNamespace(
            category=str(category), drop_x_m=float(drop_x_m), drop_y_m=float(drop_y_m),
            max_duration_s=float(max_duration_s), allow_towing=bool(allow_towing), dry_run=bool(dry_run),
        ))
        if not dry_run and hasattr(self.observations, 'record_world_event'):
            tid = track_key(runner.track or res.get('track_id', ''))
            mob = mobility_from_suitability(int(getattr(runner.track, 'transport_suitability', 1))) if runner.track is not None else 'pickupable'
            if res.get('delivered'):
                self.observations.record_world_event(
                    'completed', x=float(drop_x_m), y=float(drop_y_m),
                    track_id=tid, category=str(category), mobility=mob, detail='DELIVERED',
                )
            elif runner.track is not None:
                p = runner.track.floor_contact_m
                ox, oy = transform_xy((p.x, p.y), getattr(runner, 'current_pose', (0., 0., 0.)))
                if res.get('status_code') in ('TOWING_UNQUALIFIED_SKIP', 'UNSUPPORTED_GARMENT_SKIP', 'TOWING_REQUIRES_SUPERVISED_TRIAL'):
                    self.observations.record_world_event(
                        'upsert', x=float(ox), y=float(oy),
                        radius_m=0.14, category=str(runner.track.category), mobility=mob, relevance='deferred', detail=res['status_code'], track_id=tid,
                    )
                else:
                    self.observations.record_world_event(
                        'failed', x=float(ox), y=float(oy),
                        track_id=tid, detail=str(res.get('status_code', 'FAILED')),
                    )
        return res

    def patrol_collect(self, drop_x_m: float = 0., drop_y_m: float = 0., max_duration_s: float = 600.,
                       max_items: int = 5, allow_towing: bool = False, dry_run: bool = False) -> dict:
        self._rec('patrol_collect', drop_x_m=float(drop_x_m), drop_y_m=float(drop_y_m), max_items=int(max_items), dry_run=bool(dry_run))
        runner = PatrolCollector(
            self.observations, self.executor, PatrolLedger(self.server.patrol_directory),
            boundary_config=self.server.navigation_config,
            towing_config=getattr(self.server, 'towing_config', None),
            feedback=self.feedback,
        )
        return self._run_with_feedback(runner, SimpleNamespace(
            drop_x_m=float(drop_x_m), drop_y_m=float(drop_y_m), max_duration_s=float(max_duration_s),
            max_items=int(max_items), allow_towing=bool(allow_towing), dry_run=bool(dry_run),
        ))

    def deploy_arm(self, settle=True) -> dict:
        self._rec('deploy_arm', settle=bool(settle))
        return self._ex().deploy(settle=settle)

    def align_target(self, color: str) -> dict:
        self._rec('align_target', color=str(color))
        return self._ex().align_front(color)

    def arm_safe(self, m3_deg: float, m4_deg: float, m5_deg: float = 0., m6_deg: float = 90.) -> bool:
        return planner.safe((float(m3_deg), float(m4_deg), float(m5_deg), float(m6_deg)))

    def arm_kinematics(self, m3_deg: float, m4_deg: float, m5_deg: float = 0., m6_deg: float = 90.) -> dict:
        return planner.arm_kinematics((float(m3_deg), float(m4_deg), float(m5_deg), float(m6_deg)))

    def plan_approach(self, x_m: float, y_m: float) -> dict | None:
        try:
            return planner.approach([float(x_m), float(y_m), 0.])
        except ValueError:
            return None

    def _grid_joints(self) -> tuple[int, int, int]:
        arm = self.arm()
        joints = arm.get('joints') or {}
        if all(k in joints for k in ('m3', 'm4')):
            return planner.snap_grid((joints['m3'], joints['m4'], joints.get('m5', 0)))
        if arm.get('state') == 'UNKNOWN':
            try:
                fit = self.fit_wrist_joints()
                if fit and 'joints' in fit:
                    return planner.snap_grid(fit['joints'])
            except LookupError:
                pass
        return (150, 45, 0)

    def plan_grasp(self, x_m: float, y_m: float, size_m: float = .066) -> dict | None:
        start = self._grid_joints()[:2]
        try:
            return planner.grasp_goal([float(x_m), float(y_m), 0.], start, size_m=float(size_m))
        except ValueError:
            return None

    def plan_stack_tier(self, tier: int, size_m: float = .066) -> dict | None:
        start = self._grid_joints()[:2]
        try:
            return planner.stack_tier_plan(int(tier), size_m=float(size_m), start=start)
        except ValueError:
            return None

    def plan_trajectory(self, m3_deg: float, m4_deg: float, m5_deg: float = 0.) -> list[dict] | None:
        start = self._grid_joints()
        try:
            return planner.trajectory(start, (m3_deg, m4_deg, m5_deg))
        except ValueError:
            return None

    def move_arm(self, m3_deg: float, m4_deg: float, m5_deg: float = 0.) -> dict:
        res = self._ex().move_arm((m3_deg, m4_deg, m5_deg))
        steps_exec = res.get('steps_executed', 0)
        self._rec('move_arm', target=[round(float(m3_deg)), round(float(m4_deg)), round(float(m5_deg))], steps_executed=int(steps_exec) if isinstance(steps_exec, (int, float)) else 0)
        return res

    def arm_step(self, m3: int = 0, m4: int = 0, m5: int = 0) -> dict:
        if any(v not in (0, 1, 2) for v in (m3, m4, m5)) or not any((m3, m4, m5)):
            raise ValueError('arm_step axes must be in (0, 1, 2) with at least one nonzero axis')
        try:
            j = (self.arm() or {}).get('joints') or {}
        except Exception:
            j = {}
        if all(k in j for k in ('m3', 'm4')):
            cur = (float(j['m3']), float(j['m4']), float(j.get('m5', 0.)), float(j.get('m6', 90.)))
            deltas = [10. if s == 1 else -10. if s == 2 else 0. for s in (m3, m4, m5)]
            nxt = (cur[0] + deltas[0], cur[1] + deltas[1], cur[2] + deltas[2], cur[3])
            if not (planner.safe(nxt) and planner.safe_edge(cur, nxt)):
                raise ValueError(f'unsafe arm_step from {cur[:3]} to {nxt[:3]}')
        self._rec('arm_step', m3=int(m3), m4=int(m4), m5=int(m5))
        return self.rpc('app_arm_move', {'m': [int(m3), int(m4), int(m5)]}, timeout_ms=3000)

    def hand_step(self, direction: int) -> dict:
        if direction not in (1, 2, 3, 4):
            raise ValueError('hand_step direction must be in (1, 2, 3, 4)')
        try:
            j = (self.arm() or {}).get('joints') or {}
        except Exception:
            j = {}
        if all(k in j for k in ('m3', 'm4')):
            delta = {1: (0, 1), 2: (0, -1), 3: (1, 0), 4: (-1, 0)}[int(direction)]
            m3, m4, m5, m6 = float(j['m3']), float(j['m4']), float(j.get('m5', 0.)), float(j.get('m6', 90.))
            x, y = vendor_forward(m3, m4)[1]
            sol = vendor_inverse(x + delta[0] * HAND_STEP_M, y + delta[1] * HAND_STEP_M, (m3, m4))
            if sol is None or vendor_check(sol[0], sol[1], m5) != 'SUCCESS' or not planner.safe((sol[0], sol[1], m5, m6)):
                raise ValueError(f'unreachable hand_step direction {direction}')
        self._rec('hand_step', direction=int(direction))
        return self.rpc('app_arm_direction_move', {'direction': int(direction)}, timeout_ms=3000)

    def open_jaws(self):
        self._rec('open_jaws')
        self._ex().open_jaws()

    def set_arm_anchor(self, anchor: str, reason: str = '') -> dict:
        if not self.commands:
            raise PermissionError('dry-run forbids arm re-anchoring')
        self._rec('set_arm_anchor', anchor=str(anchor), reason=str(reason))
        return self.commands.set_arm_anchor(anchor, reason)

    def grasp_and_lift(self, lift_steps: int = 2, verify_color: str = '') -> dict:
        self._rec('grasp_and_lift', lift_steps=int(lift_steps), verify_color=str(verify_color))
        return self._ex().grasp_and_lift(lift_steps=lift_steps, verify_color=verify_color)

    def release_and_stow(self, put_down: bool = True) -> dict:
        self._rec('release_and_stow', put_down=bool(put_down))
        return self._ex().release_and_stow(put_down=put_down)

    def pulse(self, velocity: float = 0., omega: float = 0., duration: int = 300):
        self._rec('pulse', velocity=float(velocity), omega=float(omega), duration=int(duration))
        return self._ex().pulse(velocity=velocity, omega=omega, duration=duration)

    def rpc(self, method: str, params=None, timeout_ms: int = 5000) -> dict:
        self._rec('rpc', method=str(method), params=params)
        return self.client.rpc(method, [] if params is None else params, timeout_ms=timeout_ms)

    def stop(self):
        self._rec('stop')
        if self.commands:
            self.commands.stop()

    def log(self, stage: str, **fields):
        if self.executor:
            self.executor.log(stage, **fields)
        elif self.directory:
            self.directory.mkdir(parents=True, exist_ok=True)
            row = dict(t=datetime.now(timezone.utc).isoformat(), event=stage, **fields)
            with (self.directory / 'timeline.jsonl').open('a') as f:
                f.write(json.dumps(row, allow_nan=False) + '\n')

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

def _valid_pin_and_go_payload(p: dict) -> bool:
    return (_num_in(p['target_x_m'], -100., 100.) and _num_in(p['target_y_m'], -100., 100.)
            and _num_in(p['max_duration_s'], 1., 600.) and type(p['dry_run']) is bool)


def _valid_garment_payload(p: dict) -> bool:
    return (_num_in(p['drop_x_m'], -100., 100.) and _num_in(p['drop_y_m'], -100., 100.)
            and _num_in(p['max_duration_s'], 1., 3600.) and type(p['allow_towing']) is bool
            and type(p['dry_run']) is bool)


def _valid_garment_goal(_server, r) -> bool:
    return (_valid_garment_payload({
        'drop_x_m': r.drop_x_m, 'drop_y_m': r.drop_y_m, 'max_duration_s': r.max_duration_s,
        'allow_towing': bool(r.allow_towing), 'dry_run': bool(r.dry_run),
    }) and (not hasattr(r, 'max_items') or _int_in(int(r.max_items), 1, 1000)))


def _goal_curated(ctx: 'GoalContext | SimpleNamespace', fn, build_result, ok_pred=lambda _d: True):
    outcome_override = None
    try:
        data = fn(ctx)
        success = bool(ok_pred(data))
    except Exception as e:
        data = dict(error=type(e).__name__, reason=str(e), log_dir=str(ctx.directory))
        ctx.log('failure', **data)
        success = False
        if isinstance(e, ValueError):
            outcome_override = CommandOutcome(error_code='INVALID_INPUT')
    return success, build_result(outcome_override or ctx.outcome(success), success, data)


def _goal_pick(ctx: 'GoalContext | SimpleNamespace', req):
    return _goal_curated(
        ctx,
        lambda c: c.pick(req.color, dry_run=req.dry_run),
        lambda outcome, ok, report: Pick.Result(
            outcome=outcome,
            verified_hold=bool(report.get('verified')),
            dry_run_valid=bool(req.dry_run and ok),
            evidence_uri=ctx.directory.name,
        ),
        lambda r: r.get('verified') or (r.get('dry_run') and r.get('target') is not None),
    )


def _goal_stack(ctx: 'GoalContext | SimpleNamespace', req):
    return _goal_curated(
        ctx,
        lambda c: c.stack(req.base_color, req.middle_color, req.top_color, dry_run=req.dry_run),
        lambda outcome, ok, report: Stack.Result(
            outcome=outcome,
            tiers_completed=3 if (req.dry_run and ok) else int(report.get('tiers_completed', 0)),
            stack_verified=bool(report.get('stack_verified')),
            dry_run_valid=bool(req.dry_run and ok),
            evidence_uri=ctx.directory.name,
        ),
        lambda r: r.get('stack_verified') or (r.get('dry_run') and r.get('all_targets_visible')),
    )


def _build_ros_result(result_cls, **fields):
    allowed = result_cls.get_fields_and_field_types()
    return result_cls(**{k: v for k, v in fields.items() if k in allowed})


def _nav_outcome(ctx: 'GoalContext | SimpleNamespace', success: bool, data: dict) -> CommandOutcome:
    out = ctx.outcome(success)
    if not success and out.error_code == 'OBSERVATION_GAP' and data.get('status_code'):
        out.error_code = str(data['status_code'])
    return out


def _goal_pin_and_go(ctx: 'GoalContext | SimpleNamespace', req):
    data = ctx.navigate(req.target_x_m, req.target_y_m, req.max_duration_s, dry_run=req.dry_run)
    success = bool(data.get('arrived') or data.get('dry_run_valid'))
    return success, _build_ros_result(PinAndGo.Result, outcome=_nav_outcome(ctx, success, data), **data)


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
    return success, _build_ros_result(action_cls.Result, outcome=_nav_outcome(ctx, success, data), **data)


BUILTIN_GOALS: dict[str, GoalSpec] = {
    spec.name: spec for spec in (
        GoalSpec(
            'pick', Pick, '/z70/pick', 285.,
            lambda s, r: r.color in s.thresholds,
            _goal_pick,
            fields=('color', 'dry_run'),
            validate_payload=lambda p: _is_color(p['color']) and type(p['dry_run']) is bool,
        ),
        GoalSpec(
            'stack', Stack, '/z70/stack', lambda r: 60. if r.dry_run else 540.,
            lambda s, r: len({r.base_color, r.middle_color, r.top_color}) == 3 and all(c in s.thresholds for c in (r.base_color, r.middle_color, r.top_color)),
            _goal_stack,
            fields=('base_color', 'middle_color', 'top_color', 'dry_run'),
            validate_payload=lambda p: (all(_is_color(p[k]) for k in ('base_color', 'middle_color', 'top_color'))
                                        and len({p['base_color'], p['middle_color'], p['top_color']}) == 3
                                        and type(p['dry_run']) is bool),
        ),
        GoalSpec(
            'pin_and_go', PinAndGo, '/z70/nav/pin_and_go', lambda r: r.max_duration_s,
            lambda _s, r: _valid_pin_and_go_payload({
                'target_x_m': r.target_x_m, 'target_y_m': r.target_y_m,
                'max_duration_s': r.max_duration_s, 'dry_run': bool(r.dry_run),
            }),
            _goal_pin_and_go,
            needs_executor=lambda _: False,
            navigation=True,
            fields=('target_x_m', 'target_y_m', 'max_duration_s', 'dry_run'),
            validate_payload=_valid_pin_and_go_payload,
        ),
        GoalSpec(
            'deliver_garment', DeliverGarment, '/z70/deliver_garment', lambda r: r.max_duration_s,
            _valid_garment_goal,
            lambda ctx, r: _goal_garment(ctx, r, DeliverGarment, patrol=False),
            needs_executor=lambda r: not r.dry_run,
            navigation=True,
            fields=('category', 'drop_x_m', 'drop_y_m', 'max_duration_s', 'allow_towing', 'dry_run'),
            validate_payload=lambda p: _valid_garment_payload(p) and isinstance(p['category'], str) and len(p['category']) <= 32,
        ),
        GoalSpec(
            'patrol_collect', PatrolCollect, '/z70/patrol_collect', lambda r: r.max_duration_s,
            _valid_garment_goal,
            lambda ctx, r: _goal_garment(ctx, r, PatrolCollect, patrol=True),
            needs_executor=lambda r: not r.dry_run,
            navigation=True,
            fields=('drop_x_m', 'drop_y_m', 'max_duration_s', 'max_items', 'allow_towing', 'dry_run'),
            validate_payload=lambda p: _valid_garment_payload(p) and _int_in(p['max_items'], 1, 1000),
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
            validate_payload=lambda p: _is_color(p['color']),
        ),
        GoalSpec(
            'move_arm', MoveArm, '/z70/arm/move', 120.,
            lambda s, r: planner.safe((r.m3_deg, r.m4_deg, r.m5_deg, 90)),
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
            validate_payload=lambda p: (
                _num_in(p['m3_deg'], 90, 270) and _num_in(p['m4_deg'], 35, 170) and _num_in(p['m5_deg'], 0, 155)
                and planner.safe((p['m3_deg'], p['m4_deg'], p['m5_deg'], 90))
            ),
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


def _run_driver_arm_step(ctx: 'GoalContext | SimpleNamespace', r):
    return ctx.arm_step(*arm_step_vector(int(r.joint), int(r.direction)))


def _run_driver_arm_op(ctx: 'GoalContext | SimpleNamespace', r):
    op = int(r.operation)
    if op == 1:
        return ctx.deploy_arm(settle=False)
    if op == 2:
        return ctx.release_and_stow(put_down=False)
    if op == 3:
        ctx.open_jaws(); return {'ok': True}
    method, params = ARM_OPERATION_RPCS[op]
    return ctx.rpc(method, params)


DRIVER_GOALS: dict[str, GoalSpec] = {
    spec.name: spec for spec in (
        GoalSpec(
            'base_pulse', BasePulse, '/z70/base/pulse', 15.,
            lambda _s, r: _valid_pulse_payload({
                'linear_mps': r.linear_mps, 'angular_radps': r.angular_radps,
                'duration': r.duration if isinstance(r.duration, dict) else {'sec': r.duration.sec, 'nanosec': r.duration.nanosec},
            }),
            lambda ctx, r: _goal_curated(
                ctx,
                lambda c: c.pulse(
                    velocity=r.linear_mps,
                    omega=r.angular_radps,
                    duration=round(
                        (r.duration['sec'] if isinstance(r.duration, dict) else r.duration.sec) * 1000
                        + (r.duration['nanosec'] if isinstance(r.duration, dict) else r.duration.nanosec) / 1_000_000
                    ),
                ),
                lambda outcome, _ok, _d: BasePulse.Result(outcome=outcome),
            ),
            read_only=lambda _: False,
            fields=('linear_mps', 'angular_radps', 'duration'),
            validate_payload=_valid_pulse_payload,
            description='Execute a bounded linear or angular chassis RC pulse (<=0.8 s, |v|<=0.1 m/s, |w|<=0.5 rad/s).',
        ),
        GoalSpec(
            'arm_step', ArmStep, '/z70/arm/step', 35.,
            lambda _s, r: r.joint in (3, 4, 5, 34, 43) and r.direction in (-1, 1),
            lambda ctx, r: _goal_curated(
                ctx, lambda c: _run_driver_arm_step(c, r),
                lambda outcome, _ok, _d: ArmStep.Result(outcome=outcome, estimate=ctx.arm_msg()),
            ),
            read_only=lambda _: False,
            fields=('joint', 'direction'),
            validate_payload=lambda p: (type(p['joint']) is int and p['joint'] in (3, 4, 5, 34, 43)
                                        and type(p['direction']) is int and p['direction'] in (-1, 1)),
            description='Step arm joint M3 (3), M4 (4), M5 (5), or coupled M3+M4 (34/43) by +/-10 deg.',
        ),
        GoalSpec(
            'hand_step', HandStep, '/z70/arm/hand_step', 35.,
            lambda _s, r: 1 <= r.direction <= 4,
            lambda ctx, r: _goal_curated(
                ctx, lambda c: c.hand_step(r.direction),
                lambda outcome, _ok, _d: HandStep.Result(outcome=outcome, estimate=ctx.arm_msg()),
            ),
            read_only=lambda _: False,
            fields=('direction',),
            validate_payload=lambda p: _int_in(p['direction'], 1, 4),
            description='Step end-effector along +Y (1), -Y (2), +X (3), or -X (4) by 0.04 m via sagittal IK.',
        ),
        GoalSpec(
            'arm_operation', ArmOperation, '/z70/arm/operation', 60.,
            lambda _s, r: 1 <= r.operation <= 9,
            lambda ctx, r: _goal_curated(
                ctx, lambda c: _run_driver_arm_op(c, r),
                lambda outcome, _ok, _d: ArmOperation.Result(outcome=outcome, estimate=ctx.arm_msg()),
            ),
            read_only=lambda _: False,
            fields=('operation',),
            validate_payload=lambda p: _int_in(p['operation'], 1, 9),
            description='Execute vendor arm operation (1=deploy, 2=stow, 3=open_gripper, 4=grip, 5=put_down, 6=enter_pickup, 7=manual_arm_mode, 8=exit_pickup, 9=recover).',
        ),
    )
}

ALL_GOALS: dict[str, GoalSpec] = {**BUILTIN_GOALS, **DRIVER_GOALS}
