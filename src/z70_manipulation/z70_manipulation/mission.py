"""Bounded sequential mission runner over an existing GoalContext.

Executes 1..8 declarative steps from the closed V1 allowlist
(`navigate_room`, `navigate`, `observe`, `pick`) under a single borrowed
`GoalContext` and driver task session without opening nested ROS action goals
or issuing automatic failure-recovery motion.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
import re
import time
from typing import Any

from z70_interfaces.action import RunMission
from z70_interfaces.msg import CommandOutcome

ALLOWED_MISSION_OPS: tuple[str, ...] = ('navigate_room', 'navigate', 'observe', 'pick')
MAX_MISSION_STEPS: int = 8
MAX_STEPS_JSON_BYTES: int = 4096
MAX_OBSERVED_OBJECTS: int = 16
MISSION_FIELDS: tuple[str, ...] = ('steps_json', 'max_duration_s', 'dry_run')
MISSION_DESCRIPTION: str = (
    'Execute a bounded sequential mission of 1..8 steps '
    '(navigate_room, navigate, observe, pick) under one task session and deadline.'
)

__all__ = [
    'ALLOWED_MISSION_OPS',
    'MAX_MISSION_STEPS',
    'MAX_OBSERVED_OBJECTS',
    'MAX_STEPS_JSON_BYTES',
    'MISSION_DESCRIPTION',
    'MISSION_FIELDS',
    'parse_mission_steps',
    'run_mission',
    'validate_mission_goal',
    'validate_mission_payload',
]

_COLOR_RE = re.compile(r'[a-zA-Z0-9_-]{1,32}')
_FEEDBACK_FIELDS = frozenset(RunMission.Feedback.get_fields_and_field_types().keys())


def _is_num(value: Any) -> bool:
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(float(value))
    except (OverflowError, ValueError, TypeError):
        return False


def _num_in(value: Any, low: float, high: float) -> bool:
    if not _is_num(value):
        return False
    val = float(value)
    return low <= val <= high


def _valid_color(value: Any, allowed_colors=None) -> bool:
    if not (isinstance(value, str) and bool(_COLOR_RE.fullmatch(value))):
        return False
    return allowed_colors is None or value in allowed_colors


def parse_mission_steps(steps_raw: Any, *, allowed_colors=None) -> list[dict[str, Any]]:
    """Parse and strictly validate a 1..8 step declarative mission plan before execution."""
    if isinstance(steps_raw, str):
        if not steps_raw or len(steps_raw.encode('utf-8')) > MAX_STEPS_JSON_BYTES:
            raise ValueError('steps_json must be a non-empty JSON array <= 4096 bytes')
        try:
            parsed = json.loads(steps_raw)
        except (ValueError, TypeError, OverflowError) as exc:
            raise ValueError('steps_json is not valid JSON') from exc
    elif isinstance(steps_raw, (list, tuple)):
        parsed = list(steps_raw)
    else:
        raise ValueError('mission steps must be a JSON array string or list')

    if not isinstance(parsed, list) or not (1 <= len(parsed) <= MAX_MISSION_STEPS):
        raise ValueError(f'mission requires 1..{MAX_MISSION_STEPS} steps')

    normalized: list[dict[str, Any]] = []
    for idx, raw in enumerate(parsed):
        if not isinstance(raw, dict):
            raise ValueError(f'step {idx} must be an object')
        op = raw.get('op')
        if not isinstance(op, str) or op not in ALLOWED_MISSION_OPS:
            raise ValueError(f'step {idx} has unsupported op {op!r}')

        if op == 'navigate_room':
            if set(raw) - {'op', 'room', 'max_duration_s'} or 'room' not in raw:
                raise ValueError(f'step {idx} (navigate_room) has invalid fields')
            room = raw['room']
            valid_room = (
                (type(room) is int and 1 <= room <= 1000)
                or (isinstance(room, str) and 1 <= len(room.strip()) <= 64)
            )
            if not valid_room:
                raise ValueError(f'step {idx} (navigate_room) requires room name or positive segment id')
            step_dict: dict[str, Any] = {'op': 'navigate_room', 'room': room.strip() if isinstance(room, str) else int(room)}
            if 'max_duration_s' in raw:
                if not _num_in(raw['max_duration_s'], 1.0, 600.0):
                    raise ValueError(f'step {idx} (navigate_room) max_duration_s out of range 1..600')
                step_dict['max_duration_s'] = float(raw['max_duration_s'])
            normalized.append(step_dict)

        elif op == 'navigate':
            if set(raw) - {'op', 'target_x_m', 'target_y_m', 'max_duration_s'} or not {'target_x_m', 'target_y_m'} <= set(raw):
                raise ValueError(f'step {idx} (navigate) has invalid fields')
            if not (_num_in(raw['target_x_m'], -100.0, 100.0) and _num_in(raw['target_y_m'], -100.0, 100.0)):
                raise ValueError(f'step {idx} (navigate) coordinates out of range -100..100')
            step_dict = {
                'op': 'navigate',
                'target_x_m': float(raw['target_x_m']),
                'target_y_m': float(raw['target_y_m']),
            }
            if 'max_duration_s' in raw:
                if not _num_in(raw['max_duration_s'], 1.0, 600.0):
                    raise ValueError(f'step {idx} (navigate) max_duration_s out of range 1..600')
                step_dict['max_duration_s'] = float(raw['max_duration_s'])
            normalized.append(step_dict)

        elif op == 'observe':
            if set(raw) - {'op', 'camera', 'color', 'require_visible'}:
                raise ValueError(f'step {idx} (observe) has unknown fields')
            camera = raw.get('camera', 'front')
            if camera not in ('front', 'gripper'):
                raise ValueError(f'step {idx} (observe) camera must be front or gripper')
            color = raw.get('color')
            if color in ('', None):
                color = None
            elif not _valid_color(color, allowed_colors):
                raise ValueError(f'step {idx} (observe) has invalid color {color!r}')
            req_vis = raw.get('require_visible', color is not None)
            if type(req_vis) is not bool:
                raise ValueError(f'step {idx} (observe) require_visible must be bool')
            normalized.append({
                'op': 'observe',
                'camera': str(camera),
                'color': color,
                'require_visible': req_vis,
            })

        elif op == 'pick':
            if set(raw) - {'op', 'color'} or 'color' not in raw:
                raise ValueError(f'step {idx} (pick) requires only op and color')
            if not _valid_color(raw['color'], allowed_colors):
                raise ValueError(f'step {idx} (pick) has invalid color {raw["color"]!r}')
            normalized.append({'op': 'pick', 'color': str(raw['color'])})

    return normalized


def validate_mission_payload(payload: dict, *, allowed_colors=None) -> bool:
    """Validate a raw HTTP/MCP goal payload dict for `RunMission`."""
    if not isinstance(payload, dict) or set(payload) != set(MISSION_FIELDS):
        return False
    if not isinstance(payload.get('steps_json'), str):
        return False
    if not _num_in(payload.get('max_duration_s'), 1.0, 1800.0):
        return False
    if type(payload.get('dry_run')) is not bool:
        return False
    try:
        parse_mission_steps(payload['steps_json'], allowed_colors=allowed_colors)
    except (ValueError, TypeError, OverflowError):
        return False
    return True


def validate_mission_goal(server: Any, req: Any) -> bool:
    """Validate a ROS `RunMission.Goal` (or namespace) against server color thresholds."""
    thresholds = getattr(server, 'thresholds', None)
    steps_json = getattr(req, 'steps_json', None)
    max_duration_s = getattr(req, 'max_duration_s', None)
    dry_run = getattr(req, 'dry_run', None)
    if not isinstance(steps_json, str) or not _num_in(max_duration_s, 1.0, 1800.0) or type(dry_run) is not bool:
        return False
    try:
        parse_mission_steps(steps_json, allowed_colors=thresholds)
    except (ValueError, TypeError, OverflowError):
        return False
    return True


def _bounded_objects(raw_objs: Any) -> list[dict[str, Any]]:
    if not isinstance(raw_objs, (list, tuple)):
        return []
    items: list[dict[str, Any]] = []
    for obj in raw_objs[:MAX_OBSERVED_OBJECTS]:
        if not isinstance(obj, dict):
            continue
        clean: dict[str, Any] = {}
        if isinstance(obj.get('color'), str):
            clean['color'] = obj['color'][:32]
        pos = obj.get('position')
        if isinstance(pos, (list, tuple)) and len(pos) >= 2 and all(_is_num(v) for v in pos[:3]):
            clean['position'] = [round(float(v), 4) for v in pos[:3]]
        if _is_num(obj.get('size_m')):
            clean['size_m'] = round(float(obj['size_m']), 4)
        if _is_num(obj.get('area_px')):
            clean['area_px'] = round(float(obj['area_px']), 1)
        if isinstance(obj.get('camera'), str):
            clean['camera'] = obj['camera'][:16]
        items.append(clean)
    return items


def _classify_interrupt(ctx: Any, deadline: float, clock, exc: BaseException | None = None) -> tuple[str, str] | None:
    cancelled_fn = getattr(ctx, 'is_cancelled', None)
    hazard_fn = getattr(ctx, 'hazard', None)
    hazard_reason = hazard_fn() if callable(hazard_fn) else None
    is_cancelled = bool(callable(cancelled_fn) and cancelled_fn())
    expired = clock() >= deadline
    handle = getattr(ctx, 'handle', None)
    cancel_requested = bool(getattr(handle, 'is_cancel_requested', False))

    if hazard_reason:
        text = str(hazard_reason).strip()
        lower = text.lower()
        if 'cancel' in lower:
            return 'CANCELLED', (text or 'Operator cancelled mission')[:128]
        if 'deadline' in lower or 'timed out' in lower or 'timeout' in lower:
            return 'TIMED_OUT', (text or 'Mission deadline exceeded')[:128]
        if lower == 'task revoked':
            if expired and not cancel_requested:
                return 'TIMED_OUT', 'Mission deadline exceeded'
            return 'CANCELLED', text[:128]
        return 'SAFETY_STOP', (text or 'SAFETY_STOP')[:128]

    if exc is not None:
        text = str(exc).strip()
        lower = text.lower()
        if type(exc).__name__ == 'SafetyStop':
            if 'cancel' in lower or 'revoked' in lower:
                return 'CANCELLED', (text or 'Operator cancelled mission')[:128]
            if 'deadline' in lower or 'timed out' in lower or 'timeout' in lower:
                return 'TIMED_OUT', (text or 'Mission deadline exceeded')[:128]
            return 'SAFETY_STOP', (text or 'SAFETY_STOP')[:128]
        if isinstance(exc, InterruptedError):
            if 'safety' in lower or 'person' in lower or 'robot error' in lower or 'unexpected clean' in lower:
                return 'SAFETY_STOP', (text or 'SAFETY_STOP')[:128]
            if is_cancelled or 'cancel' in lower or 'revoked' in lower:
                detail_text = text if (text and text != 'CANCELLED_OR_DEADLINE') else 'Operator cancelled mission'
                return 'CANCELLED', detail_text[:128]
            if 'deadline' in lower or 'timed out' in lower or 'timeout' in lower or expired:
                return 'TIMED_OUT', 'Mission deadline exceeded'
            return 'CANCELLED', (text or 'CANCELLED')[:128]

    if is_cancelled:
        return 'CANCELLED', 'Operator cancelled mission'
    if expired:
        return 'TIMED_OUT', 'Mission deadline exceeded'
    return None


def _build_outcome(ctx: Any, success: bool, status_code: str) -> CommandOutcome:
    outcome_fn = getattr(ctx, 'outcome', None)
    out = deepcopy(outcome_fn(success)) if callable(outcome_fn) else CommandOutcome(
        error_code='NONE' if success else (status_code or 'OBSERVATION_GAP')
    )
    if not success:
        if status_code == 'INVALID_INPUT':
            out.error_code = 'INVALID_INPUT'
        elif out.error_code in ('', 'NONE', 'OBSERVATION_GAP') and status_code:
            out.error_code = str(status_code)[:64]
    return out


def _fits_json_bound(text: str) -> bool:
    return len(text) <= MAX_STEPS_JSON_BYTES and len(text.encode('utf-8')) <= MAX_STEPS_JSON_BYTES


def _encode_step_results(step_results: list[dict[str, Any]]) -> str:
    bounded_steps = list(step_results[:MAX_MISSION_STEPS])
    text = json.dumps(bounded_steps, allow_nan=False, separators=(',', ':'))
    if _fits_json_bound(text):
        return text
    compact = [
        {k: v for k, v in item.items() if k != 'objects'}
        for item in bounded_steps
    ]
    text = json.dumps(compact, allow_nan=False, separators=(',', ':'))
    if _fits_json_bound(text):
        return text
    trimmed = [
        {k: v for k, v in item.items() if k not in ('objects', 'detail', 'room', 'target')}
        for item in bounded_steps
    ]
    text = json.dumps(trimmed, allow_nan=False, separators=(',', ':'))
    if _fits_json_bound(text):
        return text
    for code_limit in (64, 32, 16, 8, 0):
        minimal = [
            {
                'index': int(item['index']) if _is_num(item.get('index')) else i,
                'op': (
                    str(item.get('op', ''))[:16]
                    if code_limit > 0
                    else str(item.get('op', '')).encode('ascii', 'ignore').decode('ascii')[:16]
                ),
                'ok': bool(item.get('ok')),
                'status_code': str(item.get('status_code', ''))[:code_limit],
            }
            for i, item in enumerate(bounded_steps)
        ]
        text = json.dumps(minimal, allow_nan=False, separators=(',', ':'))
        if _fits_json_bound(text):
            return text
    return '[]'


def run_mission(ctx: Any, req: Any) -> tuple[bool, RunMission.Result]:
    """Validate and execute a bounded 1..8 step mission on a borrowed `GoalContext`."""
    directory = getattr(ctx, 'directory', None)
    evidence_uri = str(getattr(directory, 'name', '') or '')
    server = getattr(ctx, 'server', None)
    thresholds = getattr(server, 'thresholds', None)

    steps_json = getattr(req, 'steps_json', None)
    max_duration_s = getattr(req, 'max_duration_s', None)
    dry_run = getattr(req, 'dry_run', None)

    if not isinstance(steps_json, str) or not _num_in(max_duration_s, 1.0, 1800.0) or type(dry_run) is not bool:
        detail = 'Invalid mission goal fields (steps_json, max_duration_s in 1..1800, dry_run bool required)'
        return False, RunMission.Result(
            outcome=_build_outcome(ctx, False, 'INVALID_INPUT'),
            status_code='INVALID_INPUT',
            completed=False,
            partial_completion=False,
            dry_run_valid=False,
            steps_completed=0,
            steps_total=0,
            failed_step='',
            detail=detail[:128],
            step_results_json='[]',
            evidence_uri=evidence_uri[:256],
        )

    try:
        steps = parse_mission_steps(steps_json, allowed_colors=thresholds)
    except (ValueError, TypeError, OverflowError) as exc:
        detail = str(exc)[:128]
        return False, RunMission.Result(
            outcome=_build_outcome(ctx, False, 'INVALID_INPUT'),
            status_code='INVALID_INPUT',
            completed=False,
            partial_completion=False,
            dry_run_valid=False,
            steps_completed=0,
            steps_total=0,
            failed_step='',
            detail=detail,
            step_results_json='[]',
            evidence_uri=evidence_uri[:256],
        )

    total = len(steps)
    clock = getattr(ctx, 'clock', time.monotonic)
    started_at = clock()
    requested_deadline = started_at + float(max_duration_s)
    parent_deadline = getattr(server, 'deadline', None) if server is not None else None

    is_live_ctx = (
        server is not None
        or getattr(ctx, 'handle', None) is not None
        or getattr(ctx, 'commands', None) is not None
        or getattr(ctx, 'executor', None) is not None
    )
    if is_live_ctx and (
        server is None
        or not _is_num(parent_deadline)
        or not callable(getattr(ctx, 'hazard', None))
        or not callable(getattr(server, 'hazard', None))
        or not callable(getattr(ctx, 'is_cancelled', None))
    ):
        detail = 'Live GoalContext requires an active parent server deadline and hazard lifecycle'
        return False, RunMission.Result(
            outcome=_build_outcome(ctx, False, 'INVALID_INPUT'),
            status_code='INVALID_INPUT',
            completed=False,
            partial_completion=False,
            dry_run_valid=False,
            steps_completed=0,
            steps_total=total,
            failed_step='',
            detail=detail[:128],
            step_results_json='[]',
            evidence_uri=evidence_uri[:256],
        )

    deadline = (
        min(requested_deadline, float(parent_deadline))
        if _is_num(parent_deadline)
        else requested_deadline
    )

    if dry_run:
        interrupt = _classify_interrupt(ctx, deadline, clock)
        if interrupt is not None:
            status_code, detail = interrupt
            return False, RunMission.Result(
                outcome=_build_outcome(ctx, False, status_code),
                status_code=status_code[:64],
                completed=False,
                partial_completion=False,
                dry_run_valid=False,
                steps_completed=0,
                steps_total=total,
                failed_step='',
                detail=detail[:128],
                step_results_json='[]',
                evidence_uri=evidence_uri[:256],
            )
        status_fn = getattr(ctx, 'status', None)
        if callable(status_fn):
            try:
                st = status_fn() or {}
            except Exception as exc:
                return False, RunMission.Result(
                    outcome=_build_outcome(ctx, False, 'OBSERVATION_GAP'),
                    status_code='OBSERVATION_GAP',
                    completed=False,
                    partial_completion=False,
                    dry_run_valid=False,
                    steps_completed=0,
                    steps_total=total,
                    failed_step='',
                    detail=str(exc)[:128],
                    step_results_json='[]',
                    evidence_uri=evidence_uri[:256],
                )
            bat = st.get('battery', 100.0) if isinstance(st, dict) else 100.0
            if isinstance(st, dict) and (
                st.get('error_code', 0) != 0
                or not _is_num(bat)
                or float(bat) <= 15.0
                or bool(st.get('in_cleaning'))
                or bool(st.get('in_returning'))
            ):
                return False, RunMission.Result(
                    outcome=_build_outcome(ctx, False, 'ROBOT_NOT_READY'),
                    status_code='ROBOT_NOT_READY',
                    completed=False,
                    partial_completion=False,
                    dry_run_valid=False,
                    steps_completed=0,
                    steps_total=total,
                    failed_step='',
                    detail='Robot status not ready for mission execution',
                    step_results_json='[]',
                    evidence_uri=evidence_uri[:256],
                )
        dry_steps = [{'index': i, 'op': s['op'], 'ok': True, 'status_code': 'DRY_RUN_VALID'} for i, s in enumerate(steps)]
        return True, RunMission.Result(
            outcome=_build_outcome(ctx, True, 'DRY_RUN_VALID'),
            status_code='DRY_RUN_VALID',
            completed=False,
            partial_completion=False,
            dry_run_valid=True,
            steps_completed=0,
            steps_total=total,
            failed_step='',
            detail='',
            step_results_json=_encode_step_results(dry_steps),
            evidence_uri=evidence_uri[:256],
        )

    orig_feedback = getattr(ctx, 'feedback', None)
    orig_hazard = getattr(ctx, 'hazard', None)
    orig_cancelled = getattr(ctx, 'is_cancelled', None)
    executor = getattr(ctx, 'executor', None)
    orig_ex_feedback = getattr(executor, 'feedback', None) if executor is not None else None
    orig_ex_hazard = getattr(executor, 'hazard', None) if executor is not None else None
    active_step = [0, steps[0]['op']]

    def _emit_feedback(**fields):
        if not callable(orig_feedback):
            return
        merged = {
            'phase': 3,
            'current_step': int(active_step[0]),
            'current_op': str(active_step[1])[:64],
            'reason': 'NONE',
            **fields,
        }
        merged['current_step'] = int(active_step[0])
        merged['current_op'] = str(active_step[1])[:64]
        if 'reason' in merged:
            merged['reason'] = str(merged['reason'])[:64]
        action_cls = getattr(ctx, 'action_cls', RunMission)
        fb_cls = getattr(action_cls, 'Feedback', None)
        allowed = (
            frozenset(fb_cls.get_fields_and_field_types().keys())
            if hasattr(fb_cls, 'get_fields_and_field_types')
            else _FEEDBACK_FIELDS
        )
        filtered = {k: v for k, v in merged.items() if k in allowed}
        orig_feedback(**filtered)

    def _mission_hazard():
        h = orig_hazard() if callable(orig_hazard) else None
        if h:
            return h
        if callable(orig_cancelled) and orig_cancelled():
            return 'operator cancelled task'
        if clock() >= deadline:
            return 'task deadline'
        return None

    if callable(orig_feedback):
        ctx.feedback = _emit_feedback
        if executor is not None:
            executor.feedback = lambda event: _emit_feedback(phase=3, reason=str(event)[:64])
    if callable(orig_hazard):
        ctx.hazard = _mission_hazard
        if executor is not None:
            executor.hazard = _mission_hazard

    tightened_server_deadline = False
    if server is not None and (_is_num(parent_deadline) and deadline < float(parent_deadline)):
        server.deadline = deadline
        tightened_server_deadline = True

    step_results: list[dict[str, Any]] = []
    steps_completed = 0
    failed_step = ''
    status_code = 'COMPLETED'
    detail = ''

    try:
        for idx, step in enumerate(steps):
            op = step['op']
            active_step[0] = idx
            active_step[1] = op

            interrupt = _classify_interrupt(ctx, deadline, clock)
            if interrupt is not None:
                status_code, detail = interrupt
                failed_step = op
                step_results.append({
                    'index': idx,
                    'op': op,
                    'ok': False,
                    'status_code': status_code,
                    'detail': detail,
                })
                break

            _emit_feedback(phase=3, current_step=idx, current_op=op, reason='STEP_START')
            remaining_s = deadline - clock()
            if remaining_s <= 0.0 or (op in ('navigate_room', 'navigate') and remaining_s < 1.0):
                status_code, detail = 'TIMED_OUT', 'Mission deadline exceeded'
                failed_step = op
                step_results.append({
                    'index': idx,
                    'op': op,
                    'ok': False,
                    'status_code': status_code,
                    'detail': detail,
                })
                break

            try:
                if op == 'navigate_room':
                    nav_dur_s = min(600.0, float(step.get('max_duration_s', max_duration_s)), remaining_s)
                    res = ctx.navigate_room(step['room'], max_duration_s=nav_dur_s, dry_run=False)
                    ok = bool(isinstance(res, dict) and res.get('arrived'))
                    code = str((res or {}).get('status_code') or ('ARRIVED' if ok else 'NAVIGATION_FAILED'))
                    step_detail = str((res or {}).get('detail') or ('' if ok else code))[:128]
                    pos = res.get('final_position_m') if isinstance(res, dict) else None
                    step_results.append({
                        'index': idx,
                        'op': op,
                        'room': step['room'],
                        'ok': ok,
                        'status_code': code,
                        'segment_id': res.get('segment_id') if isinstance(res, dict) else None,
                        'final_position_m': (
                            [round(float(v), 4) for v in pos[:2]]
                            if isinstance(pos, (list, tuple)) and len(pos) >= 2 and all(_is_num(v) for v in pos[:2])
                            else None
                        ),
                    })

                elif op == 'navigate':
                    nav_dur_s = min(600.0, float(step.get('max_duration_s', max_duration_s)), remaining_s)
                    res = ctx.navigate(
                        step['target_x_m'],
                        step['target_y_m'],
                        max_duration_s=nav_dur_s,
                        dry_run=False,
                    )
                    ok = bool(isinstance(res, dict) and res.get('arrived'))
                    code = str((res or {}).get('status_code') or ('ARRIVED' if ok else 'NAVIGATION_FAILED'))
                    step_detail = str((res or {}).get('detail') or ('' if ok else code))[:128]
                    pos = res.get('final_position_m') if isinstance(res, dict) else None
                    err_m = res.get('position_error_m') if isinstance(res, dict) else None
                    step_results.append({
                        'index': idx,
                        'op': op,
                        'target_x_m': round(float(step['target_x_m']), 4),
                        'target_y_m': round(float(step['target_y_m']), 4),
                        'ok': ok,
                        'status_code': code,
                        'final_position_m': (
                            [round(float(v), 4) for v in pos[:2]]
                            if isinstance(pos, (list, tuple)) and len(pos) >= 2 and all(_is_num(v) for v in pos[:2])
                            else None
                        ),
                        'position_error_m': round(float(err_m), 4) if _is_num(err_m) else None,
                    })

                elif op == 'observe':
                    camera = step['camera']
                    color = step['color']
                    trace = getattr(ctx, 'trace', None)
                    before_len = len(trace) if isinstance(trace, list) else 0
                    raw_objs = ctx.objects(camera, color)
                    if isinstance(trace, list) and len(trace) == before_len:
                        trace.append({'primitive': 'objects', 'camera': camera, 'color': color})
                    bounded = _bounded_objects(raw_objs)
                    visible = len(bounded) > 0
                    ok = visible or not step['require_visible']
                    code = 'OBSERVED' if visible else 'EMPTY_OBSERVATION'
                    step_detail = '' if ok else f'No matching object observed on {camera} camera'
                    step_results.append({
                        'index': idx,
                        'op': op,
                        'camera': camera,
                        'color': color,
                        'ok': ok,
                        'status_code': code,
                        'count': len(bounded),
                        'objects': bounded,
                    })

                elif op == 'pick':
                    res = ctx.pick(step['color'], dry_run=False)
                    ok = bool(isinstance(res, dict) and res.get('verified'))
                    code = 'PICKED' if ok else 'PICK_UNVERIFIED'
                    step_detail = '' if ok else f"Pickup verification failed for color {step['color']}"
                    raw_target = res.get('target') if isinstance(res, dict) else None
                    t_pos = raw_target.get('position') if isinstance(raw_target, dict) else None
                    target_summary = (
                        {
                            'color': str(raw_target.get('color', step['color']))[:32],
                            'position': [round(float(v), 4) for v in t_pos[:3]],
                        }
                        if isinstance(raw_target, dict)
                        and isinstance(t_pos, (list, tuple))
                        and len(t_pos) >= 2
                        and all(_is_num(v) for v in t_pos[:3])
                        else None
                    )
                    step_results.append({
                        'index': idx,
                        'op': op,
                        'color': step['color'],
                        'ok': ok,
                        'status_code': code,
                        'verified': ok,
                        'target': target_summary,
                    })

            except BaseException as exc:
                if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                    raise
                interrupt = _classify_interrupt(ctx, deadline, clock, exc)
                if interrupt is not None:
                    code, step_detail = interrupt
                elif type(exc).__name__ == 'UncertainMotion':
                    code, step_detail = 'UNCERTAIN_MOTION', str(exc)[:128]
                elif isinstance(exc, ValueError):
                    code, step_detail = 'INVALID_INPUT', str(exc)[:128]
                elif isinstance(exc, TimeoutError):
                    code, step_detail = 'TIMED_OUT', str(exc)[:128]
                else:
                    code, step_detail = 'OBSERVATION_GAP', str(exc)[:128]
                ok = False
                step_results.append({
                    'index': idx,
                    'op': op,
                    'ok': False,
                    'status_code': code,
                    'error': type(exc).__name__,
                    'detail': step_detail,
                })

            post_interrupt = _classify_interrupt(ctx, deadline, clock)
            if post_interrupt is not None and (ok or code in ('PICK_UNVERIFIED', 'NAVIGATION_FAILED', 'EMPTY_OBSERVATION')):
                ok = False
                code, step_detail = post_interrupt
                step_results[-1]['ok'] = False
                step_results[-1]['status_code'] = code
                step_results[-1]['detail'] = step_detail

            if not ok:
                failed_step = op
                status_code = code
                detail = step_detail
                log_fn = getattr(ctx, 'log', None)
                if callable(log_fn):
                    log_fn('mission_step_failed', index=idx, op=op, status_code=code, detail=step_detail)
                break

            steps_completed += 1

    finally:
        if callable(orig_feedback):
            ctx.feedback = orig_feedback
        if callable(orig_hazard):
            ctx.hazard = orig_hazard
        if executor is not None and orig_ex_feedback is not None:
            executor.feedback = orig_ex_feedback
        if executor is not None and orig_ex_hazard is not None:
            executor.hazard = orig_ex_hazard
        if tightened_server_deadline and server is not None:
            server.deadline = parent_deadline

    completed = steps_completed == total and failed_step == ''
    partial_completion = not completed and steps_completed > 0
    if completed:
        status_code = 'COMPLETED'
        detail = ''

    outcome = _build_outcome(ctx, completed, status_code)
    result = RunMission.Result(
        outcome=outcome,
        status_code=str(status_code)[:64],
        completed=completed,
        partial_completion=partial_completion,
        dry_run_valid=False,
        steps_completed=int(steps_completed),
        steps_total=int(total),
        failed_step=str(failed_step)[:64],
        detail=str(detail)[:128],
        step_results_json=_encode_step_results(step_results),
        evidence_uri=evidence_uri[:256],
    )
    return completed, result
