"""ROS 2 command action servers and RPC services for z70_driver."""
from copy import deepcopy
import json
import math
import queue
import re
import threading
import time
import uuid

import requests
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from unique_identifier_msgs.msg import UUID
from z70_interfaces.action import BasePulse, ArmStep, HandStep, ArmOperation
from z70_interfaces.msg import CommandOutcome, ArmEstimate
from z70_interfaces.srv import Stop, SetArmAnchor, RawRpc, TaskSession, SetControlSession
from std_msgs.msg import Bool
from .dispatcher import Dispatcher, ZERO


def outcome_ok(outcome):
    return outcome.dispatch == 1 and outcome.response == 1 and outcome.error_code == 'NONE'


def action_plan(kind, goal):
    """Build the vendor RPC sequence and deadline budget for an action goal."""
    if kind is BasePulse:
        duration = goal.duration.sec + goal.duration.nanosec / 1e9
        if (not all(math.isfinite(x) for x in (goal.linear_mps, goal.angular_radps, duration))
            or not 0 < duration <= .8 or abs(goal.linear_mps) > .1
            or abs(goal.angular_radps) > .5 or (goal.linear_mps and goal.angular_radps)
            or not 0 <= goal.duration.nanosec < 10**9):
            raise ValueError('INVALID_INPUT')
        return [('app_rc_start', []), ('app_rc_move', [dict(velocity=goal.linear_mps,
            omega=goal.angular_radps, duration=max(1, round(duration*1000)), seqnum=1)])], 15.
    if kind is ArmStep:
        if goal.joint not in (3, 4, 5, 34, 43) or goal.direction not in (-1, 1):
            raise ValueError('INVALID_INPUT')
        pos, neg = (1, 2) if goal.direction == 1 else (2, 1)
        if goal.joint == 34:
            m = [pos, pos, 0]
        elif goal.joint == 43:
            m = [pos, neg, 0]
        else:
            m = [0, 0, 0]; m[goal.joint-3] = pos
        return [('app_arm_move', {'m': m})], 35.
    if kind is HandStep:
        if goal.direction not in (1, 2, 3, 4):
            raise ValueError('INVALID_INPUT')
        return [('app_arm_direction_move', {'direction': goal.direction})], 35.
    if kind is ArmOperation:
        mapping = {1: ('app_arm_out_compartment', {}), 2: ('app_arm_in_compartment', {}),
            3: ('app_open_gripper', {}), 4: ('app_grip', {}), 5: ('app_put_down', {'flag': 0}),
            6: ('app_pick_up_enter_exit', {'flag': 1}), 7: ('app_pick_up_enter_exit', {'flag': 2}),
            8: ('app_pick_up_enter_exit', {'flag': 0}), 9: ('app_arm_recover', {})}
        if goal.operation not in mapping:
            raise ValueError('INVALID_INPUT')
        return [mapping[goal.operation]], 60.
    raise ValueError('INVALID_INPUT')


class CommandSurfaces:
    def init_commands(self):
        self.estimator_lock = threading.RLock()
        self.task_cancel_pub = self.create_publisher(UUID, '/z70/tasks/cancel', 10)
        self.dispatcher = Dispatcher(self.priority_stop,
            lambda token: self.task_cancel_pub.publish(UUID(uuid=list(token))),
            stop_ok=lambda outcomes: bool(outcomes) and all(outcome_ok(o) for o in outcomes))
        from .driver import LATCHED
        self.session_lock = threading.Lock()
        self.session_pub = self.create_publisher(Bool, '/z70/control/enabled', LATCHED)
        self.session_pub.publish(Bool(data=True))
        self.command_group = ReentrantCallbackGroup()
        for kind, name, callback in [(Stop, '/z70/stop', self.stop_service),
                (SetControlSession, '/z70/control/session', self.control_session),
                (SetArmAnchor, '/z70/arm/anchor', self.anchor_service),
                (RawRpc, '/z70/driver/raw_rpc', self.raw_service),
                (TaskSession, '/z70/tasks/session', self.task_service)]:
            self.create_service(kind, name, callback, callback_group=self.command_group)
        self.action_tickets = {}
        self.reserved_goals = {}
        self.action_servers = []
        self.goto_active = False
        self.rc_sequence = 0
        self.command_transport = 0 # LOCAL=0, CLOUD=1; stop uses the latest actuator path
        for kind, name in [(BasePulse, '/z70/base/pulse'), (ArmStep, '/z70/arm/step'),
                           (HandStep, '/z70/arm/hand_step'), (ArmOperation, '/z70/arm/operation')]:
            self.action_servers.append(ActionServer(self, kind, name,
                execute_callback=lambda handle, k=kind: self.execute_command(k, handle),
                goal_callback=lambda goal, k=kind: self.accept_command(k, goal),
                cancel_callback=self.cancel_command, callback_group=self.command_group))

    def local_outcome(self, code='NONE', unknown=False, command_id=None):
        # No backend request exists for local rejection. Zero attempt identifies
        # absence of a wire attempt, not a fabricated backend event.
        from .driver import uid, host_time
        return CommandOutcome(command_id=uid(command_id or uuid.uuid4()), requested_at=host_time(time.time()),
            finished_at=host_time(time.time()), error_code=code,
            dispatch=2 if unknown else 0, response=3 if unknown else 0)

    def backend_outcome(self, row):
        from .driver import uid, host_ms
        o = CommandOutcome(command_id=uid(row['commandId']), attempt_id=uid(row['attemptId']),
            attempt_index=int(row['attemptIndex']), requested_at=host_ms(row['startedAt']),
            finished_at=host_ms(row['finishedAt']),
            dispatch={'not_sent': 0, 'sent': 1, 'unknown': 2}[row['dispatch']],
            response={'none': 0, 'received': 1, 'vendor_error': 2, 'transport_error': 3}[row['response']])
        reasons = {'timeout': 'TIMEOUT', 'capacity': 'CAPACITY', 'invalid-input': 'INVALID_INPUT',
                   'unavailable': 'UNAVAILABLE', 'transport': 'TRANSPORT','cancelled':'CANCELLED'}
        o.error_code = ('NONE' if o.response == 1 else 'VENDOR_REJECTED' if o.response == 2
                        else reasons.get(row.get('transportReason'), 'TRANSPORT'))
        if isinstance(row.get('vendorCode'), int):
            o.vendor_code_known = True; o.vendor_code = row['vendorCode']
        return o

    def driver_rpc(self, method, params, timeout_ms=30000, ticket=None, transport=0):
        """One HTTP invocation, one estimator observation, no driver retry/events."""
        if ticket is not None and not self.dispatcher.valid(ticket):
            return self.local_outcome('CANCELLED'), None
        if ticket is not None and not method.startswith(('get_','app_get_')):
            self.command_transport=transport
        if method == 'app_goto_target': self.goto_active = True
        command_id=uuid.uuid4()
        from .driver import host_time
        requested_at=host_time(time.time())
        try:
            row = self.request('POST', '/api/driver/rpc',
                json=dict(method=method, params=params, timeoutMs=int(timeout_ms),transport='cloud' if transport else 'local',commandId=str(command_id)),
                timeout=(.2, max(.001, timeout_ms/1000-.2)), allow_error=True).json()
            outcome = self.backend_outcome(row)
            value = row.get('result')
        except requests.Timeout:
            outcome, value = self.local_outcome('TIMEOUT', unknown=True,command_id=command_id), None
        except (requests.RequestException, ValueError, TypeError, KeyError):
            outcome, value = self.local_outcome('TRANSPORT', unknown=True,command_id=command_id), None
        if not any(outcome.attempt_id.uuid): outcome.requested_at=requested_at
        observed = ('not_sent' if outcome.dispatch == 0 else
                    'received' if outcome.response == 1 else
                    'vendor_error' if outcome.response == 2 else 'unknown')
        # A revoked late reply cannot recover an estimate invalidated by stop.
        if ticket is not None and ticket.cancelled.is_set() and observed != 'not_sent':
            observed = 'unknown'
        with self.estimator_lock:
            if self.estimator.observe(method, params, observed,
                    command_id=str(uuid.UUID(bytes=bytes(outcome.command_id.uuid)))):
                self.estimate_source = 1
            if method == 'get_status' and outcome.response == 1:
                self.estimator.observe_status(value, outcome.requested_at.sec*1000 + outcome.requested_at.nanosec//1_000_000)
            self.publish_estimate(self.estimator.snapshot())
        return outcome, value

    def rpc_wait(self, method, params, budget, ticket):
        """Wait for RPC OR cancellation; an HTTP reader never owns action progress."""
        results = queue.Queue(maxsize=1)
        self.start_worker('rpc', lambda: results.put(self.driver_rpc(method, params, budget, ticket)))
        deadline = time.monotonic()+budget/1000
        while time.monotonic() < deadline and not ticket.cancelled.is_set() and not self.stop_event.is_set():
            try: return results.get(timeout=.02)[0]
            except queue.Empty: pass
        timed_out = not ticket.cancelled.is_set()
        if timed_out:
            self.dispatcher.cancel(ticket, lambda fn: self.start_worker('deadline_stop', fn))
        return self.local_outcome('DEADLINE' if timed_out else 'CANCELLED', unknown=True)

    def priority_stop(self, scope):
        methods = {1: ['app_rc_stop', 'app_rc_end'], 2: ['app_stop_grasp'],
                   3: ['app_rc_stop', 'app_rc_end', 'app_stop_grasp']}[scope]
        if scope in (1, 3) and getattr(self, 'goto_active', False):
            methods = ['stop_goto_target'] + methods
        if scope in (2, 3):
            with self.estimator_lock:
                if self.estimator.data['state'] != 'IN_ROBOT':
                    self.estimator.invalidate('STOP_UNVERIFIED')
                    self.publish_estimate(self.estimator.snapshot())
        outcomes = []; deadline = time.monotonic()+15
        for method in methods:
            budget = max(.001, min(5., deadline-time.monotonic()))
            result = queue.Queue(maxsize=1)
            self.start_worker('stop_rpc', lambda m=method, b=budget, q=result:
                q.put(self.driver_rpc(m, [], max(1, int(b*1000)),transport=self.command_transport)))
            try: outcome = result.get(timeout=budget)
            except queue.Empty: outcome = (self.local_outcome('STOP_UNVERIFIED', unknown=True), None)
            o = outcome[0]
            if not outcome_ok(o): o.error_code = 'STOP_UNVERIFIED'
            outcomes.append(o)
            if method == 'stop_goto_target' and outcome_ok(o): self.goto_active = False
        if any(not outcome_ok(o) for o in outcomes):
            self.diagnostic('stop', 'STOP_UNVERIFIED', 2)
        return outcomes

    def stop_service(self, request, response):
        if request.scope not in (1, 2, 3):
            response.outcomes = [self.local_outcome('INVALID_INPUT')]
        else:
            response.outcomes = self.dispatcher.stop(request.scope)
        response.cessation_observed = False
        return response

    def control_session(self, request, response):
        # This lock serializes deliberate resume with cleanup, never motion STOP.
        with self.session_lock:
            if request.enabled:
                try:
                    result = self.request('POST', '/api/driver/control/session', json={'enabled': True}, timeout=12).json()
                    if result.get('enabled') is not True: raise ValueError()
                except (requests.RequestException, ValueError):
                    response.outcomes = [self.local_outcome('SESSION_RESUME_FAILED')]
                else:
                    with self.dispatcher.lock: self.dispatcher.session_enabled = True
            else:
                # Suspend BEFORE releasing the preview. No camera lock/network wait.
                with self.dispatcher.lock: self.dispatcher.session_enabled = False
                self.acquisition_generation += 1
                self.session_pub.publish(Bool(data=False))
                motion = queue.Queue(maxsize=1)
                self.start_worker('session_stop', lambda: motion.put(self.dispatcher.stop(3)))
                try:
                    result = self.request('POST', '/api/driver/control/session', json={'enabled': False}, timeout=12).json()
                    response.streams_released = result.get('streams_released') is True
                except (requests.RequestException, ValueError):
                    response.streams_released = False
                response.outcomes = motion.get(timeout=15.5)
                # Established pickup/manual viewing exit; no stow or gripper motion.
                outcome, _ = self.driver_rpc('app_pick_up_enter_exit', {'flag': 0}, 5000,
                                             transport=self.command_transport)
                response.outcomes = list(response.outcomes) + [outcome]
                # Use the established owned-preview cleanup, not a made-up camera RPC.
                # Never stop the official app's/different client's preview.
                def preview_status():
                    o, value = self.driver_rpc('get_homesec_connect_status', [], 2000,
                                               transport=self.command_transport)
                    response.outcomes = list(response.outcomes) + [o]
                    if isinstance(value, list) and len(value) == 1: value = value[0]
                    return value if outcome_ok(o) and isinstance(value, dict) else {}
                preview = preview_status()
                if preview.get('status') == 1 and preview.get('client_id') == '676f32727463':
                    o, _ = self.driver_rpc('stop_camera_preview', {'client_id':'676f32727463'}, 2000,
                                          transport=self.command_transport)
                    response.outcomes = list(response.outcomes) + [o]
                    preview = preview_status()
                response.streams_released = (response.streams_released and
                    preview.get('status') == 0 and preview.get('client_id') == 'none')
                if not response.streams_released:
                    response.outcomes = list(response.outcomes) + [self.local_outcome('STREAM_RELEASE_UNVERIFIED', unknown=True)]
            response.enabled = self.dispatcher.session_enabled
            response.cessation_observed = False
            self.session_pub.publish(Bool(data=response.enabled))
            return response

    def task_service(self, request, response):
        token = bytes(request.task_id.uuid)
        if request.begin:
            accepted, token, reason = self.dispatcher.begin(token, request.takeover)
        else:
            accepted = self.dispatcher.end(token)
            reason = 'NONE' if accepted else 'BUSY'
        response.accepted = accepted; response.task_id = UUID(uuid=list(token)); response.reason = reason
        return response

    def anchor_service(self, request, response):
        if request.anchor not in (0, 1, 2):
            self.diagnostic('anchor', 'INVALID_INPUT')
        else:
            with self.estimator_lock:
                self.estimator.set_anchor({0:'unknown', 1:'stowed', 2:'deployed'}[request.anchor],
                    reason=request.observation_reason or 'OPERATOR_ANCHOR')
                self.estimate_source = 3
                self.publish_estimate(self.estimator.snapshot())
        with self.sample_lock:
            response.estimate = deepcopy(self.samples.get('arm', (ArmEstimate(), None))[0])
        return response

    def raw_service(self, request, response):
        try:
            if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,127}', request.method): raise ValueError()
            if len(request.params_json.encode('utf-8')) > 16384: raise ValueError()
            def invalid_constant(value): raise ValueError('INVALID_INPUT')
            params = json.loads(request.params_json or '[]', parse_constant=invalid_constant)
            if not 1 <= request.timeout_ms <= 120000: raise ValueError()
            if request.transport not in (0,1): raise ValueError()
        except (ValueError, TypeError):
            response.outcome = self.local_outcome('INVALID_INPUT'); return response
        # Native navigation stop must bypass the occupied/cancelled task lane.
        if request.method == 'stop_goto_target':
            if params != []:
                response.outcome = self.local_outcome('INVALID_INPUT'); return response
            response.outcome, _ = self.driver_rpc('stop_goto_target', [], request.timeout_ms, transport=request.transport)
            if outcome_ok(response.outcome): self.goto_active = False
            response.result_json = 'null'
            return response
        ticket = (self.dispatcher.takeover() if request.takeover else
                  self.dispatcher.reserve(bytes(request.task_id.uuid)))
        if ticket is None:
            response.outcome = self.local_outcome('SESSION_STOPPED' if not self.dispatcher.session_enabled else
                'STOP_UNVERIFIED' if self.dispatcher.blocked else 'BUSY'); return response
        try:
            response.outcome, value = self.driver_rpc(request.method, params, request.timeout_ms, ticket,request.transport)
            result_json = json.dumps(value, separators=(',', ':'))
            if len(result_json.encode()) <= 16384: response.result_json = result_json
            else: response.outcome.error_code = 'CAPACITY'
        finally:
            self.dispatcher.release(ticket)
        return response

    def accept_command(self, kind, goal):
        try: action_plan(kind, goal)
        except ValueError: return GoalResponse.REJECT
        ticket = self.dispatcher.reserve()
        if ticket is None: return GoalResponse.REJECT
        self.reserved_goals[id(goal)] = ticket
        return GoalResponse.ACCEPT

    def cancel_command(self, handle):
        ticket = self.action_tickets.get(bytes(handle.goal_id.uuid))
        if ticket is not None:
            self.dispatcher.cancel(ticket, lambda fn: self.start_worker('cancel_stop', fn))
        return CancelResponse.ACCEPT

    def execute_command(self, kind, handle):
        plan, maximum = action_plan(kind, handle.request)
        ticket = self.reserved_goals.pop(id(handle.request), None)
        if ticket is None:
            # rclpy retains the request in ServerGoalHandle; defensive fallback
            # uses the single scheduler reservation, never creates a new write.
            ticket = self.dispatcher.active
        result = kind.Result(); result.outcome = self.local_outcome('CANCELLED')
        if ticket is None:
            handle.abort(); return result
        key = bytes(handle.goal_id.uuid); self.action_tickets[key] = ticket
        deadline = time.monotonic()+maximum
        dispatch_deadline = deadline-(10. if kind is BasePulse else 15.)
        def feedback(phase, outcome=None):
            msg = kind.Feedback(phase=phase, reason='NONE')
            if outcome is not None: msg.latest_outcome = outcome
            handle.publish_feedback(msg)
        try:
            feedback(0)
            for method, params in plan:
                if handle.is_cancel_requested and not ticket.cancelled.is_set():
                    self.dispatcher.cancel(ticket, lambda fn: self.start_worker('cancel_stop', fn))
                if not self.dispatcher.valid(ticket): break
                if method == 'app_rc_move':
                    self.rc_sequence = self.rc_sequence % 0x7fffffff + 1
                    params[0]['seqnum'] = self.rc_sequence
                feedback(2)
                remaining = dispatch_deadline-time.monotonic()
                if remaining <= 0:
                    result.outcome = self.local_outcome('DEADLINE', unknown=True)
                    self.dispatcher.cancel(ticket, lambda fn: self.start_worker('deadline_stop', fn)); break
                result.outcome = self.rpc_wait(method, params, int(min(30., remaining)*1000), ticket)
                feedback(3, result.outcome)
                if result.outcome.error_code in ('TIMEOUT', 'DEADLINE') and not ticket.cancelled.is_set():
                    self.dispatcher.cancel(ticket, lambda fn: self.start_worker('deadline_stop', fn))
                if not outcome_ok(result.outcome): break
            if kind is BasePulse and not ticket.cancelled.is_set():
                # Firmware duration is not completion evidence. Wait its bounded
                # duration then explicitly end RC through the priority path.
                duration = handle.request.duration.sec+handle.request.duration.nanosec/1e9
                ticket.cancelled.wait(duration)
                if not ticket.cancelled.is_set():
                    for method in ('app_rc_stop', 'app_rc_end'):
                        if not self.dispatcher.valid(ticket): break
                        stop_outcome = self.rpc_wait(method, [], min(5000, max(1, int((deadline-time.monotonic())*1000))), ticket)
                        if not outcome_ok(stop_outcome): result.outcome = stop_outcome
            if ticket.cancelled.is_set():
                feedback(4)
                ticket.stop_done.wait(15.1)
                if handle.is_cancel_requested or result.outcome.error_code not in ('TIMEOUT', 'DEADLINE'):
                    if ticket.stop_outcomes: result.outcome = ticket.stop_outcomes[-1]
                    else: result.outcome = self.local_outcome('STOP_UNVERIFIED', unknown=True)
                if handle.is_cancel_requested: handle.canceled()
                else: handle.abort()
            elif outcome_ok(result.outcome): handle.succeed()
            else: handle.abort()
            if kind is not BasePulse:
                with self.sample_lock:
                    result.estimate = deepcopy(self.samples.get('arm', (ArmEstimate(), None))[0])
            feedback(5, result.outcome)
            return result
        finally:
            self.action_tickets.pop(key, None)
            self.dispatcher.release(ticket)
