"""Unit and DDS tests for Pin-and-Go navigation."""
from types import SimpleNamespace as NS

import numpy as np
import pytest


def room(pose=(1., 1., 0.)):
    from z70_mapping.decoder import VendorMap
    pixels = np.full((80, 80), 15, np.uint8)
    pixels[[0, -1], :] = 1; pixels[:, [0, -1]] = 1
    occupancy = np.where(pixels == 1, 100, 0).astype(np.int8)
    return VendorMap(80, 80, (0., 0.), pixels, occupancy, pose, None, {1: {}}, {}, {})


class Observations:
    def __init__(self, poses=None):
        self.poses = iter(poses or [(1., 1., 0.)] * 100)
        self.sequence = 0
        self.states = iter([16, 3] * 100)
        self.arm_state = 'IN_ROBOT'
        self.status = dict(common_status=0, error_code=0, battery=80, in_cleaning=False, in_returning=False)
        self.verified = True
        self.stale = False

    def get(self, path):
        if path == '/api/arm/estimate':
            return {'state': self.arm_state}
        return {'telemetry': {'status': {'value': dict(self.status, state=next(self.states, 3))}}}

    def navigation_snapshot(self, refresh=False, max_age_s=5.):
        self.sequence += 1
        return NS(map=room(next(self.poses)), labels={}, sequence=self.sequence,
                  active_floor_verified=self.verified, current_pose_verified=self.verified,
                  cache_age_s=10. if self.stale else 0., evidence=None)


class Commands:
    latest_outcome = None

    def __init__(self):
        self.calls = []
        self.reply = dict(dispatch='sent', response='received')
        self.stops = 0

    def rpc(self, method, params, timeout_ms=5000):
        self.calls.append((method, params))
        return self.reply if method == 'app_goto_target' else dict(dispatch='sent', response='received')

    def stop(self):
        self.stops += 1


def run(obs=None, commands=None, dry_run=True, **kwargs):
    from z70_manipulation.navigation import PinAndGoNavigator
    obs = obs or Observations()
    commands = commands or Commands()
    result = PinAndGoNavigator(obs, commands, sleep=lambda _: None, **kwargs).execute(
        NS(target_x_m=2., target_y_m=1., max_duration_s=2., dry_run=dry_run))
    return result, commands


def test_dry_run_valid_does_not_dispatch():
    result, commands = run()
    assert result['status_code'] == 'DRY_RUN_VALID' and result['dry_run_valid']
    assert not result['arrived'] and result['final_position_m'] == [1., 1.]
    assert result['position_error_m'] == 1.
    assert commands.calls == []


@pytest.mark.parametrize('attribute,value,code', [
    ('arm_state', 'UNKNOWN', 'ARM_NOT_STOWED'),
    ('verified', False, 'UNLOCALIZED_MAP'),
    ('stale', True, 'UNLOCALIZED_MAP'),
])
def test_fail_closed_preconditions(attribute, value, code):
    obs = Observations()
    setattr(obs, attribute, value)
    result, commands = run(obs)
    assert result['status_code'] == code and commands.calls == []


@pytest.mark.parametrize('key,value', [
    ('common_status', 64), ('error_code', 1), ('battery', 15),
    ('in_cleaning', True), ('in_returning', True),
])
def test_state_preconditions(key, value):
    obs = Observations()
    obs.status[key] = value
    result, commands = run(obs)
    assert result['status_code'] == 'ARM_NOT_STOWED' and not commands.calls


def test_unknown_dispatch_is_stopped_once_never_replayed():
    commands = Commands()
    commands.reply = dict(dispatch='unknown', response='transport_error')
    result, _ = run(commands=commands, dry_run=False)
    assert result['status_code'] == 'UNKNOWN_DISPATCH'
    assert commands.calls == [('app_goto_target', [2000, 1000]), ('stop_goto_target', [])]
    assert commands.stops == 1


def test_vendor_rejection_not_arrival():
    commands = Commands()
    commands.reply = dict(dispatch='sent', response='vendor_error')
    result, _ = run(commands=commands, dry_run=False)
    assert result['status_code'] == 'VENDOR_REJECTED' and not result['arrived']
    assert commands.calls == [('app_goto_target', [2000, 1000])]


@pytest.mark.parametrize('final,expected', [(1., 'STOPPED_SHORT'), (2., 'ARRIVED')])
def test_state_transition_requires_post_stop_position(final, expected):
    obs = Observations([(1., 1., 0.), (1.3, 1., 0.), (final, 1., 0.), (final, 1., 0.)])
    obs.states = iter([3, 3, 16, 3])
    result, commands = run(obs, dry_run=False)
    assert result['status_code'] == expected
    assert result['arrived'] == (expected == 'ARRIVED')
    assert result['final_position_m'] == [final, 1.]
    assert commands.calls == [('app_goto_target', [2000, 1000])]


def test_live_boundary_exit_stops_native_and_rc():
    obs = Observations([(1., 1., 0.), (.1, 1., 0.)])
    obs.states = iter([3, 3, 16])
    result, commands = run(obs, dry_run=False)
    assert result['status_code'] == 'BOUNDARY_ABORT'
    assert commands.calls[-1] == ('stop_goto_target', []) and commands.stops == 1


def test_cancel_before_dispatch_never_moves():
    result, commands = run(dry_run=False, cancelled=lambda: True)
    assert result['status_code'] == 'CANCELLED' and commands.calls == []


def test_live_cancellation_stops():
    checks = iter([False, True])
    result, commands = run(dry_run=False, cancelled=lambda: next(checks))
    assert result['status_code'] == 'CANCELLED' and commands.calls[-1] == ('stop_goto_target', [])


def test_timeout_stops_even_if_always_goto():
    obs = Observations()
    obs.states = iter([3, 3] + [16] * 100)
    ticks = iter([0., 0., .1, 1., 3.])
    result, commands = run(obs, dry_run=False, clock=lambda: next(ticks))
    assert result['status_code'] == 'TIMED_OUT' and commands.stops == 1


def test_changed_map_index_is_not_arrival():
    obs = Observations([(1., 1., 0.), (2., 1., 0.), (2., 1., 0.), (2., 1., 0.)])
    obs.states = iter([3, 3, 16, 3])
    original = obs.navigation_snapshot

    def snapshot(**kwargs):
        s = original(**kwargs)
        s.map_index = 1 if s.sequence < 4 else 2
        return s

    obs.navigation_snapshot = snapshot
    result, commands = run(obs, dry_run=False)
    assert result['status_code'] == 'BOUNDARY_ABORT' and commands.stops == 1


def test_real_dds_pin_and_go_dry_run_and_unverified_map():
    """Verify PinAndGo action over DDS with PickServer and latched ROS observations."""
    import hashlib
    import struct
    import threading
    import time
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from z70_interfaces.action import PinAndGo
    from z70_interfaces.msg import ArmEstimate, RobotState, MapSnapshot
    from z70_interfaces.srv import TaskSession, RawRpc, Stop, RefreshMap
    from unique_identifier_msgs.msg import UUID
    from nav_msgs.msg import OccupancyGrid
    from z70_manipulation.node import PickServer
    from z70_manipulation.observations import LATCHED

    def raw_map(x=1000):
        m = room()
        pixels = m.pixels.tobytes()
        image = struct.pack('<HHIiiII', 2, 24, len(pixels), 0, 0, 80, 80) + pixels
        robot = struct.pack('<HHIiii', 8, 8, 12, x, 1000, 0)
        body = image + robot + struct.pack('<HHI', 1024, 8, 20)
        raw = b'rr' + struct.pack('<H', 20) + struct.pack('<I', len(body) + 20) + bytes(12) + body
        return raw + hashlib.sha1(raw).digest()

    def wait(predicate):
        until = time.monotonic() + 8.
        while time.monotonic() < until:
            if predicate():
                return
            time.sleep(.02)
        assert predicate(), 'offline DDS deadline'

    rclpy.init(domain_id=96)
    server = peer = action = executor = thread = None
    try:
        server = PickServer()
        peer = rclpy.create_node('pin_and_go_offline_fixture')
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(server); executor.add_node(peer)
        pubs = {cls: peer.create_publisher(cls, topic, LATCHED) for cls, topic in (
            (ArmEstimate, '/z70/arm_estimate'), (RobotState, '/z70/state'),
            (MapSnapshot, '/z70/driver/map_snapshot'), (OccupancyGrid, '/z70/map/safe_grid'))}
        state = RobotState(connected=True, robot_state=3, robot_state_known=True, error_code_known=True,
                           common_status_known=True, battery_known=True, battery_fraction=.8,
                           cleaning_known=True, returning_known=True)
        state.evidence.valid = True; state.evidence.received_at = peer.get_clock().now().to_msg()
        arm = ArmEstimate(); arm.evidence.valid = True; arm.validity.value = 3; arm.validity.reason = 'COMMAND_DERIVED_STOW'
        snapshot = MapSnapshot(raw=list(raw_map()), map_index=1, map_sequence=1)
        snapshot.evidence.valid = True; snapshot.evidence.sequence = 1
        snapshot.evidence.received_at = peer.get_clock().now().to_msg()
        pubs[RobotState].publish(state); pubs[ArmEstimate].publish(arm); pubs[MapSnapshot].publish(snapshot)
        pubs[OccupancyGrid].publish(OccupancyGrid())
        calls = []; native = {'active': False, 'refreshes': 0}

        def session(request, response):
            response.accepted = True; response.task_id = UUID(uuid=[1] * 16); return response

        def rpc(request, response):
            calls.append((request.method, request.params_json))
            response.outcome.dispatch = 1; response.outcome.response = 1; response.outcome.error_code = 'NONE'
            response.result_json = '["ok"]'
            if request.method == 'app_goto_target':
                native['active'] = True; native['refreshes'] = 0; state.robot_state = 16
            return response

        def refresh(request, response):
            if native['active']:
                native['refreshes'] += 1
                x = 1300 if native['refreshes'] == 1 else 2000
                state.robot_state = 16 if native['refreshes'] == 1 else 3
            else:
                x = 1000
            state.evidence.received_at = peer.get_clock().now().to_msg(); pubs[RobotState].publish(state)
            snapshot.raw = list(raw_map(x)); snapshot.evidence.sequence += 1
            snapshot.evidence.received_at = peer.get_clock().now().to_msg(); pubs[MapSnapshot].publish(snapshot)
            response.outcome.error_code = 'NONE'; response.outcome.dispatch = 1; response.outcome.response = 1
            response.snapshot = snapshot.evidence; return response

        for srv_cls, srv_name, cb in [
            (TaskSession, '/z70/tasks/session', session),
            (RawRpc, '/z70/driver/raw_rpc', rpc),
            (Stop, '/z70/stop', lambda req, res: res),
            (RefreshMap, '/z70/map/refresh', refresh),
        ]:
            peer.create_service(srv_cls, srv_name, cb)
        action = ActionClient(peer, PinAndGo, '/z70/nav/pin_and_go')
        thread = threading.Thread(target=executor.spin); thread.start()
        assert action.wait_for_server(timeout_sec=5.)
        wait(lambda: server.observations.map_snapshot is not None and server.observations.arm is not None
             and server.observations.state is not None and server.observations.safe_grid is not None)

        def goal(dry_run=True):
            future = action.send_goal_async(PinAndGo.Goal(target_x_m=2., target_y_m=1., max_duration_s=5., dry_run=dry_run))
            wait(future.done); handle = future.result(); assert handle.accepted
            future = handle.get_result_async(); wait(future.done); return future.result()

        result = goal()
        assert result.status == 6 and result.result.status_code == 'UNLOCALIZED_MAP'
        snapshot.active_floor_verified = True; snapshot.current_pose_verified = True; snapshot.evidence.sequence = 2
        snapshot.evidence.received_at = peer.get_clock().now().to_msg(); pubs[MapSnapshot].publish(snapshot)
        wait(lambda: server.observations.map_snapshot.evidence.sequence == 2)
        result = goal()
        assert result.status == 4 and result.result.status_code == 'DRY_RUN_VALID'
        assert result.result.dry_run_valid and not result.result.arrived
        assert list(result.result.final_position_m) == [1., 1.]
        assert not server.lock.locked()
        assert calls == []
        result = goal(dry_run=False)
        assert result.status == 4 and result.result.status_code == 'ARRIVED'
        assert result.result.arrived and list(result.result.final_position_m) == [2., 1.]
        assert calls == [('app_goto_target', '[2000, 1000]')]
        assert not server.lock.locked()
    finally:
        if action is not None:
            action.destroy()
        if executor is not None:
            executor.shutdown(timeout_sec=5.)
        if thread is not None:
            thread.join(5.)
        if server is not None:
            for a in getattr(server, 'action_servers', []):
                a.destroy()
            server.observations.close(); server.destroy_node()
        if peer is not None:
            peer.destroy_node()
        rclpy.try_shutdown()
