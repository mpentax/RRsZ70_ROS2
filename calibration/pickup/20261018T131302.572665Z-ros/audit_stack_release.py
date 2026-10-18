"""Offline control-flow reproduction; no ROS nodes, sockets or robot calls.

Run from the repository root:
  PYTHONPATH=src/z70_manipulation:src/z70_description ~/.pixi/bin/pixi run python calibration/pickup/20261018T131302.572665Z-ros/audit_stack_release.py

The injected inputs are deliberately synthetic, not hardware evidence. The
real Executor.stack implementation is exercised to show that opening failure
cannot prevent it reporting stack_verified=True. This is an audit, not a trial.
"""
import json
from pathlib import Path
import tempfile
from z70_manipulation.executor import Executor


def reproduce():
    with tempfile.TemporaryDirectory(prefix='stack-release-audit-') as directory:
        executor = Executor(None, None, directory, sleep=lambda _: None)
        calls = []
        moves = []
        executor.deployed = True
        executor.safety = lambda: None
        executor.target = lambda camera, color: {'position': [.411, -.012, 0.]}
        executor.align_front = lambda color: {'position': [.411, -.012, 0.]}
        executor.front_servo = lambda color, current, spot: ({}, current, spot)
        executor.snap = lambda name: None
        executor.move_arm = lambda target, start: moves.append(list(target))
        executor.end_rc = lambda: None
        executor.pulse = lambda **kwargs: None
        executor.stow = lambda: None
        executor.pregrip_jaw = [.411, -.012, .033]

        def command(method, params=None, **kwargs):
            calls.append({'method': method, 'params': params})
            if method == 'app_open_gripper':
                return {'dispatch': 'unknown', 'response': 'transport_error', 'transportReason': 'timeout'}
            return {'dispatch': 'sent', 'response': 'received'}

        executor.command = command
        # Assume successful pickups, but never supply successful release evidence.
        executor.grasp_and_lift = lambda **kwargs: {'verified': True}
        result = executor.stack('yellow', 'purple', 'blue')
        rows = [json.loads(line) for line in (Path(directory) / 'timeline.jsonl').read_text().splitlines()]
        placements = [r['state'] for r in rows if r['event'] == 'tier_place_pose']
        return {'synthetic_inputs': True, 'hardware_test': False,
                'result': result | {'log_dir': '<temporary offline audit>'},
                'all_open_commands_timed_out': True,
                'release_observation_supplied': False,
                'actual_move_arm_targets': moves,
                'logged_placement_states': placements,
                'bug_reproduced': result['stack_verified'] is True}


if __name__ == '__main__':
    evidence = reproduce()
    print(json.dumps(evidence, indent=2))
    assert evidence['bug_reproduced'], 'Control flow changed; re-audit release verification.'
