"""CLI entry points for `pixi run pick` and `pixi run stack`."""
import argparse
import json
import os
from pathlib import Path
import cv2
import rclpy
from rclpy.action import ActionClient
from z70_interfaces.action import Pick, Stack
from . import perception
from .executor import Client, Executor, log_directory

COLORS = ['yellow', 'purple', 'blue', 'green', 'orange']


def _common_parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--origin', default='http://127.0.0.1:57010')
    p.add_argument('--repo', type=Path, default=Path(os.environ.get('Z70_REPO', Path.cwd())))
    p.add_argument('--token-file', type=Path)
    p.add_argument('--log-dir', type=Path)
    return p


def _run_dry(args, execute_fn, *, filter_by_color=False) -> dict:
    directory = args.log_dir or log_directory(args.repo)
    client = Client(args.origin, args.token_file or args.repo / '.local/service-token', read_only=True)
    cached_objects = None

    def observe(camera, color):
        nonlocal cached_objects
        if not filter_by_color or cached_objects is None:
            image, tag = client.frame()
            if tag != 'front':
                raise LookupError('dry-run requires an already-selected front camera; no selection was sent')
            directory.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(directory / 'front.jpg'), image)
            cached_objects = perception.front_objects(image)
        return [b for b in cached_objects if b.get('color') == color] if filter_by_color else cached_objects

    executor = Executor(client, observe, directory)
    try:
        report = execute_fn(executor)
        (directory / 'dry-run.json').write_text(json.dumps(report, indent=2) + '\n')
        return report
    finally:
        client.close()


def _run_action(p, args, node_name, action_cls, topic, goal, reject_msg, format_result) -> dict:
    if args.origin != p.get_default('origin') or args.token_file or args.log_dir:
        p.error('connection/log options configure --dry-run only; configure the ROS pick server for live execution')
    rclpy.init()
    node = rclpy.create_node(node_name)
    action = ActionClient(node, action_cls, topic)
    handle = None
    try:
        if not action.wait_for_server(timeout_sec=10):
            raise RuntimeError(f'{topic} server unavailable; launch bringup manipulation:=true')
        future = action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(reject_msg)
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future)
        return format_result(future.result().result)
    except KeyboardInterrupt:
        if handle:
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        raise
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


def run(argv=None):
    p = _common_parser('pixi run pick COLOR [--dry-run]')
    p.add_argument('color', choices=COLORS)
    args = p.parse_args(argv)
    if args.dry_run:
        return _run_dry(args, lambda ex: ex.pick(args.color, dry_run=True))
    return _run_action(
        p, args, 'z70_pick_client', Pick, '/z70/pick',
        Pick.Goal(color=args.color, dry_run=False),
        'pickup goal rejected (another pickup active or unknown colour)',
        lambda r: dict(
            verified=r.verified_hold, dry_run_valid=r.dry_run_valid,
            success=r.verified_hold or r.dry_run_valid, evidence_uri=r.evidence_uri,
            error_code=r.outcome.error_code,
        ),
    )


def _cli_main(fn, dry_key: str):
    try:
        report = fn()
        print(json.dumps(report, indent=2))
        if (report.get('dry_run') and not report.get(dry_key)) or report.get('success') is False:
            raise SystemExit(1)
    except (RuntimeError, LookupError, ValueError) as e:
        raise SystemExit(str(e))


def main():
    _cli_main(run, 'target')


def run_stack(argv=None):
    p = _common_parser('pixi run stack BASE_COLOR MIDDLE_COLOR TOP_COLOR [--dry-run]')
    for name in ('base_color', 'middle_color', 'top_color'):
        p.add_argument(name, choices=COLORS)
    args = p.parse_args(argv)
    if len({args.base_color, args.middle_color, args.top_color}) != 3:
        p.error('stack requires 3 distinct colors')
    if args.dry_run:
        return _run_dry(
            args,
            lambda ex: ex.stack(args.base_color, args.middle_color, args.top_color, dry_run=True),
            filter_by_color=True,
        )
    return _run_action(
        p, args, 'z70_stack_client', Stack, '/z70/stack',
        Stack.Goal(base_color=args.base_color, middle_color=args.middle_color, top_color=args.top_color, dry_run=False),
        'stack goal rejected (another manipulation task active or invalid colors)',
        lambda r: dict(
            tiers_completed=int(r.tiers_completed), stack_verified=bool(r.stack_verified),
            dry_run_valid=bool(r.dry_run_valid), success=bool(r.stack_verified or r.dry_run_valid),
            evidence_uri=r.evidence_uri, error_code=r.outcome.error_code,
        ),
    )


def stack_main():
    _cli_main(run_stack, 'all_targets_visible')


if __name__ == '__main__':
    main()
