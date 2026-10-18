"""pixi run pick COLOR [--dry-run]. Dry-run never starts the action or selects a camera."""
import argparse
import json
import os
from pathlib import Path
import cv2
from . import perception
from .executor import Client,Executor,log_directory

def run(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('color',choices=['yellow','purple','blue','green','orange'])
    p.add_argument('--dry-run',action='store_true')
    p.add_argument('--origin',default='http://127.0.0.1:57010')
    p.add_argument('--repo',type=Path,default=Path(os.environ.get('Z70_REPO',Path.cwd())))
    p.add_argument('--token-file',type=Path)
    p.add_argument('--log-dir',type=Path)
    args=p.parse_args(argv)
    if args.dry_run:
        directory=args.log_dir or log_directory(args.repo)
        client=Client(args.origin,args.token_file or args.repo/'.local/service-token',read_only=True)
        def observe(camera,color):
            image,tag=client.frame()
            if tag!='front': raise LookupError('dry-run requires an already-selected front camera; no selection was sent')
            directory.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(directory/'front.jpg'),image)
            return perception.front_objects(image)
        executor=Executor(client,observe,directory)
        try:
            report=executor.pick(args.color,dry_run=True)
            (directory/'dry-run.json').write_text(json.dumps(report,indent=2)+'\n')
            return report
        finally: client.close()
    if args.origin!=p.get_default('origin') or args.token_file or args.log_dir:
        p.error('connection/log options configure --dry-run only; configure the ROS pick server for live execution')
    import rclpy
    from rclpy.action import ActionClient
    from z70_interfaces.action import Pick
    rclpy.init();node=rclpy.create_node('z70_pick_client');action=ActionClient(node,Pick,'/z70/pick');handle=None
    try:
        if not action.wait_for_server(timeout_sec=10): raise RuntimeError('/z70/pick server unavailable; launch bringup manipulation:=true')
        future=action.send_goal_async(Pick.Goal(color=args.color,dry_run=False))
        rclpy.spin_until_future_complete(node,future);handle=future.result()
        if not handle.accepted: raise RuntimeError('pickup goal rejected (another pickup active or unknown colour)')
        future=handle.get_result_async();rclpy.spin_until_future_complete(node,future)
        result=future.result().result;report=dict(verified=result.verified_hold,dry_run_valid=result.dry_run_valid,
            success=result.verified_hold or result.dry_run_valid,evidence_uri=result.evidence_uri,
            error_code=result.outcome.error_code)
        return report
    except KeyboardInterrupt:
        if handle:
            future=handle.cancel_goal_async();rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        raise
    finally: action.destroy();node.destroy_node();rclpy.try_shutdown()

def main():
    try:
        report=run();print(json.dumps(report,indent=2))
        if not report.get('target') and report.get('dry_run') or report.get('success') is False: raise SystemExit(1)
    except (RuntimeError,LookupError) as e: raise SystemExit(str(e))


def run_stack(argv=None):
    p=argparse.ArgumentParser(description='pixi run stack BASE_COLOR MIDDLE_COLOR TOP_COLOR [--dry-run]')
    for name in ('base_color','middle_color','top_color'):
        p.add_argument(name,choices=['yellow','purple','blue','green','orange'])
    p.add_argument('--dry-run',action='store_true')
    p.add_argument('--origin',default='http://127.0.0.1:57010')
    p.add_argument('--repo',type=Path,default=Path(os.environ.get('Z70_REPO',Path.cwd())))
    p.add_argument('--token-file',type=Path)
    p.add_argument('--log-dir',type=Path)
    args=p.parse_args(argv)
    if len({args.base_color,args.middle_color,args.top_color})!=3:
        p.error('stack requires 3 distinct colors')
    if args.dry_run:
        directory=args.log_dir or log_directory(args.repo)
        client=Client(args.origin,args.token_file or args.repo/'.local/service-token',read_only=True)
        cached_objects=None
        def observe(camera,color):
            nonlocal cached_objects
            if cached_objects is None:
                image,tag=client.frame()
                if tag!='front': raise LookupError('dry-run requires an already-selected front camera; no selection was sent')
                directory.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(directory/'front.jpg'),image)
                cached_objects=perception.front_objects(image)
            return [b for b in cached_objects if b.get('color')==color]
        executor=Executor(client,observe,directory)
        try:
            report=executor.stack(args.base_color,args.middle_color,args.top_color,dry_run=True)
            (directory/'dry-run.json').write_text(json.dumps(report,indent=2)+'\n')
            return report
        finally: client.close()
    if args.origin!=p.get_default('origin') or args.token_file or args.log_dir:
        p.error('connection/log options configure --dry-run only; configure the ROS pick server for live execution')
    import rclpy
    from rclpy.action import ActionClient
    from z70_interfaces.action import Stack
    rclpy.init();node=rclpy.create_node('z70_stack_client');action=ActionClient(node,Stack,'/z70/stack');handle=None
    try:
        if not action.wait_for_server(timeout_sec=10): raise RuntimeError('/z70/stack server unavailable; launch bringup manipulation:=true')
        future=action.send_goal_async(Stack.Goal(base_color=args.base_color,middle_color=args.middle_color,top_color=args.top_color,dry_run=False))
        rclpy.spin_until_future_complete(node,future);handle=future.result()
        if not handle.accepted: raise RuntimeError('stack goal rejected (another manipulation task active or invalid colors)')
        future=handle.get_result_async();rclpy.spin_until_future_complete(node,future)
        result=future.result().result;report=dict(
            tiers_completed=int(result.tiers_completed),
            stack_verified=bool(result.stack_verified),
            dry_run_valid=bool(result.dry_run_valid),
            success=bool(result.stack_verified or result.dry_run_valid),
            evidence_uri=result.evidence_uri,
            error_code=result.outcome.error_code)
        return report
    except KeyboardInterrupt:
        if handle:
            future=handle.cancel_goal_async();rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        raise
    finally: action.destroy();node.destroy_node();rclpy.try_shutdown()


def stack_main():
    try:
        report=run_stack();print(json.dumps(report,indent=2))
        if (report.get('dry_run') and not report.get('all_targets_visible')) or report.get('success') is False: raise SystemExit(1)
    except (RuntimeError,LookupError,ValueError) as e: raise SystemExit(str(e))

if __name__=='__main__': main()
