"""ROS image/TF perception and single-owner manipulation action servers."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import yaml
from z70_description.arm_model import check as check_arm_limits
from . import perception, planner
from .executor import Client, Executor, log_directory
from z70_interfaces.action import DeliverGarment, PatrolCollect, PinAndGo, Pick, Stack, DeployArm, AlignToTarget, MoveArm, GraspAndLift, ReleaseAndStow
from z70_interfaces.msg import ArmEstimate, CalibrationState, CommandOutcome
from z70_odometry.calibration_state import gains_from_state
from .observations import Observations, LATCHED
from z70_driver.frame_subscription import MatchedFrames

RGB = {'yellow': (1., 1., 0.), 'orange': (1., .4, 0.), 'purple': (.7, 0., 1.), 'blue': (0., .2, 1.), 'green': (0., 1., .2)}


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Perception(Node):
    def __init__(self, name='z70_perception', parameter_overrides=None, subscribe=True):
        super().__init__(name, parameter_overrides=parameter_overrides or [])
        self.repo = Path(self.declare_parameter('repo', str(Path(os.environ.get('Z70_REPO', Path.cwd())))).value)
        config = self.declare_parameter('colors_file', '').value
        self.thresholds = yaml.safe_load(Path(config).read_text()) if config else perception.colors()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.arm_trusted = False
        self.create_subscription(ArmEstimate, '/z70/arm_estimate', self.arm, LATCHED)
        self.pub_json = self.create_publisher(String, '/z70/objects/json', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/z70/objects', 10)
        self.K = planner.GRIPPER_MATRIX.copy()
        self.D = planner.GRIPPER_D.copy()
        self.create_subscription(CameraInfo, '/camera/gripper/camera_info', self.info, 10)
        if subscribe:
            self.frames = [MatchedFrames(self, camera, lambda msg, c=camera: self.image(msg, c), compressed=True)
                           for camera in ('front', 'gripper')]

    def arm(self, msg):
        self.arm_trusted = msg.evidence.valid and not msg.evidence.stale and msg.validity.value in (1, 2) and not msg.restored

    def info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

    def project(self, image, camera, stamp=None):
        if camera == 'front':
            objects = perception.front_objects(image, thresholds=self.thresholds)
        else:
            objects = perception.detect(image, self.thresholds)
            if self.arm_trusted:
                tr = self.tf.lookup_transform('base_footprint', 'gripper_camera_optical_frame', stamp or Time(), timeout=Duration(seconds=.5))
                p = tr.transform.translation
                C = np.array([p.x, p.y, p.z])
                R = rotation(tr.transform.rotation)
                K = self.K.copy()
                K[0, :] *= image.shape[1] / 1600
                K[1, :] *= image.shape[0] / 896
                for b in objects:
                    point = perception.ray_floor(b['contact_px'], K, self.D, C, R)
                    if point:
                        b['position'] = point
                        b['size_m'] = .05
                        b['camera_distance_m'] = float(np.linalg.norm(np.array(point) - C))
                        b['camera_height_m'] = float(C[2])
            for b in objects:
                b['camera'] = 'gripper'
        self.publish(objects, camera, stamp)
        return objects

    def publish(self, objects, camera, stamp=None):
        now = self.get_clock().now().to_msg()
        self.pub_json.publish(String(data=json.dumps(dict(camera=camera, frame_id='base_footprint', stamp_ns=self.get_clock().now().nanoseconds, objects=objects))))
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = 'base_footprint'
        clear.header.stamp = now
        markers = [clear]
        for index, b in enumerate(objects):
            if 'position' not in b:
                continue
            m = Marker()
            m.header.frame_id = 'base_footprint'
            m.header.stamp = now
            m.ns = camera
            m.id = index
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, b['position'])
            m.pose.position.z += b.get('size_m', .05) / 2
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = float(b.get('size_m', .05))
            m.color.r, m.color.g, m.color.b = RGB[b['color']]
            m.color.a = .9
            m.text = b['color']
            markers.append(m)
        self.pub_markers.publish(MarkerArray(markers=markers))

    def image(self, msg, camera):
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        try:
            self.project(image, camera, Time.from_msg(msg.header.stamp))
        except TransformException:
            self.get_logger().warning('camera TF unavailable; next observation will retry', throttle_duration_sec=5)


class PickServer(Perception):
    def __init__(self, parameter_overrides=None):
        super().__init__('z70_pick_server', parameter_overrides, subscribe=False)
        self.log_root = self.declare_parameter('log_root', '').value
        self.safety_stop = False
        self.odom_value = None
        self.odom_at = 0.
        self.lock = threading.Lock()
        self.active_handle = None
        self.navigation_active = False
        self.monitor = None
        self.client = None
        self.gains = {}
        self.deadline = None
        self.create_subscription(Bool, '/z70/safety_stop', self.stop_signal, 10)
        self.create_subscription(Odometry, '/odom', self.odometry, 30)
        self.group = ReentrantCallbackGroup()
        self.observations = Observations(self)
        if hasattr(self.observations, 'node'):
            from .commands import Commands
            Commands(self.observations)
        self.observations.cancelled = lambda: self.client is not None and self.client.commands is not None and self.client.commands.cancelled.is_set()
        self.create_subscription(CalibrationState, '/z70/chassis_calibration', self.calibration, LATCHED)
        self.create_timer(.05, self.watchdog, callback_group=self.group)
        self.action = ActionServer(self, Pick, '/z70/pick', execute_callback=self.execute, goal_callback=self.goal,
                                   cancel_callback=self.cancel, callback_group=self.group)
        self.stack_action = ActionServer(self, Stack, '/z70/stack', execute_callback=self.execute_stack,
                                         goal_callback=self.goal_stack, cancel_callback=self.cancel, callback_group=self.group)
        from z70_mapping.boundary import BoundaryConfig
        self.navigation_config = BoundaryConfig(allowed_segments=tuple(
            self.declare_parameter('pin_and_go_allowed_segments', [1,2,3,4,5,6]).value))
        self.pin_and_go_action = ActionServer(self, PinAndGo, '/z70/nav/pin_and_go',
            execute_callback=self.execute_pin_and_go, goal_callback=self.goal_pin_and_go,
            cancel_callback=self.cancel, callback_group=self.group)
        self.patrol_directory = Path(self.declare_parameter('patrol_ledger_directory', str(self.repo / 'calibration/patrol')).value)
        self.deliver_garment_action = ActionServer(self, DeliverGarment, '/z70/deliver_garment',
            execute_callback=self.execute_deliver_garment, goal_callback=self.goal_garment,
            cancel_callback=self.cancel, callback_group=self.group)
        self.patrol_collect_action = ActionServer(self, PatrolCollect, '/z70/patrol_collect',
            execute_callback=self.execute_patrol_collect, goal_callback=self.goal_garment,
            cancel_callback=self.cancel, callback_group=self.group)
        self.curated_actions = [
            ActionServer(self, DeployArm, '/z70/arm/deploy', execute_callback=self.execute_deploy,
                         goal_callback=self.goal_deploy, cancel_callback=self.cancel, callback_group=self.group),
            ActionServer(self, AlignToTarget, '/z70/align_target', execute_callback=self.execute_align,
                         goal_callback=self.goal_align, cancel_callback=self.cancel, callback_group=self.group),
            ActionServer(self, MoveArm, '/z70/arm/move', execute_callback=self.execute_move,
                         goal_callback=self.goal_move, cancel_callback=self.cancel, callback_group=self.group),
            ActionServer(self, GraspAndLift, '/z70/arm/grasp_and_lift', execute_callback=self.execute_grasp,
                         goal_callback=self.goal_grasp, cancel_callback=self.cancel, callback_group=self.group),
            ActionServer(self, ReleaseAndStow, '/z70/arm/release_and_stow', execute_callback=self.execute_release,
                         goal_callback=self.goal_release, cancel_callback=self.cancel, callback_group=self.group),
        ]

    def calibration(self, msg):
        self.gains = gains_from_state(msg)

    def watchdog(self):
        expired=self.deadline is not None and time.monotonic() >= self.deadline
        navigation_hazard=self.navigation_active and self.safety_stop
        if (expired or navigation_hazard) and self.client and self.client.commands and not self.client.commands.cancelled.is_set():
            self.client.commands.cancelled.set()
            threading.Thread(target=self.client.commands.stop, daemon=True).start()

    def cancel(self, handle):
        if self.client and self.client.commands:
            self.client.commands.cancelled.set()
            threading.Thread(target=self.client.commands.stop, daemon=True).start()
        return CancelResponse.ACCEPT

    def stop_signal(self, msg):
        self.safety_stop = msg.data

    def odometry(self, msg):
        p = msg.pose.pose
        q = p.orientation
        if msg.pose.covariance[0] > 1000:
            return
        self.odom_value = dict(x=p.position.x, y=p.position.y, yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.odom_at = time.monotonic()

    def odom(self):
        return self.odom_value if time.monotonic() - self.odom_at < 2. else None

    def hazard(self):
        if self.safety_stop:
            return 'person/pet/obstacle or operator stop signal'
        if self.active_handle and self.active_handle.is_cancel_requested:
            return 'operator cancelled task'
        if self.client and self.client.commands and self.client.commands.cancelled.is_set():
            return 'task revoked'
        if self.deadline is not None and time.monotonic() >= self.deadline:
            return 'task deadline'
        return None

    def _current_arm_msg(self):
        with getattr(self.observations, 'condition', threading.Lock()):
            arm = getattr(self.observations, 'arm', None)
            return deepcopy(arm) if arm is not None else ArmEstimate()

    def goal(self, request):
        if request.color not in self.thresholds or not self.lock.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def goal_stack(self, request):
        colors = [request.base_color, request.middle_color, request.top_color]
        if len(set(colors)) != 3 or any(c not in self.thresholds for c in colors) or not self.lock.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def goal_pin_and_go(self, request):
        valid = (all(math.isfinite(v) and -100 <= v <= 100 for v in (request.target_x_m,request.target_y_m))
                 and math.isfinite(request.max_duration_s) and 1 <= request.max_duration_s <= 600)
        if not valid or not self.lock.acquire(blocking=False): return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_pin_and_go(self, handle):
        from .navigation import PinAndGoNavigator
        client = None
        self.active_handle = handle
        self.navigation_active = True
        try:
            client = Client('', None, read_only=handle.request.dry_run, observations=self.observations)
            self.client = client
            if client.commands: client.commands.begin()
            self.deadline = time.monotonic()+handle.request.max_duration_s
            def feedback(**fields):
                msg = PinAndGo.Feedback(**fields)
                if client.commands and client.commands.latest_outcome is not None:
                    msg.latest_outcome = client.commands.latest_outcome
                if navigator.latest_observation is not None: msg.latest_observation = navigator.latest_observation
                handle.publish_feedback(msg)
            navigator = PinAndGoNavigator(self.observations,client.commands,boundary_config=self.navigation_config,
                cancelled=lambda:handle.is_cancel_requested or bool(client.commands and client.commands.cancelled.is_set()),
                hazard=lambda:self.safety_stop, odom=self.odom, feedback=feedback)
            result = PinAndGo.Result(**navigator.execute(handle.request))
            if client.commands and client.commands.latest_outcome is not None:
                result.outcome = client.commands.latest_outcome
            if handle.is_cancel_requested: handle.canceled()
            elif result.arrived or result.dry_run_valid: handle.succeed()
            else: handle.abort()
            return result
        except Exception:
            if client and client.commands:
                try: client.commands.stop()
                except (LookupError,TimeoutError): pass
            if handle.is_cancel_requested: handle.canceled()
            else: handle.abort()
            return PinAndGo.Result(status_code='OBSERVATION_GAP',detail='Shared driver observation or command unavailable')
        finally:
            if client:
                if client.commands:
                    try: client.commands.end()
                    except (LookupError,TimeoutError,InterruptedError): pass
                client.close()
            self.navigation_active = False
            self.active_handle = None; self.client = None; self.deadline = None
            self.lock.release()

    def goal_garment(self, request):
        valid = (all(math.isfinite(v) and -100 <= v <= 100 for v in (request.drop_x_m,request.drop_y_m))
                 and math.isfinite(request.max_duration_s) and 1 <= request.max_duration_s <= 3600
                 and (not hasattr(request,'max_items') or 1 <= request.max_items <= 1000))
        return GoalResponse.ACCEPT if valid and self.lock.acquire(blocking=False) else GoalResponse.REJECT

    def map_pose(self, source='base_footprint'):
        tr = self.tf.lookup_transform('map',source,Time(),timeout=Duration(seconds=.5))
        age=(self.get_clock().now().nanoseconds-Time.from_msg(tr.header.stamp).nanoseconds)/1e9
        if not 0 <= age <= 2.: raise LookupError('stale map -> odom -> base TF')
        p=tr.transform.translation; q=tr.transform.rotation
        return (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def _execute_garment_task(self, handle, action_cls, patrol=False):
        from .garment_delivery import GarmentDeliverer
        from .patrol import PatrolCollector, PatrolLedger
        client=None
        self.active_handle=handle; self.navigation_active=True
        try:
            client=Client('',None,read_only=handle.request.dry_run,observations=self.observations)
            self.client=client; self.deadline=time.monotonic()+handle.request.max_duration_s
            directory=(Path(self.log_root) if self.log_root else log_directory(self.repo))
            executor=None if handle.request.dry_run else Executor(client,lambda *_:[],directory,
                hazard=self.hazard,odom=self.odom,calibration=self.gains)
            if client.commands: client.commands.begin()
            def feedback(**fields):
                msg=action_cls.Feedback(**fields)
                if runner.latest_observation is not None: msg.latest_observation=runner.latest_observation
                if client.commands and client.commands.latest_outcome is not None: msg.latest_outcome=client.commands.latest_outcome
                handle.publish_feedback(msg)
            if patrol:
                runner=PatrolCollector(self.observations,executor,PatrolLedger(self.patrol_directory),
                    boundary_config=self.navigation_config,pose=lambda:self.map_pose(),
                    map_odom=lambda:self.map_pose('odom'),feedback=feedback)
            else:
                runner=GarmentDeliverer(self.observations,executor,boundary_config=self.navigation_config,
                    pose=lambda:self.map_pose(),map_odom=lambda:self.map_pose('odom'),feedback=feedback)
            data=runner.execute(handle.request)
            success=(data.get('delivered',False) or data.get('completed',False) or data.get('dry_run_valid',False)
                     or data.get('status_code')=='PARTIAL_COMPLETION_SMALL_ITEMS_ONLY')
            result=action_cls.Result(**data)
            result.outcome=(deepcopy(client.commands.latest_outcome) if client.commands and client.commands.latest_outcome is not None
                            else CommandOutcome(error_code='NONE' if success else 'OBSERVATION_GAP'))
            if handle.is_cancel_requested: handle.canceled()
            elif success: handle.succeed()
            else: handle.abort()
            return result
        except Exception as exc:
            self.get_logger().warning('garment task failed: '+type(exc).__name__)
            if handle.is_cancel_requested: handle.canceled()
            else: handle.abort()
            return action_cls.Result(status_code='OBSERVATION_GAP',detail='ROS context, observation or execution unavailable',
                                     outcome=CommandOutcome(error_code='OBSERVATION_GAP'))
        finally:
            if client:
                if client.commands:
                    try: client.commands.end()
                    except (LookupError,TimeoutError,InterruptedError): pass
                client.close()
            self.navigation_active=False; self.active_handle=None; self.client=None; self.deadline=None
            self.lock.release()

    def execute_deliver_garment(self, handle):
        return self._execute_garment_task(handle,DeliverGarment)

    def execute_patrol_collect(self, handle):
        return self._execute_garment_task(handle,PatrolCollect,patrol=True)

    def goal_deploy(self, request):
        return GoalResponse.ACCEPT if self.lock.acquire(blocking=False) else GoalResponse.REJECT

    def goal_align(self, request):
        if request.color not in self.thresholds or not self.lock.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def goal_move(self, request):
        valid = (all(math.isfinite(x) for x in (request.m3_deg, request.m4_deg, request.m5_deg))
                 and check_arm_limits(request.m3_deg, request.m4_deg, request.m5_deg) == 'SUCCESS'
                 and planner.safe((request.m3_deg, request.m4_deg, request.m5_deg, 90)))
        if not valid or not self.lock.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def goal_grasp(self, request):
        valid = 0 <= request.lift_steps <= 10 and (not request.verify_color or request.verify_color in self.thresholds)
        if not valid or not self.lock.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def goal_release(self, request):
        return GoalResponse.ACCEPT if self.lock.acquire(blocking=False) else GoalResponse.REJECT

    def _run_curated(self, handle, action_cls, timeout_s, fn, build_result):
        self.active_handle = handle
        client = None
        try:
            client = Client('', None, read_only=False, observations=self.observations)
            self.client = client
            self.deadline = time.monotonic() + timeout_s
            directory = log_directory(self.repo) if not self.log_root else Path(self.log_root) / log_directory(self.repo).name

            def observe(camera, color):
                image, tag = client.frame()
                if tag != camera:
                    raise LookupError('frame camera tag mismatch')
                if camera == 'gripper':
                    estimate = client.get('/api/arm/estimate')
                    self.arm_trusted = not executor.gripped and estimate.get('state') not in ('UNKNOWN', 'IN_ROBOT')
                try:
                    objects = self.project(image, camera)
                except TransformException:
                    raise LookupError('camera TF missing')
                return [b for b in objects if b['color'] == color]

            def feedback(phase):
                handle.publish_feedback(action_cls.Feedback(phase=3, reason='NONE'))

            def sleep(duration):
                until = time.monotonic() + duration
                while time.monotonic() < until:
                    if self.hazard():
                        raise InterruptedError('CANCELLED_OR_DEADLINE')
                    time.sleep(min(.05, max(0, until - time.monotonic())))

            executor = Executor(client, observe, directory, sleep=sleep, hazard=self.hazard, odom=self.odom, feedback=feedback, calibration=self.gains)
            try:
                if client.commands:
                    client.commands.begin()
                data = fn(executor)
                success = True
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.succeed()
            except Exception as e:
                data = dict(error=type(e).__name__, reason=str(e))
                executor.log('failure', **data)
                success = False
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
            finally:
                if client.commands:
                    try:
                        client.commands.end()
                    except (LookupError, TimeoutError):
                        pass
                client.close()
            outcome = client.commands.latest_outcome if client.commands else None
            if outcome is None:
                outcome = CommandOutcome(error_code='NONE' if success else 'OBSERVATION_GAP')
            return build_result(outcome, success, data)
        except Exception:
            handle.abort()
            return build_result(CommandOutcome(error_code='OBSERVATION_GAP'), False, {})
        finally:
            self.active_handle = None
            self.client = None
            self.deadline = None
            self.lock.release()

    def execute_deploy(self, handle):
        return self._run_curated(
            handle, DeployArm, 90.,
            lambda ex: ex.deploy(settle=handle.request.settle),
            lambda outcome, ok, data: DeployArm.Result(outcome=outcome, estimate=self._current_arm_msg(), ready=bool(ok and data.get('ready'))),
        )

    def execute_align(self, handle):
        return self._run_curated(
            handle, AlignToTarget, 120.,
            lambda ex: ex.align_front(handle.request.color),
            lambda outcome, ok, data: AlignToTarget.Result(
                outcome=outcome,
                aligned=bool(ok and 'position' in data),
                target_position=[float(x) for x in data.get('position', [0., 0., 0.])[:3]],
                bearing_error_deg=float(data.get('bearing_error_deg', 0.)),
                range_error_m=float(data.get('range_error_m', 0.)),
            ),
        )

    def execute_move(self, handle):
        req = handle.request
        return self._run_curated(
            handle, MoveArm, 120.,
            lambda ex: ex.move_arm((req.m3_deg, req.m4_deg, req.m5_deg)),
            lambda outcome, ok, data: MoveArm.Result(
                outcome=outcome,
                estimate=self._current_arm_msg(),
                steps_executed=int(data.get('steps_executed', 0)) if ok else 0,
            ),
        )

    def execute_grasp(self, handle):
        req = handle.request
        return self._run_curated(
            handle, GraspAndLift, 90.,
            lambda ex: ex.grasp_and_lift(lift_steps=req.lift_steps, verify_color=req.verify_color),
            lambda outcome, ok, data: GraspAndLift.Result(
                outcome=outcome,
                estimate=self._current_arm_msg(),
                verified_hold=bool(ok and data.get('verified')),
            ),
        )

    def execute_release(self, handle):
        req = handle.request
        return self._run_curated(
            handle, ReleaseAndStow, 90.,
            lambda ex: ex.release_and_stow(put_down=req.put_down),
            lambda outcome, ok, data: ReleaseAndStow.Result(
                outcome=outcome,
                estimate=self._current_arm_msg(),
                stowed=bool(ok and data.get('stowed')),
            ),
        )

    def execute(self, handle):
        self.active_handle = handle
        try:
            return self._execute(handle)
        except Exception:
            handle.abort()
            return Pick.Result(outcome=CommandOutcome(error_code='OBSERVATION_GAP'))
        finally:
            self.active_handle = None
            self.client = None
            self.deadline = None
            self.lock.release()

    def _execute(self, handle):
        client = Client('', None, read_only=handle.request.dry_run, observations=self.observations)
        self.client = client
        self.deadline = time.monotonic() + 285.
        directory = log_directory(self.repo) if not self.log_root else Path(self.log_root) / log_directory(self.repo).name

        def observe(camera, color):
            image, tag = client.frame()
            if tag != camera:
                raise LookupError('frame camera tag mismatch')
            if camera == 'gripper':
                estimate = client.get('/api/arm/estimate')
                self.arm_trusted = not executor.gripped and estimate.get('state') not in ('UNKNOWN', 'IN_ROBOT')
            try:
                objects = self.project(image, camera)
            except TransformException:
                raise LookupError('camera TF missing')
            return [b for b in objects if b['color'] == color]

        def feedback(phase):
            handle.publish_feedback(Pick.Feedback(phase=3, reason='NONE'))

        def sleep(duration):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                if self.hazard():
                    raise InterruptedError('CANCELLED_OR_DEADLINE')
                time.sleep(min(.05, max(0, until - time.monotonic())))

        executor = Executor(client, observe, directory, sleep=sleep, hazard=self.hazard, odom=self.odom, feedback=feedback, calibration=self.gains)
        try:
            if client.commands:
                client.commands.begin()
            report = executor.pick(handle.request.color, dry_run=handle.request.dry_run)
            success = bool(report.get('verified', report.get('dry_run') and report.get('target') is not None))
            if handle.is_cancel_requested:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()
        except Exception as e:
            report = dict(error=type(e).__name__, reason=str(e), log_dir=str(directory))
            executor.log('failure', **report)
            success = False
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            if client.commands:
                try:
                    client.commands.end()
                except (LookupError, TimeoutError):
                    pass
            client.close()
        outcome = client.commands.latest_outcome if client.commands else None
        if outcome is None:
            outcome = CommandOutcome(error_code='NONE' if success else 'OBSERVATION_GAP')
        return Pick.Result(outcome=outcome, verified_hold=bool(report.get('verified')),
                           dry_run_valid=bool(handle.request.dry_run and success), evidence_uri=directory.name)

    def execute_stack(self, handle):
        self.active_handle = handle
        try:
            return self._execute_stack(handle)
        except Exception:
            handle.abort()
            return Stack.Result(outcome=CommandOutcome(error_code='OBSERVATION_GAP'))
        finally:
            self.active_handle = None
            self.client = None
            self.deadline = None
            self.lock.release()

    def _execute_stack(self, handle):
        req = handle.request
        client = Client('', None, read_only=req.dry_run, observations=self.observations)
        self.client = client
        self.deadline = time.monotonic() + (60. if req.dry_run else 540.)
        directory = log_directory(self.repo) if not self.log_root else Path(self.log_root) / log_directory(self.repo).name

        def observe(camera, color):
            image, tag = client.frame()
            if tag != camera:
                raise LookupError('frame camera tag mismatch')
            if camera == 'gripper':
                estimate = client.get('/api/arm/estimate')
                self.arm_trusted = not executor.gripped and estimate.get('state') not in ('UNKNOWN', 'IN_ROBOT')
            try:
                objects = self.project(image, camera)
            except TransformException:
                raise LookupError('camera TF missing')
            return [b for b in objects if b['color'] == color]

        def feedback(phase):
            handle.publish_feedback(Stack.Feedback(phase=3, current_tier=1, reason='NONE'))

        def sleep(duration):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                if self.hazard():
                    raise InterruptedError('CANCELLED_OR_DEADLINE')
                time.sleep(min(.05, max(0, until - time.monotonic())))

        executor = Executor(client, observe, directory, sleep=sleep, hazard=self.hazard, odom=self.odom, feedback=feedback, calibration=self.gains)
        try:
            if client.commands:
                client.commands.begin()
            report = executor.stack(req.base_color, req.middle_color, req.top_color, dry_run=req.dry_run)
            success = bool(report.get('stack_verified', report.get('dry_run') and report.get('all_targets_visible')))
            if handle.is_cancel_requested:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()
        except Exception as e:
            report = dict(error=type(e).__name__, reason=str(e), log_dir=str(directory))
            executor.log('failure', **report)
            success = False
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            if client.commands:
                try:
                    client.commands.end()
                except (LookupError, TimeoutError):
                    pass
            client.close()
        outcome = client.commands.latest_outcome if client.commands else None
        if outcome is None:
            outcome = CommandOutcome(error_code='NONE' if success else 'OBSERVATION_GAP')
        tiers_done = 3 if (req.dry_run and success) else int(report.get('tiers_completed', 0))
        return Stack.Result(
            outcome=outcome,
            tiers_completed=tiers_done,
            stack_verified=bool(report.get('stack_verified')),
            dry_run_valid=bool(req.dry_run and success),
            evidence_uri=directory.name,
        )


def run(node_type):
    rclpy.init()
    node = node_type()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def perception_main():
    run(Perception)


def server_main():
    run(PickServer)
