"""Thin ROS command client; no vendor transport or credentials in manipulation."""
import json
import threading
import time
from unique_identifier_msgs.msg import UUID
from z70_interfaces.srv import RawRpc, Stop, TaskSession


class _Transport:
    """Node-owned entities outlive individual tasks and the executor wait set."""
    def __init__(self, observations):
        self.node = observations.node
        self.group = observations.group
        self.clients = {kind:self.node.create_client(kind,name,callback_group=self.group)
            for kind,name in ((RawRpc,'/z70/driver/raw_rpc'),(Stop,'/z70/stop'),(TaskSession,'/z70/tasks/session'))}
        self.active: 'Commands | None' = None
        self.subscription = self.node.create_subscription(UUID,'/z70/tasks/cancel',self.revoked,10,callback_group=self.group)

    def revoked(self, token):
        active=self.active
        if active is not None: active.revoked(token)


class Commands:
    def __init__(self, observations):
        self.node = observations.node
        transport=getattr(observations,'_command_transport',None)
        if transport is None:
            transport=_Transport(observations)
            observations._command_transport=transport
        self.transport=transport
        self.clients=transport.clients
        self.task_id = UUID()
        self.cancelled = threading.Event()
        self.latest_outcome = None
        self.stop_lock=threading.Lock();self.stop_future=None;self.driver_revoked=False
        transport.active=self

    def revoked(self, token):
        if any(self.task_id.uuid) and token == self.task_id:
            self.driver_revoked=True;self.cancelled.set()

    def call(self, kind, request, timeout, cancellable=True):
        client=self.clients[kind]
        if not client.wait_for_service(timeout_sec=1.): raise LookupError('ROS_DRIVER_UNAVAILABLE')
        future=client.call_async(request); done=threading.Event();future.add_done_callback(lambda _:done.set())
        deadline=time.monotonic()+timeout
        while not done.wait(.02):
            if cancellable and self.cancelled.is_set(): raise InterruptedError('CANCELLED')
            if time.monotonic()>=deadline: raise TimeoutError('ROS_DEADLINE_UNKNOWN_DISPATCH')
        return future.result()

    def begin(self):
        response=self.call(TaskSession,TaskSession.Request(begin=True),3.)
        if not response.accepted: raise LookupError(response.reason)
        self.task_id=response.task_id

    def end(self):
        if any(self.task_id.uuid):
            try: self.call(TaskSession,TaskSession.Request(begin=False,task_id=self.task_id),16.,False)
            finally: self.task_id=UUID()

    def rpc(self,method,params,timeout_ms):
        if self.cancelled.is_set(): raise InterruptedError('CANCELLED')
        response=self.call(RawRpc,RawRpc.Request(method=method,params_json=json.dumps(params,allow_nan=False),
            timeout_ms=timeout_ms,task_id=self.task_id),timeout_ms/1000+1)
        self.latest_outcome=o=response.outcome
        if o.error_code in ('BUSY','CANCELLED','STOP_UNVERIFIED'):
            if any(self.task_id.uuid): self.driver_revoked=True
            self.cancelled.set();raise InterruptedError(o.error_code)
        return dict(dispatch={0:'not_sent',1:'sent',2:'unknown'}[o.dispatch],
            response={0:'none',1:'received',2:'vendor_error',3:'transport_error'}[o.response],
            transportReason='timeout' if o.error_code in ('TIMEOUT','DEADLINE') else o.error_code,
            vendorCode=o.vendor_code if o.vendor_code_known else None,
            result=json.loads(response.result_json) if response.result_json else None)

    def stop(self):
        self.cancelled.set()
        with self.stop_lock:
            # Takeover already owns the ordered stop. A revoked task must not
            # send a second stop after the new controller has started moving.
            if self.driver_revoked and self.stop_future is None: return None
            if self.stop_future is None:
                client=self.clients[Stop]
                if not client.wait_for_service(timeout_sec=1.): raise LookupError('PHYSICAL_STOP_REQUIRED')
                self.stop_future=client.call_async(Stop.Request(scope=3))
            future=self.stop_future
        done=threading.Event();future.add_done_callback(lambda _:done.set())
        if not done.wait(15.5): raise TimeoutError('PHYSICAL_STOP_REQUIRED')
        return future.result()

    def close(self):
        # Destroying clients while a shared executor enumerates QoS waitables
        # can kill its spin thread. Node shutdown owns native entity disposal;
        # task retirement only releases this task's cancellation binding.
        if self.transport.active is self: self.transport.active=None
