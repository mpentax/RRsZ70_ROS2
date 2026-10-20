"""Pure bounded scheduler. State locks never cover network/camera waits."""
from dataclasses import dataclass, field
import threading
import uuid

ZERO = bytes(16)


@dataclass(eq=False)
class Ticket:
    task_id: bytes = ZERO
    cancelled: threading.Event = field(default_factory=threading.Event)
    stop_done: threading.Event = field(default_factory=threading.Event)
    stop_outcomes: list = field(default_factory=list)


class Dispatcher:
    def __init__(self, stop, cancel_task=lambda token: None, stop_ok=lambda outcomes: True):
        self.lock = threading.RLock()
        self.active = None
        self.task_id = ZERO
        self.stopping = 0
        self.blocked = False
        self.session_enabled = True
        self._stop = stop
        self._stop_ok = stop_ok
        self._cancel_task = cancel_task

    def reserve(self, task_id=ZERO):
        with self.lock:
            if not self.session_enabled or self.blocked or self.stopping or self.active is not None or task_id != self.task_id:
                return None
            self.active = Ticket(task_id)
            return self.active

    def valid(self, ticket):
        with self.lock:
            return self.session_enabled and self.active is ticket and not ticket.cancelled.is_set() and not self.stopping

    def release(self, ticket):
        with self.lock:
            if self.active is ticket:
                self.active = None

    def _revoke(self):
        ticket, task = self.active, self.task_id
        if ticket:
            ticket.cancelled.set()
        self.active = None
        self.task_id = ZERO
        if task != ZERO:
            self._cancel_task(task)
        return ticket

    def stop(self, scope=3, ticket=None):
        # Concurrent callbacks for the same cancellation share its single stop.
        with self.lock:
            if ticket is not None and ticket.cancelled.is_set():
                return ticket.stop_outcomes
            self.stopping += 1
            previous = self._revoke()
        try:
            outcomes = self._stop(scope)
            with self.lock:
                self.blocked = not self._stop_ok(outcomes)
            if previous:
                previous.stop_outcomes = outcomes
            return outcomes
        finally:
            with self.lock:
                if previous:
                    previous.stop_done.set()
                self.stopping -= 1

    def begin(self, task_id=ZERO, takeover=False):
        if not self.session_enabled:
            return False, ZERO, 'SESSION_STOPPED'
        if takeover:
            if task_id != ZERO:
                return False, ZERO, 'INVALID_INPUT'
            # Guard the entire transition, including the stop network wait.
            with self.lock:
                if not self.session_enabled or self.stopping:
                    return False, ZERO, 'BUSY'
                self.stopping += 1
            try:
                self.stop(3)
                with self.lock:
                    if not self.session_enabled or self.blocked:
                        return False, ZERO, 'STOP_UNVERIFIED'
                    self.task_id = uuid.uuid4().bytes
                    return True, self.task_id, 'NONE'
            finally:
                with self.lock:
                    self.stopping -= 1
        with self.lock:
            if not self.session_enabled or self.stopping or self.blocked:
                return False, ZERO, 'BUSY'
            if self.task_id != ZERO:
                return (True, self.task_id, 'NONE') if task_id == self.task_id else (False, ZERO, 'BUSY')
            if self.active is not None:
                return False, ZERO, 'BUSY'
            if task_id != ZERO:
                return False, ZERO, 'INVALID_INPUT'
            self.task_id = uuid.uuid4().bytes
            return True, self.task_id, 'NONE'

    def _finish_stop(self, ticket):
        try:
            ticket.stop_outcomes = self._stop(3)
            with self.lock:
                self.blocked = not self._stop_ok(ticket.stop_outcomes)
        finally:
            with self.lock:
                ticket.stop_done.set()
                self.stopping -= 1

    def end(self, task_id):
        with self.lock:
            if task_id == ZERO or task_id != self.task_id:
                return False
            previous = self._revoke()
            if previous:
                self.stopping += 1
        if previous:
            self._finish_stop(previous)
        return True

    def cancel(self, ticket, launch):
        with self.lock:
            if ticket.cancelled.is_set():
                return
            self.stopping += 1
            self._revoke()
        launch(lambda: self._finish_stop(ticket))

    def takeover(self):
        # Return a reserved manual ticket only after stop has completed, with no
        # gap in which another caller could dispatch before this transition.
        with self.lock:
            if not self.session_enabled or self.stopping:
                return None
            self.stopping += 1
        try:
            self.stop(3)
            with self.lock:
                if not self.session_enabled or self.blocked:
                    return None
                self.active = Ticket()
                return self.active
        finally:
            with self.lock:
                self.stopping -= 1
