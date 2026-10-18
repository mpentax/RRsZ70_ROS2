"""No ROS or transport needed for scheduler invariants."""
import threading
from z70_driver.dispatcher import Dispatcher, ZERO


def test_no_queue_and_task_token_conflicts():
    cancelled = []; stops = []
    d = Dispatcher(lambda scope: stops.append(scope) or [], cancelled.append)
    ticket = d.reserve(); assert ticket and d.reserve() is None
    assert not d.begin()[0]
    d.release(ticket)
    accepted, token, reason = d.begin()
    assert accepted and token != ZERO and reason == 'NONE'
    assert d.reserve() is None and d.reserve(bytes([1])*16) is None
    assert d.begin(token)[0] and not d.begin()[0]
    owned = d.reserve(token); assert owned
    assert not d.end(bytes([1])*16)
    assert d.end(token) and cancelled == [token] and stops == [3]
    assert owned.cancelled.is_set() and owned.stop_done.is_set()
    assert d.reserve(token) is None
    assert d.reserve() is not None


def test_takeover_revokes_then_stop_then_new_write():
    entered = threading.Event(); release = threading.Event(); events = []
    def stop(scope):
        events.append('stop'); entered.set(); assert release.wait(2); events.append('stopped'); return [scope]
    d = Dispatcher(stop, lambda token: events.append('cancel'))
    _, token, _ = d.begin(); old = d.reserve(token)
    new = []
    thread = threading.Thread(target=lambda: new.append(d.takeover())); thread.start()
    assert entered.wait(1)
    assert events == ['cancel', 'stop'] and old.cancelled.is_set()
    assert d.reserve() is None and d.reserve(token) is None
    d.release(old)  # Late command completion cannot free a new reservation.
    release.set(); thread.join(2)
    assert new[0] and events == ['cancel', 'stop', 'stopped']
    assert not d.valid(old) and d.valid(new[0])
    d.release(old); assert d.reserve() is None


def test_cancel_async_revokes_immediately_and_stops_once():
    callbacks = []; stops = []
    d = Dispatcher(lambda scope: stops.append(scope) or [scope])
    ticket = d.reserve()
    d.cancel(ticket, callbacks.append)
    assert ticket.cancelled.is_set() and not ticket.stop_done.is_set()
    assert d.reserve() is None and len(callbacks) == 1
    d.cancel(ticket, callbacks.append); assert len(callbacks) == 1
    callbacks[0]()
    assert stops == [3] and ticket.stop_done.is_set() and ticket.stop_outcomes == [3]
    assert d.reserve() is not None


def test_priority_stop_never_waits_for_reserved_command():
    stopped = []; d = Dispatcher(lambda scope: stopped.append(scope) or [scope])
    old = d.reserve()
    assert d.stop(2) == [2] and stopped == [2]
    assert old.cancelled.is_set() and old.stop_done.is_set()
    assert d.reserve() is not None


def test_suspended_session_cannot_takeover_or_reserve_but_scoped_stop_still_works():
    stops = []; d = Dispatcher(lambda scope: stops.append(scope) or [scope])
    d.session_enabled = False
    assert d.takeover() is None
    assert d.reserve() is None and not d.begin(takeover=True)[0]
    for scope in (1, 2, 3): assert d.stop(scope) == [scope]
    assert stops == [1, 2, 3] and not d.session_enabled


def test_failed_stop_latches_no_blind_takeover_and_successful_stop_recovers():
    received = [False]
    d = Dispatcher(lambda scope: [received[0]], stop_ok=lambda values: all(values))
    old = d.reserve()
    assert d.takeover() is None and old.cancelled.is_set()
    assert d.blocked and d.reserve() is None and not d.begin()[0]
    received[0] = True
    d.stop(3)
    assert not d.blocked and d.reserve() is not None
