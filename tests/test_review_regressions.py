# SPDX-License-Identifier: MIT
"""Regression coverage for the independent review's Python failure paths.

Crash regressions run in subprocesses, so a bad extension fails a test rather
than taking down the full test runner. No physical servo is required.
"""
import ast
import os
from pathlib import Path
import subprocess
import sys
import textwrap
import threading
import time

import pytest
import hiwonder_bus as h
from hiwonder_bus.exceptions import ServoArgumentError, ServoIOError, ServoLogicalError
from pylx16a import LX16A, _BSpline


def isolated_script(script):
    # Do not allow an old installed/editable module to mask the extension
    # currently under test. -S also disables editable-install .pth hooks.
    root = str(Path(h.__file__).resolve().parent.parent)
    env = dict(os.environ, PYTHONPATH=root)
    result = subprocess.run([sys.executable, "-S", "-c", textwrap.dedent(script)],
                            env=env, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)


def test_servo_retains_bus_until_collected():
    isolated_script("""
        import gc, weakref
        import hiwonder_bus as h
        b = h.make_simulated_bus([1])
        s = h.Servo(b, 1)
        ref = weakref.ref(b)
        del b
        gc.collect()
        assert ref() is not None
        assert s.get_temp() > 0
        del s
        gc.collect()
        assert ref() is None, 'bus must not leak through a keep-alive cycle'
    """)


@pytest.mark.parametrize("method", ["command", "query"])
def test_invalid_raw_inputs_raise_without_crashing(method):
    isolated_script(f"""
        import hiwonder_bus as h
        from hiwonder_bus.exceptions import ServoArgumentError
        b = h.make_simulated_bus([1])
        for args in [(-1,28), (256,28), (1,-1), (1,256), (1,28,[256]), (1,28,[-1])]:
            try:
                b.{method}(*args)
            except ServoArgumentError:
                pass
            else:
                raise AssertionError(args)
        assert b.stats().writes == 0
    """)


@pytest.mark.parametrize("duration", [-65536, -1, 30001, 65536, 65537])
def test_durations_checked_before_narrowing(duration):
    bus = h.make_simulated_bus([1])
    servo = h.Servo(bus, 1)
    servo.enable_torque()
    bus.reset_stats()
    for operation in [
        lambda: servo.move(40, duration),
        lambda: servo.move_to(40, duration),
        lambda: servo.move_relative(10, duration),
        lambda: bus.move_sync([1], [40], duration),
        lambda: h.move_sync(bus, [servo], [40], duration),
    ]:
        with pytest.raises(ServoArgumentError):
            operation()
    assert servo.try_move(40, duration) == (False, h.ErrorCode.INVALID_ARGUMENT)
    assert servo.last_error() == h.ErrorCode.INVALID_ARGUMENT
    assert bus.stats().writes == 0


@pytest.mark.parametrize("speed", [-65537, -1001, 1001, 65536, 65537])
def test_speed_checked_before_narrowing(speed):
    bus = h.make_simulated_bus([1])
    servo = h.Servo(bus, 1)
    servo.enable_torque()
    bus.reset_stats()
    with pytest.raises(ServoArgumentError):
        servo.set_motor_mode(speed)
    assert bus.stats().writes == 0


def test_relative_motion_validates_final_destination_and_wait():
    bus = h.make_simulated_bus([1])
    servo = h.Servo(bus, 1)
    servo.enable_torque()
    servo.move(230)
    before = bus.stats().writes
    with pytest.raises(ServoArgumentError):
        servo.move(20, relative=True)
    assert bus.stats().writes == before
    servo.move(-20, relative=True)
    assert servo.get_commanded_angle() == pytest.approx(210, abs=0.25)
    servo.move(-10, relative=True, wait=True)
    assert servo.get_waiting_angle() == pytest.approx(200, abs=0.25)
    assert servo.get_commanded_angle() == pytest.approx(210, abs=0.25)


@pytest.mark.parametrize("mode", [h.ErrorMode.STRICT, h.ErrorMode.NON_THROWING])
def test_batches_validate_all_inputs_before_transmission(mode):
    bus = h.make_simulated_bus([1, 2])
    other = h.make_simulated_bus([1])
    servo = h.Servo(bus, 1)
    foreign = h.Servo(other, 1)
    bus.error_mode = mode
    with pytest.raises(ServoLogicalError):
        h.move_sync(bus, [servo], [90])  # torque mirror is disabled
    servo.enable_torque()
    servo.set_angle_limits(50, 100)
    bus.reset_stats()
    for operation in [
        lambda: h.move_sync(bus, [servo], []),
        lambda: h.move_sync(bus, [servo], [200]),
        lambda: h.move_sync(bus, [foreign], [90]),
        lambda: h.move_sync(bus, [None], [90]),
        lambda: h.set_torque(bus, [foreign], True),
        lambda: h.set_torque(bus, [], True),
        lambda: h.read_angles(bus, [foreign]),
        lambda: h.read_telemetry(bus, [foreign]),
        lambda: bus.move_sync([1, 254], [90, 90]),
        lambda: bus.set_torque([255], True),
    ]:
        with pytest.raises(ServoArgumentError):
            operation()
    assert bus.stats().writes == 0


def test_polled_mode_and_torque_update_cached_authorization():
    bus = h.make_simulated_bus([1])
    servo = h.Servo(bus, 1)
    # Hardware torque starts enabled, unlike a new conservative mirror.
    assert servo.is_torque_enabled(True)
    assert servo.is_torque_enabled(False)
    servo.move(90)
    bus.command(1, 29, [1, 0, 0x0c, 0xfe])  # motor speed -500
    assert servo.get_motor_speed(True) == -500
    assert servo.is_motor_mode(False)
    with pytest.raises(ServoLogicalError):
        servo.move(45)
    bus.command(1, 29, [0, 0, 0, 0])
    assert not servo.is_motor_mode(True)
    bus.command(1, 31, [0])
    assert not servo.is_torque_enabled(True)
    assert not servo.is_torque_enabled(False)
    with pytest.raises(ServoLogicalError):
        servo.move(45)


def test_bus_config_is_a_snapshot_and_errors_recover():
    bus = h.make_simulated_bus([1])
    cfg = bus.config
    old_timeout = bus.read_timeout_ms
    cfg.read_timeout_ms = 999
    assert bus.read_timeout_ms == old_timeout
    assert bus.update_config(cfg)
    assert bus.read_timeout_ms == 999
    bus.read_timeout_ms = 5
    assert bus.query(2, 26) == []
    assert bus.last_error_code() == h.ErrorCode.TIMEOUT
    assert "timeout" in bus.last_error().lower()
    assert bus.query(1, 26)
    assert bus.last_error_code() == h.ErrorCode.OK
    assert bus.last_error() == ""


def test_open_failures_use_the_same_exception_bridge():
    for operation in [lambda: h.SerialBus("/definitely/not/a/serial/device"),
                      lambda: h.connect("/definitely/not/a/serial/device"),
                      lambda: LX16A.initialize("/definitely/not/a/serial/device")]:
        with pytest.raises(ServoIOError) as failure:
            operation()
        assert failure.value.id_ is None


@pytest.mark.parametrize("knots", [[0, 0, 0, 1, 1, 1], [0, 1], [2, 2, 2, 5, 5, 5]])
def test_spline_closed_endpoint_and_clamping(knots):
    spline = _BSpline(knots, [(0, 40), (0.5, 120), (1, 60)], 2, 50)
    assert spline.samples[0] == pytest.approx((0, 40))
    assert spline.samples[-1] == pytest.approx((1, 60))
    assert spline.sample_x(-1) == pytest.approx(40)
    assert spline.sample_x(1) == pytest.approx(60)
    assert spline.sample_x(2) == pytest.approx(60)


def test_spline_invalid_domain_is_a_typed_argument_error():
    LX16A.attach_simulator([1])
    servo = LX16A(1)
    try:
        for knots in [[0, 0, 0, 0, 0, 0], [0, 1, 0, 1, 1, 1], [0, 0, 0, 1, 1, float('nan')]]:
            with pytest.raises(ServoArgumentError):
                servo.set_bspline(knots, [(0, 40), (0.5, 120), (1, 60)], 2)
    finally:
        LX16A._controller.close()
        LX16A._controller = None


def test_typing_stub_is_valid_python_and_marker_is_packaged():
    package = Path(h.__file__).resolve().parent
    ast.parse((package / "__init__.pyi").read_text())
    assert (package / "py.typed").is_file()


def test_simulator_disconnect_ignores_broadcast():
    bus = h.make_simulated_bus([1, 2])
    before = bus.read_angles([1])[0]
    h.simulate_disconnect(bus, 1)
    bus.command(254, 7, [0xf4, 1, 0, 0])  # broadcast queue 500 ticks
    bus.broadcast_move_start()
    h.simulate_disconnect(bus, 1, True)
    assert bus.read_angles([1])[0] == pytest.approx(before)
    assert bus.read_angles([2])[0] == pytest.approx(120)


@pytest.mark.skipif(sys.platform != "linux", reason="Linux POSIX backend")
def test_gil_released_during_blocked_posix_query():
    # A silent real PTY blocks the backend in poll; simulator timing cannot
    # establish GIL progress. No driver, pyserial, or powered hardware needed.
    import pty
    master, slave = pty.openpty()
    bus = h.SerialBus(os.ttyname(slave), read_timeout_ms=150)
    entered = threading.Event()
    done = threading.Event()
    errors = []

    def query():
        try:
            entered.set()
            assert bus.query(1, 26, timeout_ms=150) == []
        except BaseException as error:
            errors.append(error)
        finally:
            done.set()

    worker = threading.Thread(target=query)
    try:
        worker.start()
        assert entered.wait(1)
        # Main thread must progress while native query remains blocked.
        progressed = 0
        end = time.monotonic() + 0.08
        while time.monotonic() < end and not done.is_set():
            progressed += 1
            time.sleep(0.001)
        assert progressed >= 10 and not done.is_set()
        worker.join(2)
        assert not worker.is_alive()
        assert not errors
    finally:
        worker.join(2)
        bus.close()
        os.close(slave)
        os.close(master)
