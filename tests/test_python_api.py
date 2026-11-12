# SPDX-License-Identifier: MIT
#
# tests/test_python_api.py
#
# Python-side tests for the hiwonder_bus / pylx16a packages, run against the
# in-process servo model. Together with tests/test_protocol.cpp this is the CI
# gate: nothing here needs a serial port.
#
import threading

import pytest

import hiwonder_bus
import pylx16a
from hiwonder_bus import (
    BusConfig,
    ErrorCode,
    ErrorMode,
    Servo,
    ServoArgumentError,
    ServoError,
    ServoLogicalError,
    ServoTimeoutError,
    angle_to_ticks,
    checksum,
    make_simulated_bus,
    move_sync,
    parse_response,
    read_angles,
    read_telemetry,
    set_torque,
    simulate_disconnect,
    ticks_to_angle,
)


@pytest.fixture()
def bus():
    handle = make_simulated_bus([1, 2, 3, 4, 5])
    yield handle
    handle.close()


@pytest.fixture()
def pylx():
    """The ``pylx16a`` module with a clean, unconnected bus."""
    pylx16a.LX16A._controller = None
    yield pylx16a
    controller = pylx16a.LX16A._controller
    pylx16a.LX16A._controller = None
    if controller is not None:
        controller.close()


# --------------------------------------------------------------------------- #
# Module surface
# --------------------------------------------------------------------------- #

def test_native_module_is_importable_and_named():
    from hiwonder_bus import _hiwonder_bus

    assert _hiwonder_bus.__version__ == hiwonder_bus.__version__
    assert "HiwonderBus" in hiwonder_bus.version()


def test_cpp_aliases_are_the_same_classes():
    assert hiwonder_bus.LX16A is hiwonder_bus.Servo
    assert hiwonder_bus.LX16ABus is hiwonder_bus.SerialBus
    assert hiwonder_bus.Bus is hiwonder_bus.SerialBus
    assert isinstance(hiwonder_bus.Servo(make_simulated_bus([9]), 9), hiwonder_bus.LX16A)


def test_all_exports_resolve():
    for name in hiwonder_bus.__all__:
        assert hasattr(hiwonder_bus, name), f"__all__ advertises a missing name: {name}"


# --------------------------------------------------------------------------- #
# Protocol helpers
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("degrees,ticks", [(0.0, 0), (90.0, 375), (240.0, 1000)])
def test_classic_tick_geometry(degrees, ticks):
    assert angle_to_ticks(degrees) == ticks
    assert ticks_to_angle(ticks) == pytest.approx(degrees, abs=0.25)


def test_non_default_geometry_is_honoured():
    wide = hiwonder_bus.ServoConfig(max_ticks=2000, min_angle_deg=0.0, max_angle_deg=300.0)
    assert angle_to_ticks(150.0, wide) == 1000
    assert wide.deg_per_tick() == pytest.approx(0.15)


def test_checksum_and_frame_builders():
    frame = hiwonder_bus.build_packet(1, 28)          # read physical angle
    assert frame[:2] == [0x55, 0x55]
    assert frame[2] == 1                              # id
    assert frame[3] == 3                              # length covers id..cmd
    assert frame[-1] == checksum(frame[:-1])
    result, servo_id, status, data, ok = parse_response(frame)
    assert ok and servo_id == 1 and data == []


def test_parse_response_flags_a_corrupt_frame():
    frame = hiwonder_bus.build_packet(7, 26)          # read temp
    frame[-1] ^= 0xFF
    result, _id, _status, _data, ok = parse_response(frame)
    assert not ok and result != 0


# --------------------------------------------------------------------------- #
# Bus / servo behaviour
# --------------------------------------------------------------------------- #

def test_bus_config_round_trips():
    bus = make_simulated_bus([1])
    config = BusConfig()
    config.device = "simulator"
    config.baudrate = 115200
    config.read_timeout_ms = 33
    config.max_retries = 2
    config.strict_status = True
    assert bus.update_config(config)

    # The knobs the transport honours are readable and writable.
    assert bus.read_timeout_ms == 33
    bus.read_timeout_ms = 12
    assert bus.read_timeout_ms == 12
    assert bus.write_timeout_ms > 0
    bus.reset_stats()
    assert bus.stats().writes == 0
    bus.close()


def test_scan_and_read_round_trip(bus):
    assert bus.scan_ids(0, 10) == [1, 2, 3, 4, 5]
    angles = bus.read_angles([1, 2, 3])
    assert len(angles) == 3
    assert all(isinstance(a, float) for a in angles)


def test_servo_move_updates_the_mirror(bus):
    servo = Servo(bus, 2)
    assert not servo.is_torque_enabled()
    servo.enable_torque()
    assert servo.is_torque_enabled()

    servo.move(90.0, 250)
    assert servo.get_commanded_angle() == pytest.approx(90.0, abs=0.25)
    assert servo.get_physical_angle(poll_hardware=True) == pytest.approx(90.0, abs=0.25)

    servo.disable_torque()
    with pytest.raises(ServoLogicalError):
        servo.move(45.0, 100)


def test_validation_never_clamps(bus):
    servo = Servo(bus, 1)
    servo.enable_torque()
    with pytest.raises(ServoArgumentError):
        servo.move(999.0, 100)
    with pytest.raises(ServoArgumentError):
        servo.set_temp_limit(10)
    with pytest.raises(ServoArgumentError):
        servo.set_motor_mode(5000)
    with pytest.raises(ServoArgumentError):
        Servo(bus, 254)


def test_exceptions_carry_the_servo_id(bus):
    simulate_disconnect(bus, 3)
    servo = Servo(bus, 3)
    with pytest.raises(ServoTimeoutError) as excinfo:
        servo.get_temp()
    assert excinfo.value.id_ == 3
    assert isinstance(excinfo.value, ServoError)


def test_non_throwing_mode_survives_a_dead_servo():
    bus = make_simulated_bus([1, 2, 3])
    bus.error_mode = ErrorMode.NON_THROWING
    simulate_disconnect(bus, 2)

    good = Servo(bus, 1)
    good.enable_torque()
    good.move(60.0, 100)
    assert good.healthy()

    dead = Servo(bus, 2)
    # No exception, and the failure is inspectable.
    assert dead.get_temp() == -1
    assert dead.last_error() == ErrorCode.TIMEOUT
    assert "timeout" in dead.last_message().lower()

    sweep = bus.read_angles([1, 2, 3])
    assert sweep[0] == pytest.approx(60.0, abs=0.25)
    assert sweep[1] != sweep[1]  # NaN
    assert sweep[2] == sweep[2]  # not NaN
    bus.close()


# --------------------------------------------------------------------------- #
# Batch API
# --------------------------------------------------------------------------- #

def test_move_sync_coalesces_into_one_write(bus):
    servos = [Servo(bus, i) for i in (1, 2, 3, 4, 5)]
    set_torque(bus, servos, True)
    bus.reset_stats()

    written = move_sync(bus, servos, [30.0] * 5, time_ms=200, start=True)
    stats = bus.stats()

    # five 10-byte targets + the 6-byte broadcast, one syscall.
    assert written == 5 * 10 + 6
    assert stats.writes == 1
    for servo, angle in zip(servos, [30.0] * 5):
        assert servo.get_commanded_angle() == pytest.approx(angle, abs=0.25)


def test_batch_read_reports_failures_without_raising():
    bus = make_simulated_bus([1, 2, 3])
    simulate_disconnect(bus, 2)
    angles = read_angles(bus, [Servo(bus, i) for i in (1, 2, 3)])
    assert angles[1] != angles[1]          # NaN, index still aligned
    assert angles[0] == angles[0]
    telemetry = read_telemetry(bus, [Servo(bus, i) for i in (1, 2, 3)])
    assert not telemetry[1].valid and telemetry[0].valid
    bus.close()


def test_rejects_mismatched_batch_lengths(bus):
    with pytest.raises(ServoArgumentError):
        bus.move_sync([1, 2], [10.0], time_ms=0)
    with pytest.raises(ServoArgumentError):
        bus.move_sync([1, 2], [999.0, 10.0], time_ms=0)


# --------------------------------------------------------------------------- #
# Concurrency: the point of releasing the GIL
# --------------------------------------------------------------------------- #

def test_two_threads_share_the_bus_without_deadlock():
    bus = make_simulated_bus([1, 2, 3, 4, 5, 6])
    bus.error_mode = ErrorMode.NON_THROWING
    results = []
    errors = []
    barrier = threading.Barrier(2, timeout=5)

    def worker(offset):
        try:
            # Each worker owns its own mirrors, sharing only the bus.
            servos = [Servo(bus, i) for i in (1, 2, 3)]
            set_torque(bus, servos, True)
            barrier.wait()
            for _ in range(30):
                assert move_sync(bus, servos, [float(40 + offset)] * 3, time_ms=10, start=True) > 0
                snapshot = read_telemetry(bus, servos)
                results.append(len(snapshot) == 3 and all(s.valid for s in snapshot))
        except BaseException as error:
            errors.append(error)

    threads = [threading.Thread(target=worker, args=(a * 10,)) for a in (0, 1)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=30)
        assert not thread.is_alive()

    assert not errors
    assert len(results) == 60 and all(results)
    bus.close()


# --------------------------------------------------------------------------- #
# pylx16a drop-in contract
# --------------------------------------------------------------------------- #

def test_pylx16a_documented_flow(pylx):
    LX16A = pylx.LX16A
    LX16A.attach_simulator([1, 2, 3])
    assert LX16A.get_timeout() > 0
    LX16A.set_timeout(0.05)
    assert LX16A.get_timeout() == pytest.approx(0.05)

    servo = LX16A(1)
    assert servo.id_ == 1
    assert servo.is_torque_enabled()

    servo.move(90.0, 500)
    assert servo.get_commanded_angle() == pytest.approx(90.0, abs=0.25)

    servo.move(10.0, 0, relative=True, wait=True)
    assert servo.get_waiting_angle() == pytest.approx(100.0, abs=0.5)
    servo.move_start()

    servo.set_angle_limits(0.0, 240.0)
    assert servo.get_angle_limits()[1] == pytest.approx(240.0, abs=0.25)
    servo.set_temp_limit(80)
    assert servo.get_temp_limit() == 80
    servo.motor_mode(-500)
    assert servo.is_motor_mode() and servo.get_motor_speed() == -500
    servo.servo_mode()
    assert not servo.is_motor_mode()
    assert isinstance(servo.get_physical_angle(), float)
    assert 3000 < servo.get_vin() < 13000


def test_pylx16a_initialize_reports_a_bad_port(pylx):
    from hiwonder_bus.exceptions import ServoIOError

    with pytest.raises((ServoIOError, RuntimeError, OSError)):
        pylx.LX16A.initialize("/definitely/not/a/tty/device")


def test_pylx16a_exception_hierarchy_is_pyserials():
    for cls in (
        pylx16a.ServoTimeoutError,
        pylx16a.ServoChecksumError,
        pylx16a.ServoArgumentError,
        pylx16a.ServoLogicalError,
    ):
        assert issubclass(cls, ServoError)
    with pytest.raises(ServoError):
        raise pylx16a.ServoArgumentError("nope", 5)


def test_pylx16a_requires_an_initialized_bus(pylx):
    with pytest.raises(ServoLogicalError):
        pylx.LX16A(1)


def test_pylx16a_static_controller_is_shared(pylx):
    pylx.LX16A.attach_simulator([1, 2])
    first = pylx.LX16A(1)
    second = pylx.LX16A(2)
    assert first.servo.bus() is second.servo.bus()
    assert pylx.LX16A._controller is not None


def test_pylx16a_rejects_bad_arguments(pylx):
    pylx.LX16A.attach_simulator([2, 3])
    servo = pylx.LX16A(2)
    with pytest.raises(ServoArgumentError):
        servo.move(300.0, 100)
    with pytest.raises(ServoArgumentError):
        pylx.LX16A(999)
    with pytest.raises(ServoLogicalError):
        servo.get_motor_speed()


def test_pylx16a_bspline_and_context_manager(pylx):
    pylx.LX16A.attach_simulator([3])

    with pylx.LX16A(3) as servo:
        servo.set_bspline(
            [0, 0, 0, 1, 1, 1], [(0.0, 40.0), (0.5, 120.0), (1.0, 60.0)], 2, 50
        )
        servo.move_bspline(0.5, 100)
    with pytest.raises(ServoArgumentError):
        servo.set_bspline([0, 1], [(0.0, 40.0)], 2)
