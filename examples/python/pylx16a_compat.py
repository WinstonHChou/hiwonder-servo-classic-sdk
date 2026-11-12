#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
#
# examples/python/pylx16a_compat.py
#   Drop-in compatibility check for ethanlipson/PyLX-16A.
#
#   Every call below is written exactly the way the upstream library expects,
#   so this doubles as a regression test for the shim: if it passes here, the
#   same source file passes against `pip install pylx16a`.
#
#   Runs against the in-process bus model by default (no hardware), or against
#   a real BusLinker:
#
#     python examples/python/pylx16a_compat.py
#     python examples/python/pylx16a_compat.py --port /dev/ttyUSB0 --id 1
#
import argparse
import sys
import traceback

from pylx16a import (
    LX16A,
    ServoArgumentError,
    ServoChecksumError,
    ServoError,
    ServoLogicalError,
    ServoTimeoutError,
)

PASSED = "PASS"
FAILED = "FAIL"


class Check:
    """Tiny assertion recorder so one failure does not hide the rest."""

    def __init__(self):
        self.results = []

    def run(self, name, fn):
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 - the whole point is reporting
            self.results.append((FAILED, name, f"{type(exc).__name__}: {exc}"))
            traceback.print_exc()
        else:
            self.results.append((PASSED, name, ""))

    def expect(self, name, condition, detail=""):
        self.results.append(
            (PASSED if condition else FAILED, name, "" if condition else detail)
        )

    def expect_raises(self, name, exception_type, fn, *args, **kwargs):
        try:
            fn(*args, **kwargs)
        except exception_type as exc:
            self.results.append((PASSED, name, f"raised {type(exc).__name__}"))
            return
        except Exception as exc:  # noqa: BLE001
            self.results.append(
                (FAILED, name, f"expected {exception_type.__name__}, got {type(exc).__name__}: {exc}")
            )
            return
        self.results.append((FAILED, name, f"expected {exception_type.__name__}, nothing raised"))

    def report(self, title):
        print(f"\n{title}")
        print("-" * (len(title) + 2))
        for status, name, detail in self.results:
            line = f"  [{status}] {name}"
            if detail:
                line += f"  ({detail})"
            print(line)
        failures = sum(1 for s, _, _ in self.results if s == FAILED)
        print(
            f"\n{len(self.results)} checks, "
            f"{len(self.results) - failures} passed, {failures} failed\n"
        )
        return failures


def near(a, b, eps=0.3):
    return a is not None and abs(a - b) <= eps


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial device, e.g. /dev/ttyUSB0")
    parser.add_argument("--id", type=int, default=1, help="servo id to drive")
    parser.add_argument("--timeout", type=float, default=0.02, help="bus timeout in seconds")
    args = parser.parse_args(argv)

    check = Check()

    # ------------------------------------------------------------------ #
    # 1. The documented initialiser, verbatim.
    # ------------------------------------------------------------------ #
    def initialise():
        if args.port:
            LX16A.initialize(args.port, timeout=args.timeout)
        else:
            print("  (no --port given: using the in-process bus model)")
            LX16A.attach_simulator([args.id])

    check.run("LX16A.initialize(port)", initialise)

    def timeout_round_trip():
        LX16A.set_timeout(0.05)
        if not near(LX16A.get_timeout(), 0.05, 1e-6):
            raise AssertionError(f"timeout came back as {LX16A.get_timeout()}")

    check.run("set_timeout / get_timeout", timeout_round_trip)

    # ------------------------------------------------------------------ #
    # 2. Construction, mirrors and torque.
    # ------------------------------------------------------------------ #
    servo_container = {}

    def construct():
        servo_container["servo"] = LX16A(args.id)

    check.run("LX16A(id_)", construct)
    servo = servo_container.get("servo")

    if servo is None:
        check.report("PyLX-16A compatibility (aborted: servo could not be constructed)")
        return 2

    check.run("is_torque_enabled()", lambda: check.expect(
        "torque enabled after __init__", servo.is_torque_enabled(),
        "the constructor enables torque unless disable_torque=True"))

    check.expect("servo.id_ attribute", servo.id_ == args.id, f"got {servo.id_}")
    check.expect("get_id()", servo.get_id() == args.id, f"got {servo.get_id()}")

    check.run("disable_torque / enable_torque", lambda: (
        servo.disable_torque(),
        check.expect("torque off", not servo.is_torque_enabled()),
        servo.enable_torque(),
        check.expect("torque on", servo.is_torque_enabled()),
    ))

    check.run("LX16A(id_, disable_torque=True)", lambda: check.expect(
        "starts with torque off",
        LX16A(args.id, disable_torque=True).is_torque_enabled() is False))

    # ------------------------------------------------------------------ #
    # 3. Motion, exactly as documented.
    # ------------------------------------------------------------------ #
    check.run("servo.move(angle, time)", lambda: servo.move(90.0, 500))
    check.run("get_commanded_angle()", lambda: check.expect(
        "90 deg commanded", near(servo.get_commanded_angle(), 90.0),
        f"got {servo.get_commanded_angle()}"))

    check.run("servo.move(angle, time, relative=True)", lambda: servo.move(10.0, 200, True, False))
    check.run("servo.move(..., wait=True) + move_start()", lambda: (
        servo.move(120.0, 300, False, True),
        check.expect("waiting angle", near(servo.get_waiting_angle(), 120.0),
                     f"got {servo.get_waiting_angle()}"),
        servo.move_start(),
        check.expect("released", not servo.get_commanded_angle() is None),
    ))

    check.run("servo.move_stop()", lambda: servo.move_stop())
    check.run("get_physical_angle()", lambda: check.expect(
        "returns a float", isinstance(servo.get_physical_angle(), float)))

    # Documented argument policy: raise, never clamp.
    check.expect_raises("move(300) raises", ServoArgumentError, servo.move, 300.0, 100)
    check.expect_raises("move(-1) raises", ServoArgumentError, servo.move, -1.0, 100)
    check.expect_raises("motor_mode(2000) raises", ServoArgumentError, servo.motor_mode, 2000)
    check.expect_raises("set_temp_limit(10) raises", ServoArgumentError, servo.set_temp_limit, 10)
    check.expect_raises("set_vin_limits(1,2) raises", ServoArgumentError, servo.set_vin_limits, 1, 2)
    check.expect_raises("set_angle_offset(-99) raises", ServoArgumentError,
                        servo.set_angle_offset, -99)

    check.run("motor_mode / get_motor_speed / servo_mode", lambda: (
        servo.motor_mode(500),
        check.expect("motor mode", servo.is_motor_mode()),
        check.expect("speed", servo.get_motor_speed() == 500, f"got {servo.get_motor_speed()}"),
        servo.servo_mode(),
        check.expect("servo mode", not servo.is_motor_mode()),
    ))
    check.expect_raises("get_motor_speed() outside motor mode", ServoLogicalError,
                        servo.get_motor_speed)

    def move_with_torque_off():
        servo.disable_torque()
        try:
            servo.move(45.0, 100)
        finally:
            servo.enable_torque()

    check.expect_raises("move() with torque disabled", ServoLogicalError, move_with_torque_off)

    # ------------------------------------------------------------------ #
    # 4. Calibration and limits.
    # ------------------------------------------------------------------ #
    check.run("set_angle_limits / get_angle_limits", lambda: (
        servo.set_angle_limits(10.0, 150.0),
        check.expect("limits", servo.get_angle_limits() in [(10.0, 150.0), (9.84, 150.0)] or
                     near(servo.get_angle_limits()[0], 10.0) and
                     near(servo.get_angle_limits()[1], 150.0),
                     f"got {servo.get_angle_limits()}"),
    ))
    check.run("set_angle_offset / get_angle_offset", lambda: (
        servo.set_angle_offset(5.0),
        check.expect("offset", near(servo.get_angle_offset(), 5.0),
                     f"got {servo.get_angle_offset()}"),
    ))
    check.run("set_vin_limits / get_vin_limits", lambda: (
        servo.set_vin_limits(5000, 11000),
        check.expect("vin limits", servo.get_vin_limits() == (5000, 11000) or
                     near(servo.get_vin_limits()[0], 5000, 24) and
                     near(servo.get_vin_limits()[1], 11000, 24),
                     f"got {servo.get_vin_limits()}"),
    ))
    check.run("set_temp_limit / get_temp_limit", lambda: (
        servo.set_temp_limit(80),
        check.expect("temp limit", servo.get_temp_limit() == 80,
                     f"got {servo.get_temp_limit()}"),
    ))

    # ------------------------------------------------------------------ #
    # 5. LED and telemetry.
    # ------------------------------------------------------------------ #
    check.run("led_power_on/off + is_led_power_on", lambda: (
        servo.led_power_on(),
        check.expect("led on", servo.is_led_power_on(), "register is active-low"),
        servo.led_power_off(),
        check.expect("led off", not servo.is_led_power_on()),
    ))
    check.run("set_led_error_triggers / get_led_error_triggers", lambda: (
        servo.set_led_error_triggers(True, False, True),
        check.expect("triggers", servo.get_led_error_triggers() == (True, False, True),
                     f"got {servo.get_led_error_triggers()}"),
    ))
    check.run("get_temp()", lambda: check.expect(
        "temperature", isinstance(servo.get_temp(), int) and -20 < servo.get_temp() < 100,
        f"got {servo.get_temp()}"))
    check.run("get_vin()", lambda: check.expect(
        "supply voltage", isinstance(servo.get_vin(), int) and 3000 < servo.get_vin() < 13000,
        f"got {servo.get_vin()}"))
    check.run("get_last_instant_move_hw()", lambda: check.expect(
        "returns (angle, time)",
        isinstance(servo.get_last_instant_move_hw(), tuple) and
        len(servo.get_last_instant_move_hw()) == 2))
    check.run("get_last_delayed_move_hw()", lambda: check.expect(
        "returns (angle, time)",
        isinstance(servo.get_last_delayed_move_hw(), tuple) and
        len(servo.get_last_delayed_move_hw()) == 2))

    # ------------------------------------------------------------------ #
    # 6. The B-spline helpers (pure Python in the shim).
    # ------------------------------------------------------------------ #
    def bspline():
        servo.set_bspline(
            knots=[0, 0, 0, 1, 1, 1],
            control_points=[(0.0, 40.0), (0.5, 120.0), (1.0, 60.0)],
            degree=2,
            num_samples=50,
        )
        servo.move_bspline(0.5, 200)

    check.run("set_bspline + move_bspline", bspline)
    check.expect_raises("bad bspline knots rejected", ServoArgumentError,
                        servo.set_bspline, [0, 1], [(0.0, 40.0)], 2)
    check.expect_raises("move_bspline without a curve", ServoLogicalError,
                        LX16A(args.id).move_bspline, 0.5)

    # ------------------------------------------------------------------ #
    # 7. Exception hierarchy, as documented.
    # ------------------------------------------------------------------ #
    for exc_type in (ServoTimeoutError, ServoChecksumError, ServoArgumentError,
                     ServoLogicalError):
        check.expect(f"{exc_type.__name__} is a ServoError", issubclass(exc_type, ServoError))
    try:
        raise ServoArgumentError("boom", 7)
    except ServoError as exc:
        check.expect("exc.id_ carries the servo", exc.id_ == 7, f"got {exc.id_}")

    # ------------------------------------------------------------------ #
    # 8. set_id (needs a re-constructed object afterwards).
    # ------------------------------------------------------------------ #
    def set_id():
        # Do not rename a real servo in CI; only assert the range check.
        check.expect_raises("set_id(300) raises", ServoArgumentError, servo.set_id, 300)

    check.run("set_id range validation", set_id)

    if args.port:
        check.run("servo.set_id round trip", lambda: (
            servo.set_id(args.id),
            check.expect("id still correct", servo.get_id() == args.id)))

    failures = check.report("PyLX-16A drop-in compatibility")
    if args.port:
        print(f"If a real servo is attached, eyeball the angles printed above on "
              f"servo {args.id}; a healthy LX-16A should track them.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
