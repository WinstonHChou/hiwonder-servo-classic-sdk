# SPDX-License-Identifier: MIT
#
# pylx16a -- drop-in compatibility shim for ethanlipson/PyLX-16A
#
# Everything routes into the native HiwonderBus core, so behaviour, validation
# and error types match `from pylx16a import LX16A` in the original library
# while the bytes on the wire come from a C++17 POSIX driver instead of
# pyserial.
#
# Migration contract:
#   * same module-level `LX16A` class with the same static initializer
#   * same snake_case method names and keyword arguments
#   * same exception classes, including the `id_` attribute
#   * same "raise, never clamp" argument policy
#
"""A modern, drop-in replacement for ethanlipson/PyLX-16A.

    from pylx16a import LX16A

    LX16A.initialize("/dev/ttyUSB0")
    servo = LX16A(1)
    servo.move(90.0, 1000)
    print(servo.get_physical_angle())
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from hiwonder_bus import SerialBus, Servo
from hiwonder_bus.exceptions import (
    ServoArgumentError,
    ServoChecksumError,
    ServoError,
    ServoFirmwareError,
    ServoIOError,
    ServoLogicalError,
    ServoTimeoutError,
)

__version__ = "0.1.0"

__all__ = [
    "LX16A",
    "ServoError",
    "ServoTimeoutError",
    "ServoChecksumError",
    "ServoArgumentError",
    "ServoLogicalError",
    "ServoIOError",
    "ServoFirmwareError",
]

# PyLX-16A's documented ranges; kept here so the shim owns the contract.
_ANGLE_RANGE = (0.0, 240.0)
_MOTOR_SPEED_RANGE = (-1000, 1000)
_VIN_LIMIT_RANGE = (4500, 12000)
_TEMP_LIMIT_RANGE = (50, 100)
_ANGLE_OFFSET_RANGE = (-30, 30)
_MAX_MOVE_TIME_MS = 30000


class _BSpline:
    """Cox-de Boor B-spline, as used by ``set_bspline``/``move_bspline``.

    Two deliberate corrections relative to the original PyLX-16A implementation,
    both of which are unreachable-by-accident bugs there:

    * the recursive basis call passed the wrong arity (``weight(knots, i, p-1,
      u)`` against a ``(i, u)`` signature), so any ``degree > 1`` raised
      ``TypeError``; the recursion is now the textbook one;
    * ``sample_x`` returned ``None`` past the last sample, which surfaced as a
      confusing failure inside ``move``. It now clamps to the final sample and
      the curve is sampled over a closed interval.
    """

    def __init__(
        self,
        knots: Sequence[float],
        control_points: Sequence[Tuple[float, float]],
        degree: int,
        num_samples: int,
    ) -> None:
        self.knots = [float(k) for k in knots]
        self.control_points = [tuple(cp) for cp in control_points]
        if not isinstance(degree, int) or not 0 <= degree < len(self.control_points):
            raise ValueError("degree must be an integer smaller than the control point count")
        self.degree = degree
        if not isinstance(num_samples, int) or num_samples < 1:
            raise ValueError("num_samples must be a positive integer")
        if any(len(cp) != 2 or not all(math.isfinite(v) for v in cp)
               for cp in self.control_points):
            raise ValueError("control points must be finite (x, angle) pairs")
        if (not self.knots or not all(math.isfinite(k) for k in self.knots)
                or any(a > b for a, b in zip(self.knots, self.knots[1:]))):
            raise ValueError("knots must be finite and nondecreasing")
        n = len(self.control_points)
        if len(self.knots) == n - degree + 1:
            # The short documented convention omits repeated endpoint knots.
            self.knots = ([self.knots[0]] * degree + self.knots
                          + [self.knots[-1]] * degree)
        if len(self.knots) != n + degree + 1:
            raise ValueError("invalid knot count")
        self.start = self.knots[degree]
        self.end = self.knots[n]
        if not self.start < self.end:
            raise ValueError("knot domain must have positive width")

        self.samples: List[Tuple[float, float]] = []
        for i in range(num_samples + 1):
            u = self.end if i == num_samples else self.start + (self.end - self.start) * i / num_samples
            self.samples.append(self.sample(u))
        if any(a[0] > b[0] for a, b in zip(self.samples, self.samples[1:])):
            raise ValueError("sampled curve x coordinates must be nondecreasing")

    def _basis(self, i: int, degree: int, u: float) -> float:
        # Recursive basis functions may address the padding beyond the controls.
        if i < 0 or i + degree + 1 >= len(self.knots):
            return 0.0
        if degree == 0:
            inside = self.knots[i] <= u < self.knots[i + 1]
            # Evaluate the right endpoint as the left-hand limit. Half-open
            # intervals alone would make every basis zero at the final knot.
            right_end = u == self.end and self.knots[i] < u == self.knots[i + 1]
            return 1.0 if inside or right_end else 0.0

        first_denominator = self.knots[i + degree] - self.knots[i]
        first = (
            0.0
            if first_denominator == 0
            else (u - self.knots[i])
            / first_denominator
            * self._basis(i, degree - 1, u)
        )

        second_denominator = self.knots[i + degree + 1] - self.knots[i + 1]
        second = (
            0.0
            if second_denominator == 0
            else (self.knots[i + degree + 1] - u)
            / second_denominator
            * self._basis(i + 1, degree - 1, u)
        )
        return first + second

    def sample(self, u: float) -> Tuple[float, float]:
        sx = sy = 0.0
        for i, (point_x, point_y) in enumerate(self.control_points):
            weight = self._basis(i, self.degree, u)
            sx += weight * point_x
            sy += weight * point_y
        return (sx, sy)

    def sample_x(self, x: float) -> float:
        for sample in self.samples[:-1]:
            if sample[0] >= x:
                return sample[1]
        return self.samples[-1][1]


class LX16A:
    """The PyLX-16A servo object, backed by the HiwonderBus native core.

    Like the original, the serial handle is a class-level singleton: create the
    bus once with :meth:`initialize`, then build as many servo objects as you
    like.
    """

    _controller: Optional[SerialBus] = None

    # ------------------------------------------------------------------ #
    # Class-level setup
    # ------------------------------------------------------------------ #
    @staticmethod
    def initialize(port: str, timeout: float = 0.02) -> None:
        """Open the bus on ``port`` at 115200 8N1.

        Replaces the previous controller, if any, after flushing it -- same
        contract as PyLX-16A.
        """
        if not isinstance(timeout, (int, float)) or timeout <= 0:
            raise ServoArgumentError("timeout must be a positive number of seconds", None)

        timeout_ms = max(1, int(round(float(timeout) * 1000)))

        if LX16A._controller is not None:
            controller = LX16A._controller
            LX16A._controller = None
            try:
                controller.flush_input()
                controller.flush_output()
            except Exception:  # pragma: no cover - best-effort teardown
                pass
            controller.close()

        LX16A._controller = SerialBus(
            device=port,
            baudrate=115200,
            read_timeout_ms=timeout_ms,
            write_timeout_ms=timeout_ms,
            low_latency=True,
        )

    @staticmethod
    def attach_simulator(ids: Sequence[int]) -> None:
        """Swap in the in-process bus model (addition, not part of the original API).

        Lets CI and the compatibility example exercise the whole documented
        surface with no serial port attached. Call it instead of
        :meth:`initialize`.
        """
        from hiwonder_bus import make_simulated_bus

        controller = LX16A._controller
        LX16A._controller = make_simulated_bus(list(ids))
        if controller is not None:
            try:
                controller.close()
            except Exception:  # pragma: no cover - best-effort teardown
                pass

    @staticmethod
    def set_timeout(seconds: float) -> None:
        """Set the bus read/write timeout, in seconds (PyLX-16A parity)."""
        controller = LX16A._require_controller()
        controller.read_timeout_ms = max(1, int(round(float(seconds) * 1000)))
        controller.write_timeout_ms = max(1, int(round(float(seconds) * 1000)))

    @staticmethod
    def get_timeout() -> float:
        """Current bus timeout in seconds."""
        controller = LX16A._require_controller()
        return controller.read_timeout_ms / 1000.0

    # ------------------------------------------------------------------ #
    # Construction
    # ------------------------------------------------------------------ #
    def __init__(self, id_: int, disable_torque: bool = False) -> None:
        if not isinstance(id_, int) or isinstance(id_, bool):
            raise ServoArgumentError("Servo ID must be an integer", None)
        if id_ < 0 or id_ > 253:
            raise ServoArgumentError("Servo ID must be between 0 and 253 inclusive", id_)

        controller = LX16A._controller
        if controller is None:
            raise ServoLogicalError(
                "the bus is not initialized; call LX16A.initialize(port) first", id_
            )

        self._id = id_
        self._bspline: Optional[_BSpline] = None
        self._servo = Servo(controller, id_)
        # Mirrors PyLX-16A's __init__ exactly: read every register, then honour
        # disable_torque (it enables torque otherwise).
        self._servo.initialize(disable_torque=disable_torque)

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    @staticmethod
    def _require_controller() -> SerialBus:
        controller = LX16A._controller
        if controller is None:
            raise ServoLogicalError(
                "the bus is not initialized; call LX16A.initialize(port) first", None
            )
        return controller

    @property
    def id_(self) -> int:
        """The servo's current id (PyLX-16A exposes ``servo.id_``)."""
        return self._id

    def __repr__(self) -> str:
        return f"LX16A(id_={self._id})"

    # ------------------------------------------------------------------ #
    # Write commands
    # ------------------------------------------------------------------ #
    def move(
        self,
        angle: float,
        time: int = 0,
        relative: bool = False,
        wait: bool = False,
    ) -> None:
        """Rotate to ``angle`` degrees over ``time`` ms.

        ``wait=True`` queues the move (opcode 7) until :meth:`move_start`.
        """
        self._servo.move(angle, time, relative, wait)

    def move_bspline(self, x: float, time: int = 0, wait: bool = False) -> None:
        """Move to the y-value of the B-spline at x-value ``x``."""
        if self._bspline is None:
            raise ServoLogicalError(f"Servo {self._id}: no B-spline defined", self._id)
        self._servo.move(self._bspline.sample_x(x), time, False, wait)

    def move_start(self) -> None:
        """Begin the move queued by ``move(..., wait=True)``."""
        self._servo.move_start()

    def move_stop(self) -> None:
        """Halt the current movement."""
        self._servo.move_stop()

    def set_id(self, id_: int) -> None:
        """Assign a new firmware id; this object follows it."""
        if not isinstance(id_, int) or isinstance(id_, bool):
            raise ServoArgumentError("Servo ID must be an integer", self._id)
        if id_ < 0 or id_ > 253:
            raise ServoArgumentError("Servo ID must be between 0 and 253 inclusive", id_)
        self._servo.set_id(id_)
        self._id = id_

    def set_angle_offset(self, offset: float, permanent: bool = False) -> None:
        """Shift commanded zero by ``offset`` degrees (+/-30)."""
        self._servo.set_angle_offset(offset, permanent)

    def set_angle_limits(self, lower_limit: float, upper_limit: float) -> None:
        """Bound the range of motion (degrees)."""
        self._servo.set_angle_limits(lower_limit, upper_limit)

    def set_vin_limits(self, lower_limit: int, upper_limit: int) -> None:
        """Set the voltage protection window in millivolts (4500..12000)."""
        self._servo.set_vin_limits(lower_limit, upper_limit)

    def set_temp_limit(self, upper_limit: int) -> None:
        """Set the overheat ceiling in deg C (50..100)."""
        self._servo.set_temp_limit(upper_limit)

    def motor_mode(self, speed: int) -> None:
        """Switch to continuous-spin mode at ``speed`` (-1000..1000)."""
        self._servo.set_motor_mode(speed)

    def servo_mode(self) -> None:
        """Switch back to position control."""
        self._servo.set_servo_mode()

    def enable_torque(self) -> None:
        self._servo.enable_torque()

    def disable_torque(self) -> None:
        self._servo.disable_torque()

    def led_power_off(self) -> None:
        self._servo.led_power_off()

    def led_power_on(self) -> None:
        self._servo.led_power_on()

    def set_led_error_triggers(
        self, over_temperature: bool, over_voltage: bool, rotor_locked: bool
    ) -> None:
        """Choose which faults flash the status LED."""
        self._servo.set_led_error_triggers(over_temperature, over_voltage, rotor_locked)

    def set_bspline(
        self,
        knots: List[float],
        control_points: List[Tuple[float, float]],
        degree: int,
        num_samples: int = 100,
    ) -> None:
        """Install the B-spline used by :meth:`move_bspline`.

        The ``len(knots) == len(control_points) - degree + 1`` rule is kept as
        an accepted alternative so existing PyLX-16A call sites do not break, but
        the standard knot-vector convention is accepted too (and is the one that
        actually evaluates to a curve).
        """
        try:
            spline = _BSpline(knots, control_points, degree, num_samples)
        except (ValueError, TypeError, OverflowError) as exc:
            raise ServoArgumentError(f"Servo {self._id}: invalid B-spline: {exc}", self._id) from exc
        self._bspline = spline

    # ------------------------------------------------------------------ #
    # Read commands
    # ------------------------------------------------------------------ #
    def get_last_instant_move_hw(self) -> Tuple[float, int]:
        """(angle, time_ms) of the last ``move(wait=False)`` from the servo."""
        return self._servo.get_last_instant_move()

    def get_last_delayed_move_hw(self) -> Tuple[float, int]:
        """(angle, time_ms) of the last ``move(wait=True)`` from the servo."""
        return self._servo.get_last_delayed_move()

    def get_id(self, poll_hardware: bool = False) -> int:
        return self._servo.get_id(poll_hardware)

    def get_angle_offset(self, poll_hardware: bool = False) -> float:
        return self._servo.get_angle_offset(poll_hardware)

    def get_angle_limits(self, poll_hardware: bool = False) -> Tuple[float, float]:
        return self._servo.get_angle_limits(poll_hardware)

    def get_vin_limits(self, poll_hardware: bool = False) -> Tuple[int, int]:
        return self._servo.get_vin_limits(poll_hardware)

    def get_temp_limit(self, poll_hardware: bool = False) -> int:
        return self._servo.get_temp_limit(poll_hardware)

    def is_motor_mode(self, poll_hardware: bool = False) -> bool:
        return self._servo.is_motor_mode(poll_hardware)

    def get_motor_speed(self, poll_hardware: bool = False) -> int:
        """Current motor speed; raises if the servo is in servo mode."""
        return self._servo.get_motor_speed(poll_hardware)

    def is_torque_enabled(self, poll_hardware: bool = False) -> bool:
        return self._servo.is_torque_enabled(poll_hardware)

    def is_led_power_on(self, poll_hardware: bool = False) -> bool:
        return self._servo.is_led_power_on(poll_hardware)

    def get_led_error_triggers(
        self, poll_hardware: bool = False
    ) -> Tuple[bool, bool, bool]:
        """(over_temperature, over_voltage, rotor_locked)."""
        return self._servo.get_led_error_triggers(poll_hardware)

    def get_temp(self) -> int:
        """Die temperature in deg C."""
        return self._servo.get_temp()

    def get_vin(self) -> int:
        """Supply voltage in millivolts."""
        return self._servo.get_vin()

    def get_physical_angle(self) -> float:
        """Shaft position in degrees, read from the potentiometer."""
        return self._servo.get_physical_angle(True)

    def get_commanded_angle(self) -> float:
        """The angle this object last commanded (mirror, no wire traffic)."""
        return self._servo.get_commanded_angle()

    def get_waiting_angle(self) -> float:
        """The queued target angle; raises if nothing is queued."""
        return self._servo.get_waiting_angle()

    # ------------------------------------------------------------------ #
    # Extras that do not break drop-in behaviour
    # ------------------------------------------------------------------ #
    @property
    def servo(self) -> Servo:
        """The underlying native :class:`hiwonder_bus.Servo`."""
        return self._servo

    @property
    def config(self):
        """The native servo's :class:`hiwonder_bus.ServoConfig`."""
        return self._servo.config

    def __enter__(self) -> "LX16A":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        # Leave the servo wherever the exception stopped it, but stop it if the
        # block raised: a half-executed move is the dangerous case.
        if exc_type is not None:
            try:
                self._servo.move_stop()
            except Exception:  # pragma: no cover - nothing better than to try
                pass
