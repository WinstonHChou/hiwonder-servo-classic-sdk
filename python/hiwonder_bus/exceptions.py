# SPDX-License-Identifier: MIT
#
# hiwonder_bus.exceptions
#
# The exception hierarchy, matching ethanlipson/PyLX-16A one for one so that
# existing code keeping `except ServoError` (and reading `exc.id_`) works
# unchanged. The native module raises these same classes from C++ -- see
# `raiseFrom()` in python/bindings.cpp -- so there is exactly one definition.
#
"""Exceptions raised by the HiwonderBus SDK.

All servo-attributable failures carry the offending id in ``id_``::

    try:
        servo.move(300.0)
    except ServoArgumentError as exc:
        print(exc.id_, str(exc))

The four PyLX-16A names are the documented surface; ``ServoIOError`` and
``ServoFirmwareError`` are additions that inherit from :class:`ServoError`, so
catching the base class still covers everything.
"""

from __future__ import annotations

__all__ = [
    "ServoError",
    "ServoTimeoutError",
    "ServoChecksumError",
    "ServoArgumentError",
    "ServoLogicalError",
    "ServoIOError",
    "ServoFirmwareError",
]


class ServoError(Exception):
    """Base class for every failure this SDK reports.

    Parameters
    ----------
    message:
        Human-readable description, already prefixed with ``"Servo <id>: "``
        when the id is known.
    id_:
        The servo the failure came from, or ``None`` for bus-level errors.
    """

    def __init__(self, message: str, id_: int | None = None) -> None:  # noqa: A002 - PyLX parity
        super().__init__(message)
        self.id_ = id_
        self.message = message

    def __str__(self) -> str:
        return self.message

    def __repr__(self) -> str:
        cls = type(self).__name__
        if self.id_ is None:
            return f"{cls}({self.message!r})"
        return f"{cls}({self.message!r}, id_={self.id_!r})"


class ServoTimeoutError(ServoError):
    """The servo did not answer before the read deadline elapsed.

    Mirrors PyLX-16A, which raises this both when nothing arrives and when a
    partial packet arrives.
    """


class ServoChecksumError(ServoError):
    """A reply arrived but its inverted checksum did not match.

    Typically line noise, a bad ground, two devices driving the bus at once, or
    a servo that reset mid-transaction.
    """


class ServoArgumentError(ServoError):
    """An argument was outside the range the firmware accepts.

    Nothing is clamped: out-of-range values raise, exactly as in PyLX-16A, so a
    typo can never quietly command a servo somewhere unexpected.
    """


class ServoLogicalError(ServoError):
    """The command was illegal given the servo's current state.

    For example ``move()`` while torque is disabled, ``get_motor_speed()`` while
    in servo mode, or ``move_start()`` with nothing queued.
    """


class ServoIOError(ServoError):
    """The kernel rejected a serial syscall (open/tcsetattr/write/poll)."""


class ServoFirmwareError(ServoError):
    """Reserved compatibility exception, not emitted by the classic protocol.

    Classic replies echo the command opcode; there is no firmware status byte.
    """
