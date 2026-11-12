# SPDX-License-Identifier: MIT
"""Static consumer smoke check: mypy --strict tests/typing_smoke.py."""
from hiwonder_bus import (
    ErrorCode,
    ErrorMode,
    LX16A,
    LX16ABus,
    Servo,
    Telemetry,
    make_simulated_bus,
    move_sync,
    read_telemetry,
    set_torque,
)

bus: LX16ABus = make_simulated_bus([1])
bus.error_mode = ErrorMode.STRICT
servo: LX16A = Servo(bus, 1)
set_torque(bus, [servo], True)
written: int = move_sync(bus, [servo], [90.0], time_ms=100)
snapshots: list[Telemetry] = read_telemetry(bus, [servo])
result: tuple[bool, ErrorCode] = servo.try_move(90.0)
assert written > 0 and snapshots[0].valid and result[0]
