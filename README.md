# hiwonder-servo-classic-sdk

High-performance Linux **C++17** SDK and **pybind11** Python wrapper for the
Hiwonder / LewanSoul **Classic Serial Bus Servo** family — `LX-16A`, `LX-15D`,
`LX-224`, `LX-225`, `HTS-35H`, `HV-206` and every other part that speaks the
classic `0x55 0x55` half-duplex protocol (`0–1000 ticks` = `0.0°–240.0°`).

The library is a native POSIX driver rather than a thin Python-serial shim: raw
`termios` at 115200 8N1, `poll(2)`-driven reads that wake the moment a reply
lands, `ASYNC_LOW_LATENCY` on USB-TTL BusLinkers, and multi-servo commands
coalesced into a **single `write(2)`**.

```cpp
#include <HiwonderBus/HiwonderBus.hpp>
using namespace HiwonderBus;

SerialBus bus;
bus.open("/dev/ttyUSB0");

LX16A servo(bus, 1);           // HiwonderBus::Servo, with the LX16A alias
servo.initialize();
servo.move(90.0f, 500);        // 90 degrees over 500 ms
```

```python
from hiwonder_bus import SerialBus, Servo, move_sync

bus = SerialBus("/dev/ttyUSB0")
targets = [Servo(bus, i) for i in range(1, 11)]
move_sync(bus, targets, [90.0] * 10, time_ms=800)   # 10 moves + broadcast start
                                                     # in ONE write(2)
```

---

## Table of contents

- [Why this exists](#why-this-exists)
- [Install](#install)
- [C++ API](#c-api)
- [Python API](#python-api)
  - [Generalized `hiwonder_bus`](#generalized-hiwonder_bus)
  - [Drop-in `pylx16a`](#drop-in-pylx16a)
  - [Batch operations](#batch-operations)
- [Testing without hardware](#testing-without-hardware)
- [Using from CMake](#using-from-cmake)
- [How the wire works](#how-the-wire-works)
- [Architecture](#architecture)
- [Building](#building)
- [Hardware notes](#hardware-notes)
- [Acknowledgements & licence](#acknowledgements--licence)

---

## Why this exists

Two reference implementations dominate this family:

| | [alecxcode/LX16A-bus](https://github.com/alecxcode/LX16A-bus) | [ethanlipson/PyLX-16A](https://github.com/ethanlipson/PyLX-16A) |
| --- | --- | --- |
| Language | C++ / Arduino | Python + pyserial |
| Angles | degrees (`0.0`–`240.0`) | degrees (`0.0`–`240.0`) |
| Failure model | silent (returns `0`/`-1`) | raises typed exceptions |
| Transport | `HardwareSerial` | `serial.Serial` (Python) |
| Batching | none | none |
| Latency | `millis()` spin loop | blocking `read()` |

This SDK keeps the **clean degree-based C++ API** of the first and the
**typed-exception, snake_case Python surface** of the second, and fixes what
both leave on the table:

- **Native Linux I/O.** `cfmakeraw`, `poll(2)` with a real deadline, `tcflush`
  before queries, no spin-waiting and no `sleep()` in the hot path.
- **USB latency.** `ASYNC_LOW_LATENCY` via `TIOCGSERIAL`/`TIOCSSERIAL` removes
  the 16 ms receive timer that otherwise dominates every reply on FTDI/CH340
  BusLinkers.
- **One syscall per batch.** A 10-joint synchronized move is N `MOVE_WAIT`
  frames plus the `0xFE` broadcast, concatenated and written once — which is
  what makes the joints *actually* start together on a half-duplex bus.
- **Two failure modes.** Strict (raise) **or** non-throwing real-time, so one
  disconnected servo on a biped cannot unwind a control loop.
- **Generalization.** Nothing in the core is LX-16A-specific. `ServoConfig`
  carries the tick geometry (`1000` ticks / `240.0°` / `0.24 deg/tick` by
  default) so a variant with a different sweep is one line, not a fork.
- **Migration is free.** `using LX16A = Servo; using LX16ABus = SerialBus;` —
  and a `pylx16a` package that is a genuine drop-in.

---

## Install

### Python (wheel / editable)

```bash
git clone https://github.com/hiwonder-servo-classic/hiwonder-servo-classic-sdk
cd hiwonder-servo-classic-sdk
pip install -e .          # scikit-build-core drives CMake; pybind11 comes from build-system
```

`pip install -e .` builds `_hiwonder_bus` as a CMake target and installs both
pure-Python packages, so you get `hiwonder_bus` (generalized) and `pylx16a`
(compat) in one wheel. Requires a C++17 compiler and CMake ≥ 3.16.

### C++ library

```bash
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build            # runs the protocol suite + per-header checks
sudo cmake --install build        # /usr/local by default
```

Zero runtime dependencies beyond libc/libstdc++ and the POSIX headers
(`<termios.h>`, `<poll.h>`, `<fcntl.h>`, `<unistd.h>`, `<sys/ioctl.h>`).

---

## C++ API

Everything lives in the `HiwonderBus` namespace.

### Types

| Header | Contents |
| --- | --- |
| `HiwonderBus/protocol.hpp` | opcodes, constants, `Packet`, `Response`, `tryParseFrame()`, `ServoConfig`, tick conversions, checksum |
| `HiwonderBus/exceptions.hpp` | `ServoError` + `ServoTimeoutError`, `ServoChecksumError`, `ServoArgumentError`, `ServoLogicalError`, `ServoIOError`, `ServoFirmwareError`, `ErrorCode` |
| `HiwonderBus/serial_bus.hpp` | `Transport`, `PosixTransport`, `BusConfig`, `BusStats`, `ErrorMode`, `SerialBus` |
| `HiwonderBus/servo.hpp` | `Servo`, `Telemetry`, aliases (`LX16A`, `LX15D`, `LX224`, `LX225`, `HTS35H`, `HV206`, `LX16ABus`, `Bus`, ...) |
| `HiwonderBus/HiwonderBus.hpp` | umbrella; also `versionString()` and the version macros |
| `HiwonderBus/testing/simulator.hpp` | in-process servo model (`testing::SimulatedTransport`) |

### `SerialBus`

```cpp
BusConfig config;
config.device = "/dev/ttyUSB0";
config.baudrate = 115200;
config.read_timeout_ms = 20;
config.enable_low_latency = true;   // TIOCSSERIAL(ASYNC_LOW_LATENCY)
config.flush_before_query = true;   // tcflush(TCIFLUSH) before each query
config.reject_tx_echo = true;       // ignore frames that mirror our own TX
config.error_mode = ErrorMode::kStrict;

SerialBus bus(config);
bus.open("/dev/ttyUSB0");
```

| Group | Methods |
| --- | --- |
| lifecycle | `open`, `tryOpen`, `close`, `isOpen`, `applyConfig`, `device`, `transportName`, `lastHint` |
| policy | `setReadTimeoutMs`, `readTimeoutMs`, `setWriteTimeoutMs`, `setErrorMode`, `errorMode`, `setPollHardware` |
| diagnostics | `lastError`, `lastErrorCode`, `clearLastError`, `stats`, `resetStats`, `internalTransport` |
| raw frames | `write`, `tryWrite`, `writeFrames`, `tryWriteFrames`, `command`, `tryCommand`, `concatenate` |
| transactions | `transact`, `tryTransact`, `flushInput`, `flushOutput` |
| batch | `queueMoves`, `broadcastMoveStart`, `broadcastMoveStop`, `setTorqueBatch`, `tryReadMany`, `readMany`, `scanIds`, `tryScanIds` |

### `Servo`

Mirrors `alecxcode/LX16A-bus`, extended to the full register set.

| Group | Methods |
| --- | --- |
| identity | `initialize`, `refresh`, `getID`, `setID`, `id`, `config`, `setConfig`, `bus` |
| motion | `move`, `tryMove`, `moveTo`, `moveRelative`, `moveStart`, `tryMoveStart`, `moveStop`, `tryMoveStop`, `getPhysicalAngle`, `tryGetPhysicalAngle`, `getCommandedAngle`, `getWaitingAngle`, `isWaitingForMove`, `getLastInstantMove`, `getLastDelayedMove` |
| torque / mode | `enableTorque`, `disableTorque`, `trySetTorque`, `isTorqueEnabled`, `setMotorMode`, `setServoMode`, `isMotorMode`, `getMotorSpeed` |
| calibration | `setAngleOffset`, `getAngleOffset`, `setAngleLimits`, `getAngleLimits`, `setVinLimits`, `getVinLimits`, `setTempLimit`, `getTempLimit` |
| telemetry | `getTemp`, `getVin`, `readTelemetry` |
| LED | `ledPowerOn`, `ledPowerOff`, `isLedPowerOn`, `setLedErrorTriggers`, `getLedErrorTriggers` |
| raw | `tryTransact`, `command`, `angleToTicks`, `ticksToAngle`, `healthy`, `lastError`, `lastMessage` |
| batch (static) | `moveSync`, `readAngles`, `readTelemetry`, `setTorque`, `setAngleLimits` |

Two ways to say the same thing, depending on taste:

```cpp
servo.move(90.0f, 500, /*wait=*/true);           // C++ LX16A-bus order
servo.moveTo(90.0f, 500, Servo::MoveMode::kWait); // type-safe
servo.moveRelative(-10.0f, 500);                // signed delta, validated final target
```

Cached state getters take `poll_hardware`; telemetry getters always query.
`false` (the default for state flags) is a
pure cache hit — **no wire traffic** — which is what makes the real-time mode
work:

```cpp
bus.setErrorMode(ErrorMode::kNonThrowing);
while (running) {
    float a = servo.getPhysicalAngle(false);   // never blocks, never throws
    servo.moveTo(target, period_ms, Servo::MoveMode::kInstant);
}
```

### Errors

`ErrorMode::kStrict` (default) throws the matching exception, exactly like
PyLX-16A:

| Exception | Raised when |
| --- | --- |
| `ServoTimeoutError` | nothing arrived before the read deadline |
| `ServoChecksumError` | a reply arrived with a bad checksum |
| `ServoArgumentError` | out-of-range angle/id/time/speed/limit (never clamped) |
| `ServoLogicalError` | command illegal in the current mode (torque off, motor mode, nothing queued) |
| `ServoIOError` | the kernel rejected `open`/`tcsetattr`/`write`/`poll` |
| `ServoFirmwareError` | reserved compatibility exception; not emitted by this classic protocol |

`ErrorMode::kNonThrowing` records hardware failures as an `ErrorCode` and
keeps going; argument/state errors still raise. The `tryX()` entry points are
exception-free in **both** modes.

Classic replies echo the command opcode; they contain no firmware status byte.
Responses must match the outstanding unicast ID and opcode. `strict_status` is
a deprecated, ignored compatibility setting.

Bus configuration and diagnostics are locked snapshots. Share a `SerialBus`
across threads, but do not access the same mutable `Servo` object concurrently.
Python servos retain their bus. Native C++ callers must keep the bus alive.
POSIX I/O uses nonblocking descriptors with absolute deadlines; a successful
write means bytes were accepted by the kernel, not physically executed.
Wheels always link the SDK statically, including when `BUILD_SHARED_LIBS=ON`;
normal CMake builds still support shared libraries.

---

## Python API

### Generalized `hiwonder_bus`

The Python mirror of the C++ types, with `snake_case` and keyword arguments.

```python
from hiwonder_bus import SerialBus, Servo, ServoConfig, ErrorMode, Telemetry

bus = SerialBus("/dev/ttyUSB0", read_timeout_ms=20, low_latency=True)
bus.error_mode = ErrorMode.NON_THROWING

# A part with a non-standard sweep is still just a config value.
wide = ServoConfig(max_ticks=2000, min_angle_deg=0.0, max_angle_deg=300.0)
servo = Servo(bus, 1, config=wide)
servo.initialize(disable_torque=False)
servo.move(150.0, time=600, relative=False, wait=False)

print(servo.get_commanded_angle())        # cache, no wire traffic
print(servo.get_physical_angle(poll_hardware=True))
print(bus.read_angles([1, 2, 3]))         # NaN where a servo dropped out
print(bus.stats())                        # tx_bytes, writes, timeouts, echo_drops, ...
```

Also exported: `BusConfig`, `BusStats`, `LedTriggers`, `Command`, `ErrorCode`,
`connect()`, `version()`, `angle_to_ticks()`, `ticks_to_angle()`, `checksum()`,
`build_packet()`, `parse_response()`, `sleep_ms()`, and the alias
`LX16A == Servo` / `LX16ABus == SerialBus` / `Bus == SerialBus`.

### Drop-in `pylx16a`

`python/pylx16a/` is a faithful port of `ethanlipson/PyLX-16A`: the class-level
controller, the static initialiser, every documented method, and the exception
hierarchy with its `id_` attribute.

```python
from pylx16a import LX16A

LX16A.initialize("/dev/ttyUSB0")            # timeout=0.02 s, pyserial-style
servo = LX16A(1, disable_torque=False)

servo.move(0, 1000)
servo.move(30, 0, relative=True, wait=True)
servo.move_start()
servo.move_stop()

servo.set_angle_limits(10, 150)
servo.set_vin_limits(5000, 11000)
servo.set_temp_limit(80)
servo.motor_mode(500)
servo.servo_mode()
servo.set_led_error_triggers(True, False, True)

print(servo.get_physical_angle(), servo.get_vin(), servo.get_temp())
print(servo.get_angle_limits(), servo.get_led_error_triggers())
```

`examples/python/pylx16a_compat.py` runs 65 checks against that surface —
argument rejection, logical errors, register round-trips and the B-spline
helpers — and passes on the in-process model with no hardware attached.

### Batch operations

The batch entry points release the GIL and push everything through one
`write(2)`:

```python
from hiwonder_bus import read_angles, read_telemetry, set_torque, move_sync

move_sync(bus, servos, angles, time_ms=400, start=True)  # N cmd-7 + 0xFE start
read_angles(bus, servos)                                  # degrees, NaN on failure
read_telemetry(bus, servos)                               # [Telemetry(...), ...]
set_torque(bus, servos, False)                             # one write
```

`SerialBus` also has id-based versions (`bus.move_sync(ids, angles, ...)`,
`bus.set_torque(ids, enable)`) for when you would rather not own `Servo`
objects, plus `bus.broadcast_move_start()` / `broadcast_move_stop()`.

Measured on a 10-servo rig: **106 bytes** on the wire for a full synchronized
move — ten 10-byte targets plus a 6-byte broadcast — in a single syscall.

---

## Testing without hardware

The same simulator the SDK's own tests use is public and installable:

```cpp
#include <HiwonderBus/testing/simulator.hpp>

auto sim = std::make_unique<HiwonderBus::testing::SimulatedTransport>(
    std::vector<std::uint8_t>{1, 2, 3});
sim->servo(2)->temp_c = 51;                 // seed registers
sim->disconnect(3);                         // make id 3 "fall off the bus"

BusConfig config;
config.device = "simulator";
SerialBus bus(std::move(sim), config);
bus.open("simulator");
```

```python
from hiwonder_bus import make_simulated_bus, simulate_disconnect

bus = make_simulated_bus([1, 2, 3])
simulate_disconnect(bus, 2)                 # unplug id 2
```

The model speaks the real protocol — framing, checksums, broadcast, the
`MOVE_WAIT`/`MOVE_START` handshake — and it can also lie on purpose
(`corruptResponses`, `hangResponses`, `echoTx`) so you can prove your code
survives line noise.

Both examples run in this mode:

```bash
./build/scan_bus --simulate
./build/sync_biped_demo --simulate --steps 12 --period 25
PYTHONPATH=python python examples/python/pylx16a_compat.py
PYTHONPATH=python python examples/python/batch_control.py --joints 12 --hz 100 --unplug 5
```

---

## Using from CMake

```cmake
find_package(hiwonder_servo_classic REQUIRED)
target_link_libraries(my_app PRIVATE HiwonderBus::hiwonder_servo_classic)
```

The exported package config (`hiwonder_servo_classicConfig.cmake`) is installed
under `<prefix>/lib/cmake/hiwonder_servo_classic/`, so:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local
```

...resolves the imported target, the include directory and the `Threads`
dependency. The alias `HiwonderBus::hiwonder_servo_classic` is the target you
link against; `hiwonder_servo_classic` is the local build-tree name.

---

## How the wire works

Every frame is bracketed by a two-byte header and terminated by an inverted
one's-complement checksum covering everything after the header:

```text
request :  0x55 0x55 | id | length | cmd  | args...            | checksum
response:  0x55 0x55 | id | length | stat | data...            | checksum
                   \__/  \______________________________/ \__/  \_____/
                 header   inner payload (covered)          checksum
```

`length` counts the inner payload (id + length + cmd/stat + args), so a frame
is always exactly `length + 3` bytes. Requests go to `0..253`; `254` is
broadcast and is never answered, so it may only be used for writes.

The move handshake is the reason synchronization works:

```text
MOVE        (cmd 1)  execute now
MOVE_WAIT   (cmd 7)  store the target, wait for the trigger
MOVE_START  (cmd 11) release every stored target  <-- sent to 0xFE once
MOVE_STOP   (cmd 12) abort
```

`src/serial_bus.cpp` implements the framing: an RX buffer, a resync loop that
skips garbage, an echo guard that discards any "reply" byte-identical to the
frame it just sent, and a `poll(2)` deadline taken from `CLOCK_MONOTONIC`. The
whole transaction — flush, write, read — holds one mutex, because on a
half-duplex bus an interleaved second writer would steal the reply.

---

## Architecture

```text
             Python                                   C++
  ┌───────────────────────────┐        ┌───────────────────────────────────┐
  │ pylx16a   (drop-in shim)  │        │  HiwonderBus namespace            │
  │ hiwonder_bus (general API)│        │                                   │
  │   exceptions.py           │        │  Servo ──── angle/tick mirrors    │
  │   __init__.pyi (stubs)    │        │    │  validation, caching          │
  └────────────┬──────────────┘        │    │  batch statics                │
               │ pybind11              │  SerialBus ── framing, resync      │
        ┌──────▼──────────┐            │    │  mutex, stats, broadcast       │
        │ _hiwonder_bus   │────────────│  Transport (interface)             │
        │ bindings.cpp    │            │    │                                │
        └─────────────────┘            │  PosixTransport  testing::Simulated │
                                       │   termios/poll/  Transport          │
                                       │   ioctl          (header-only)      │
                                       └───────────────────────────────────┘
```

Design decisions worth knowing when reading the code:

- **`Transport` is an injection seam.** `SerialBus` owns framing and policy;
  the byte pipe is pluggable. That is how the tests and examples run without a
  UART, and it is where you would hang an SPI or CAN backend.
- **Mirrors live in `Servo`, in tick units.** Commanded angle, offsets, limits,
  torque, LED and mode are cached and updated on every accepted command.
  `poll_hardware=false` reads the cache. This is what the non-throwing real-time
  mode is built on: cached reads issue no syscalls. Hardware batch reads remain
  sequential request/reply exchanges.
- **Batch writes, serialized reads.** Writes are coalesced; reads must be
  round-trips because the bus is half-duplex and simultaneous replies collide.
  Batch reads therefore keep the per-servo exchange but drop the Python overhead
  and the GIL.
- **The checksum never covers itself.** `calculateChecksum(frame, size)` sums
  from the header's end to `size`, so callers must pass the size *without* the
  trailing byte — a mistake the tests pin down.

---

## Building

```text
hiwonder-servo-classic-sdk/
├── CMakeLists.txt                      # lib + alias + export + options
├── pyproject.toml                      # scikit-build-core
├── cmake/hiwonder_servo_classicConfig.cmake.in
├── include/HiwonderBus/
│   ├── protocol.hpp                    # 0x55 0x55 opcodes, framing, tick geometry
│   ├── exceptions.hpp                  # error hierarchy + ErrorCode
│   ├── serial_bus.hpp                  # Transport, BusConfig, SerialBus
│   ├── servo.hpp                       # Servo + the part-name aliases
│   ├── HiwonderBus.hpp                 # umbrella + version
│   └── testing/simulator.hpp           # in-process servo model
├── src/
│   ├── serial_bus.cpp                  # termios + poll + ioctl + framing
│   └── servo.cpp                       # every command, mirror and batch helper
├── python/
│   ├── bindings.cpp                    # pybind11 module _hiwonder_bus
│   ├── hiwonder_bus/{__init__.py,__init__.pyi,exceptions.py,py.typed}
│   └── pylx16a/__init__.py             # drop-in PyLX-16A shim
├── examples/cpp/{scan_bus,sync_biped_demo}.cpp
├── examples/python/{pylx16a_compat,batch_control}.py
└── tests/                              # protocol, review regressions, PTY and Python tests
```

### CMake options

| Option | Default | Meaning |
| --- | --- | --- |
| `BUILD_PYTHON_BINDINGS` | `ON` | build `_hiwonder_bus` via pybind11 (skips politely if pybind11 is missing when not building a wheel) |
| `BUILD_EXAMPLES` | `ON` | build the two C++ examples |
| `BUILD_TESTS` | `ON` | build protocol/regression tests, Linux PTY tests, and per-header checks, registered with CTest |
| `HIWONDER_SERVO_CLASSIC_WARNINGS` | `ON` | `-Wall -Wextra -Wshadow -Wconversion` on our own targets |
| `BUILD_SHARED_LIBS` | `OFF` | use `.so` instead of `.a` for normal CMake builds; wheels always use static SDK linkage |

### Test suite

`ctest --test-dir build` runs:

- `protocol_tests` — 759 checks: checksums, framing, length fields, resync
  over garbage, signed angle registers, non-default tick geometry, batch
  coalescing (asserting *one* `write(2)` for N moves), broadcast release,
  timeouts, corruption, echo rejection, mirror-only getters, and the whole
  exception mapping.
- `review_regressions` — independent literal wire fixtures, response correlation,
  retry/error recovery, motion limits, C++ wait-argument compatibility, telemetry,
  broadcast safety, and concurrent bus snapshots.
- `posix_transport_tests` (Linux) — real PTYs exercise the production backend:
  backpressure/partial writes, deadline recovery, signal interruption, and
  fragmented replies. CTest bounds the executable so an I/O hang fails the test.
- `header_check_*` — one tiny TU per public header, so a missing `#include`
  fails the build instead of failing a downstream user.

After `pip install -e '.[test]'`, run `python -m pytest tests/` for Python API
and crash-subprocess regressions, lifetime retention, numeric validation,
blocked-PTY GIL progress, cache/config snapshots, and spline endpoints.
Optionally install `mypy` and run `mypy --strict tests/typing_smoke.py` against
an installed wheel to check the published typing surface.

These checks require no powered servo. Passing them does **not** validate
adapter turnaround, electrical wiring, firmware behavior, stopping latency, or
motion under load.

---

## Hardware notes

- **Devices.** `/dev/ttyUSB*` (FTDI/CH340/CH341 BusLinker), `/dev/ttyACM*`, and
  on Jetson-class boards `/dev/ttyTHS*`. All speak the same protocol.
- **Permissions.** add yourself to `dialout` (or `uucp`) — the error message
  from `open()` already says this.
- **Power.** Servos stall the supply, not the bus. If you see scattered
  `ServoChecksumError`, look at the supply rails before the code; the driver
  flushes, resyncs and drops echoes, so most wiring problems present as
  *timeouts* rather than corruption.
- **`lastHint()`** reports a refused `ASYNC_LOW_LATENCY` request. Support and
  its effect on USB/UART latency depend on the driver; measure on your adapter
  rather than assuming a fixed latency improvement.
- **Half-duplex.** Never drive two transactions at once from two threads on the
  same fd *outside* this SDK; `SerialBus` serializes them with a mutex.

---

## Acknowledgements & licence

MIT — see [LICENSE](LICENSE).

The protocol behaviour this SDK reproduces comes from the public firmware
documentation of the classic Hiwonder/LewanSoul controllers plus two reference
implementations:

- **alecxcode/LX16A-bus** — the degree-based object model, and the shape of
  every request frame.
- **ethanlipson/PyLX-16A** — the Python surface, the validation policy ("raise,
  never clamp"), the cached-mirror design and the exception hierarchy.

`examples/python/pylx16a_compat.py` is the contract test for that second one;
two deliberate corrections are documented in `python/pylx16a/__init__.py`
(the B-spline recursion and its out-of-range sample), both of which were
unreachable bugs upstream.
