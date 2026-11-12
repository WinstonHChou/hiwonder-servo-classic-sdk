#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
#
# examples/python/batch_control.py
#   High-rate multi-servo loop using the vectorized API.
#
#   The loop drives N joints through a gait and reads their telemetry, using
#   three batch entry points only:
#
#     move_sync(bus, servos, angles, time_ms)   N cmd-7 frames + 0xFE start,
#                                               coalesced into ONE write(2)
#     read_telemetry(bus, servos)               angle/vin/temp sweep
#     set_torque(bus, servos, enable)           one write(2)
#
#   All of them release the GIL, so a monitor thread can keep up while the
#   wire is busy -- and `--unplug` demonstrates that a servo that drops out is
#   reported per-joint instead of taking the loop down.
#
#   python examples/python/batch_control.py                      # model
#   python examples/python/batch_control.py --port /dev/ttyUSB0  # hardware
#   python examples/python/batch_control.py --joints 12 --hz 100 --unplug 5
#
import argparse
import math
import threading
import time

from hiwonder_bus import (
    ErrorMode,
    SerialBus,
    Servo,
    make_simulated_bus,
    move_sync,
    set_torque,
    simulate_disconnect,
)


def build_servos(count):
    """Ids 1..count, mirroring a real multi-servo rig."""
    return list(range(1, count + 1))


def main(argv=None):
    parser = argparse.ArgumentParser(description="batch bus control loop")
    parser.add_argument("--port", help="serial device; omit to use the in-process model")
    parser.add_argument("--joints", type=int, default=10, help="number of servos (default 10)")
    parser.add_argument("--hz", type=float, default=50.0, help="control rate (default 50)")
    parser.add_argument("--seconds", type=float, default=2.0, help="how long to run")
    parser.add_argument("--unplug", type=int, default=None,
                        help="simulate this servo dropping out half way (model only)")
    parser.add_argument("--verbose", action="store_true", help="print every cycle")
    args = parser.parse_args(argv)

    ids = build_servos(args.joints)

    if args.port:
        bus = SerialBus(device=args.port, read_timeout_ms=10, write_timeout_ms=50)
    else:
        bus = make_simulated_bus(ids)
    # A dead servo must not raise its way out of the control loop.
    bus.error_mode = ErrorMode.NON_THROWING

    print(bus.last_error() or "")
    print(f"bus: {bus.device} @ {bus.baudrate} 8N1, transport={bus.transport}")
    print(f"{len(ids)} servos, target {args.hz:g} Hz for {args.seconds:g}s "
          f"({1 / args.hz * 1000:.1f} ms budget per cycle)\n")

    servos = [Servo(bus, id_) for id_ in ids]

    # One write(2) for every joint, whatever the joint count.
    bytes_on = set_torque(bus, servos, True)
    print(f"enabled torque on {len(servos)} servos: {bytes_on} bytes in "
          f"1 write(2) = {bytes_on / max(1, len(servos)):.0f} bytes/servo")

    stop = threading.Event()
    samples = []

    def monitor():
        """Runs alongside the wire: proves the GIL is released where it counts."""
        while not stop.is_set():
            # Distinct threads share a bus, not mutable Servo mirrors.
            snapshot = bus.read_telemetry(ids)
            live = [s for s in snapshot if s.valid]
            samples.append((time.perf_counter(), len(live)))
            stop.wait(0.02)

    watcher = threading.Thread(target=monitor, daemon=True)
    watcher.start()

    period = 1.0 / args.hz
    deadline = time.perf_counter() + args.seconds
    cycles = 0
    overruns = 0
    failures = 0
    last_report = 0.0
    unplug_cycle = None
    unplug_at = deadline - args.seconds / 2

    try:
        while time.perf_counter() < deadline:
            started = time.perf_counter()
            cycles += 1
            if (args.unplug is not None and not args.port
                    and unplug_cycle is None and started >= unplug_at):
                simulate_disconnect(bus, args.unplug, present=False)
                unplug_cycle = cycles
                print(f"\nsimulated: servo {args.unplug} unplugged at cycle {cycles}")

            phase = started * 2.0
            gait = []
            for index, servo in enumerate(servos):
                neutral = 90.0
                swing = 25.0 if index % 2 == 0 else -25.0
                # A staggered sine so the frame looks like a walk, not a twitch.
                target = neutral + swing * math.sin(phase * 2 * math.pi + index * 0.5)
                gait.append(max(0.0, min(240.0, target)))

            written = move_sync(bus, servos, gait, time_ms=int(period * 2000))
            if written == 0:
                failures += 1

            if args.verbose or time.perf_counter() - last_report > 0.5:
                last_report = time.perf_counter()
                angles = [s.get_physical_angle(False) for s in servos]
                preview = " ".join(f"{a:6.1f}" for a in angles[:6])
                print(f"  cycle {cycles:4d}  queued={written:3d}B  angles[{preview} ...]")

            elapsed = time.perf_counter() - started
            sleep_for = period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                overruns += 1

        if unplug_cycle is not None:
            angles = bus.read_telemetry(ids)
            broken = [a.id for a in angles if not a.valid]
            print(f"after the dropout the sweep reports invalid ids: {broken}")
            still_running = sum(1 for a in angles if a.valid)
            print(f"completed {cycles - unplug_cycle + 1} control cycles after disconnect; "
                  f"{still_running}/{len(angles)} servos still reporting")

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        stop.set()
        watcher.join(timeout=0.5)
        set_torque(bus, servos, False)
        print("torque released")

    duration = max(1e-9, time.perf_counter() - (deadline - args.seconds))
    stats = bus.stats()
    rates = [n for _, n in samples]
    print("\n--- summary ---")
    print(f"  cycles            : {cycles} ({cycles / duration:.1f} Hz achieved, "
          f"{overruns} overruns, {failures} failed moves)")
    print(f"  monitor samples   : {len(samples)} "
          f"(mean {sum(rates) / len(rates):.1f} of {len(servos)} servos answering)"
          if rates else "  monitor samples   : none")
    print(f"  bus writes        : {stats.writes} syscalls carrying "
          f"{stats.tx_bytes} bytes tx / {stats.rx_bytes} bytes rx")
    print(f"  transactions      : {stats.transactions} "
          f"({stats.timeouts} timeouts, {stats.checksum_errors} checksum errors, "
          f"{stats.echo_drops} echoes dropped)")
    print("\nNote the writes-per-cycle ratio: a 10-joint synchronized move is a")
    print("single write(2), which is what keeps the joints firing together on a")
    print("half-duplex bus. Serialising them one syscall each would cost 10x.")

    bus.close()
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
