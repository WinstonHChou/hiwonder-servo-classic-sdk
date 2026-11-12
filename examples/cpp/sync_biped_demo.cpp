// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// examples/cpp/sync_biped_demo.cpp
//   A 10-DOF biped walk cycle on the classic bus.
//
//   The interesting part is *how* the moves go out. Every joint target is
//   queued with SERVO_MOVE_TIME_WAIT_WRITE (opcode 7) and then released by one
//   broadcast SERVO_MOVE_START (0xFE, opcode 11) -- and the whole burst leaves
//   through a single write(2). On a half-duplex bus that is what makes ten
//   joints start within microseconds of each other instead of a millisecond
//   apart.
//
//   It also shows the two failure modes a real biped has to survive:
//     * ErrorMode::kNonThrowing so one disconnected leg servo does not unwind
//       the control loop, and
//     * cached mirrors (`poll_hardware = false`) for the values the loop needs
//       at 100 Hz, with telemetry polled on a slower cadence.
//
//   $ ./sync_biped_demo /dev/ttyUSB0
//   $ ./sync_biped_demo /dev/ttyUSB0 --steps 20 --period 20
//   $ ./sync_biped_demo --simulate          # run the loop with no hardware
//
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <algorithm>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "HiwonderBus/HiwonderBus.hpp"
#include "HiwonderBus/testing/simulator.hpp"

using namespace HiwonderBus;

namespace {

/// The ten joints of a typical LX-16A biped: two hips, two knees, two ankles,
/// two arms, two "waist/shoulder" helpers.
constexpr std::size_t kDoF = 10;

struct Joint {
  const char* name;
  std::uint8_t id;
  float neutral_deg;   ///< stance pose
  float swing_deg;     ///< how far it travels during a step
};

/// The classic ALPHABOT-style mapping, ids 1..10.
const Joint kJoints[kDoF] = {
    {"hip_right", 1, 90.0f, 20.0f}, {"knee_right", 2, 90.0f, 30.0f},
    {"ankle_right", 3, 90.0f, 15.0f}, {"hip_left", 4, 90.0f, -20.0f},
    {"knee_left", 5, 90.0f, -30.0f}, {"ankle_left", 6, 90.0f, -15.0f},
    {"arm_right", 7, 60.0f, 25.0f}, {"arm_left", 8, 60.0f, -25.0f},
    {"waist", 9, 90.0f, 5.0f}, {"torso", 10, 90.0f, 0.0f},
};

void printUsage(const char* program) {
  std::cout
      << "usage: " << program << " <device> [--steps N] [--period MS] [--throwing]\n"
      << "       " << program << " --simulate [options]\n"
      << "\n"
      << "  --steps N     walk cycles to run        (default 12)\n"
      << "  --period MS   milliseconds per cycle    (default 25)\n"
      << "  --throwing    strict mode: abort on the first failed servo\n"
      << "  --simulate    drive the loop from the in-process model\n";
}

std::string fixed(float value, int precision = 2) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0] == "-h" || args[0] == "--help") {
    printUsage(argv[0]);
    return args.empty() ? 1 : 0;
  }

  std::string device;
  bool simulate = false;
  bool throwing = false;
  int steps = 12;
  int period_ms = 25;

  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string& arg = args[i];
    if (arg == "--simulate") {
      simulate = true;
    } else if (arg == "--throwing") {
      throwing = true;
    } else if (arg == "--steps" && i + 1 < args.size()) {
      steps = std::atoi(args[++i].c_str());
    } else if (arg == "--period" && i + 1 < args.size()) {
      period_ms = std::atoi(args[++i].c_str());
    } else if (device.empty() && !simulate) {
      device = arg;
    }
  }

  if (!simulate && device.empty()) {
    std::cerr << "error: a device path is required (or pass --simulate)\n";
    return 2;
  }

  BusConfig config;
  config.baudrate = kDefaultBaudrate;
  config.read_timeout_ms = 10;   // a reply is 8 bytes: 10 ms is generous at 115200
  config.write_timeout_ms = 50;  // ten queued moves plus the broadcast
  config.enable_low_latency = true;
  config.flush_before_query = true;
  config.reject_tx_echo = true;
  // The biped keeps walking even if a leg servo falls off the bus.
  config.error_mode = throwing ? ErrorMode::kStrict : ErrorMode::kNonThrowing;

  std::unique_ptr<SerialBus> bus;
  if (simulate) {
    std::vector<std::uint8_t> ids;
    for (std::size_t i = 0; i < kDoF; ++i) ids.push_back(kJoints[i].id);
    auto simulator = std::make_unique<testing::SimulatedTransport>(ids);
    for (std::size_t i = 0; i < kDoF; ++i) {
      if (testing::SimulatedServo* model = simulator->servo(kJoints[i].id)) {
        model->physical_angle_ticks = static_cast<std::int16_t>(
            angleToTicks(kJoints[i].neutral_deg));
        model->temp_c = static_cast<std::uint8_t>(35 + i);
        model->vin_mv = static_cast<std::uint16_t>(7300 + 17 * i);
      }
    }
    config.device = "simulator";
    bus = std::make_unique<SerialBus>(std::move(simulator), config);
    if (!bus->open("simulator")) {
      std::cerr << "error: the simulator refused to open\n";
      return 2;
    }
  } else {
    config.device = device;
    bus = std::make_unique<SerialBus>(config);
    std::string error;
    if (!bus->tryOpen(device, &error)) {
      std::cerr << "error: " << error << "\n";
      return 2;
    }
  }

  std::cout << versionString() << "\n"
            << "bus: " << bus->device() << " @ " << bus->config().baudrate
            << " 8N1, " << (throwing ? "strict" : "non-throwing") << " mode\n";

  // --- Build the joints ----------------------------------------------------
  std::vector<Servo*> joints;
  for (std::size_t i = 0; i < kDoF; ++i) {
    joints.push_back(new Servo(*bus, kJoints[i].id));
  }

  // Torque up front, in ONE write(2).
  const std::size_t torque_bytes = Servo::setTorque(*bus, joints, true);
  std::cout << "enabled torque on " << joints.size() << " servos in one write ("
            << torque_bytes << " bytes)\n";

  // --- Walk ---------------------------------------------------------------
  const auto started = std::chrono::steady_clock::now();
  std::uint64_t sync_bytes = 0;
  std::uint64_t sync_writes = 0;
  long long worst_cycle_us = 0;

  for (int step = 0; step < steps; ++step) {
    const auto cycle_started = std::chrono::steady_clock::now();

    // Phase: 0 = right swing, 1 = left swing. Everything is a target, so the
    // broadcast start makes the ten joints change direction together.
    const float phase = (step % 2 == 0) ? 1.0f : -1.0f;
    std::vector<float> targets;
    targets.reserve(kDoF);
    for (std::size_t i = 0; i < kDoF; ++i) {
      const Joint& joint = kJoints[i];
      const float target = joint.neutral_deg + phase * joint.swing_deg *
                                                   std::sin(3.14159265f * step /
                                                            std::max(1, steps));
      targets.push_back(target);
    }

    // The synchronized move: N cmd-7 frames + the 0xFE cmd-11 broadcast, all in
    // a single write(2).
    ErrorCode code = ErrorCode::kOk;
    const std::size_t written =
        Servo::moveSync(*bus, joints, targets, /*time_ms=*/static_cast<std::uint16_t>(period_ms * 2),
                        /*broadcast_start=*/true, &code);
    if (written == 0) {
      std::cerr << "step " << step << ": synchronized move failed ("
                << toString(code) << "): " << bus->lastError() << "\n";
      if (throwing) break;
    } else {
      sync_bytes += written;
      ++sync_writes;
    }

    // Telemetry every 4th cycle, so the 100 Hz loop is not slowed down by it.
    if (step % 4 == 0) {
      const std::vector<Telemetry> telemetry = Servo::readTelemetry(*bus, joints);
      std::ostringstream line;
      line << "  step " << std::setw(3) << step << "  ";
      for (std::size_t i = 0; i < kDoF && i < telemetry.size(); ++i) {
        const Telemetry& snapshot = telemetry[i];
        if (i) line << " | ";
        line << kJoints[i].name << "="
             << (snapshot.valid ? fixed(snapshot.angle_deg, 1) : "--");
      }
      const Telemetry& first = telemetry.front();
      if (first.valid) line << "   vin=" << first.vin_mv << "mV temp=" << first.temp_c << "C";
      std::cout << line.str() << "\n";
    }

    const auto cycle_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - cycle_started)
                              .count();
    if (cycle_us > worst_cycle_us) worst_cycle_us = cycle_us;

    // Pace the loop against the wall clock.
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - started)
                             .count();
    const long long target_ms = static_cast<long long>((step + 1) * period_ms);
    if (elapsed < target_ms) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(target_ms - static_cast<long long>(elapsed)));
    }
  }

  // --- Park it and let go --------------------------------------------------
  std::vector<float> neutral;
  for (std::size_t i = 0; i < kDoF; ++i) neutral.push_back(kJoints[i].neutral_deg);
  Servo::moveSync(*bus, joints, neutral, 1000, true);

  const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - started)
                            .count();
  const BusStats stats = bus->stats();
  std::cout << "\n"
            << steps << " cycles in " << total_ms << " ms\n"
            << "  " << sync_writes << " synchronized moves, " << sync_bytes
            << " bytes queued (one write(2) each)\n"
            << "  worst cycle " << worst_cycle_us << " us\n"
            << "  bus: " << stats.writes << " writes, " << stats.tx_bytes << " bytes out, "
            << stats.rx_bytes << " bytes in, " << stats.timeouts << " timeouts, "
            << stats.checksum_errors << " checksum errors, " << stats.echo_drops
            << " echoes dropped\n";

  // Report anything the non-throwing mode swallowed.
  for (std::size_t i = 0; i < kDoF; ++i) {
    if (!joints[i]->healthy()) {
      std::cout << "  warning: " << kJoints[i].name << " (id "
                << static_cast<int>(joints[i]->id()) << ") last error: "
                << toString(joints[i]->lastError()) << " " << joints[i]->lastMessage()
                << "\n";
    }
  }

  std::cout << "releasing torque (they go limp, so support the frame)\n";
  Servo::setTorque(*bus, joints, false);

  for (Servo* servo : joints) delete servo;
  bus->close();
  return 0;
}
