// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// examples/cpp/scan_bus.cpp
//   Walk every unicast address, print the servos that answer, and dump a
//   little telemetry for each one. Usually the first thing to run on a new
//   bench: it proves the BusLinker wiring, the baud rate and the supply all
//   at once.
//
//   $ ./scan_bus /dev/ttyUSB0
//   $ ./scan_bus /dev/ttyUSB0 1 12
//   $ ./scan_bus --simulate              # no hardware needed
//
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "HiwonderBus/HiwonderBus.hpp"
#include "HiwonderBus/testing/simulator.hpp"

using namespace HiwonderBus;

namespace {

void printUsage(const char* program) {
  std::cout
      << "usage: " << program << " <device> [first_id] [last_id] [--probe N] [--timeout MS]\n"
      << "       " << program << " --simulate [first_id] [last_id]\n"
      << "\n"
      << "  <device>      /dev/ttyUSB*, /dev/ttyACM*, /dev/ttyTHS*, ...\n"
      << "  first_id      scan start address (default 0)\n"
      << "  last_id       scan end address   (default 253)\n"
      << "  --probe N     opcode used to probe, default 28 (read physical angle)\n"
  "  --timeout MS  per-address read budget in ms, default 15\n"
  "  --simulate    answer from the in-process model, no hardware required\n";
}

int fail(const std::string& message) {
  std::cerr << "error: " << message << "\n";
  return 2;
}

std::string toStr(float value, int precision = 2) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

/// One telemetry line per servo, from three independent reads.
void printServo(SerialBus& bus, std::uint8_t id) {
  Servo servo(bus, id);
  float angle = 0.0f;
  int vin = 0;
  int temp = 0;
  const bool angle_ok = servo.tryGetPhysicalAngle(&angle, true);
  const bool vin_ok = servo.tryGetVin(&vin);
  const bool temp_ok = servo.tryGetTemp(&temp);

  std::cout << "  " << std::setw(3) << static_cast<int>(id) << "  " << std::setw(8)
            << (angle_ok ? toStr(angle) + "\xc2\xb0" : "timeout") << "  " << std::setw(8)
            << (vin_ok ? std::to_string(vin) + " mV" : "timeout") << "  " << std::setw(6)
            << (temp_ok ? std::to_string(temp) + " C" : "timeout") << "  "
            << (servo.isTorqueEnabled(false) ? "torque=on " : "torque=off")
            << (servo.isMotorMode(false) ? " motor" : " servo ") << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0] == "-h" || args[0] == "--help") {
    printUsage(argv[0]);
    return args.empty() ? 1 : 0;
  }

  bool simulate = false;
  std::string device;
  int first = -1;
  int last = 253;
  int probe = static_cast<int>(Command::kReadPhysicalAngle);
  int timeout_ms = 15;

  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string& arg = args[i];
    if (arg == "--simulate") {
      simulate = true;
    } else if (arg == "--probe" && i + 1 < args.size()) {
      probe = std::atoi(args[++i].c_str());
    } else if (arg == "--timeout" && i + 1 < args.size()) {
      timeout_ms = std::atoi(args[++i].c_str());
    } else if (device.empty() && !simulate) {
      device = arg;
    } else if (first < 0) {
      first = std::atoi(arg.c_str());
    } else {
      last = std::atoi(arg.c_str());
    }
  }
  if (first < 0) first = 0;

  if (!simulate && device.empty()) return fail("a device path is required");
  if (first < 0 || first > 253 || last < first || last > 253) {
    return fail("addresses must satisfy 0 <= first <= last <= 253");
  }

  BusConfig config;
  config.baudrate = kDefaultBaudrate;
  config.read_timeout_ms = timeout_ms;
  config.enable_low_latency = true;  // skip the 16 ms USB-serial receive timer
  // One unreachable address must not abort a 253-address sweep.
  config.error_mode = ErrorMode::kNonThrowing;

  std::unique_ptr<SerialBus> bus;
  if (simulate) {
    std::vector<std::uint8_t> ids;
    for (std::uint8_t id = static_cast<std::uint8_t>(first);
         id <= static_cast<std::uint8_t>(last) && ids.size() < 6; ++id) {
      ids.push_back(id);
    }
    auto simulator = std::make_unique<testing::SimulatedTransport>(ids);
    // Seed plausible telemetry so the table is worth looking at.
    for (std::uint8_t id : ids) {
      if (testing::SimulatedServo* model = simulator->servo(id)) {
        model->physical_angle_ticks = static_cast<std::int16_t>(111 * id);
        model->vin_mv = static_cast<std::uint16_t>(7200 + 11 * id);
        model->temp_c = static_cast<std::uint8_t>(33 + id);
        model->torque_enabled = (id % 2) == 0;
      }
    }
    config.device = "simulator";
    bus = std::make_unique<SerialBus>(std::move(simulator), config);
    if (!bus->open("simulator")) return fail("the simulator refused to open");
  } else {
    config.device = device;
    bus = std::make_unique<SerialBus>(config);
    std::string error;
    if (!bus->tryOpen(device, &error)) {
      std::cerr << "error: " << error << "\n"
                << "hint: is the BusLinker plugged in, and is this user in the "
                   "dialout/uucp group?\n";
      return 2;
    }
  }

  std::cout << versionString() << "\n"
            << "opened " << bus->device() << " at " << bus->config().baudrate
            << " 8N1 (transport: " << bus->transportName() << ")\n";
  if (const std::string hint = bus->lastHint(); !hint.empty()) {
    std::cout << "note: " << hint << "\n";
  }

  const auto started = std::chrono::steady_clock::now();
  std::vector<std::uint8_t> ids;
  if (bus->tryScanIds(static_cast<std::uint8_t>(first), static_cast<std::uint8_t>(last),
                      static_cast<Command>(probe), timeout_ms, &ids)) {
    // found into `ids`
  } else {
    std::cerr << "scan aborted: " << bus->lastError() << "\n";
    return 2;
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - started)
                           .count();

  std::cout << "\n   id     angle       vin      temp  flags\n"
            << "  ---  --------  --------  ------  ---------\n";
  for (std::uint8_t id : ids) printServo(*bus, id);

  const BusStats stats = bus->stats();
  std::cout << "\n"
            << ids.size() << " servo(s) on " << bus->device() << " in " << elapsed << " ms ("
            << stats.transactions << " transactions, " << stats.rx_bytes << " bytes in, "
            << stats.tx_bytes << " bytes out, " << stats.writes << " write syscalls, "
            << stats.timeouts << " timeouts, " << stats.checksum_errors << " checksum errors)\n";

  bus->close();
  return ids.empty() ? 1 : 0;
}
