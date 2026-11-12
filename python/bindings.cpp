// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// python/bindings.cpp
//   pybind11 module `_hiwonder_bus`.
//
//   * The generalized API (Servo / SerialBus / BusConfig / ...) lives here.
//   * Every blocking call drops the GIL, so the module is safe to drive from
//     several worker threads and never freezes the interpreter during a read.
//   * Batch calls hand the whole frame vector to one write(2); see
//     SerialBus::queueMoves for why a synchronized move is one syscall.
//   * C++ errors are re-raised as the Python exception hierarchy defined in
//     hiwonder_bus/exceptions.py, so `except ServoError` from PyLX-16A code
//     keeps working unchanged, `exc.id_` included.
//
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "HiwonderBus/HiwonderBus.hpp"
#include "HiwonderBus/testing/simulator.hpp"

namespace py = pybind11;

using HiwonderBus::BusConfig;
using HiwonderBus::BusStats;
using HiwonderBus::Command;
using HiwonderBus::ErrorCode;
using HiwonderBus::ErrorMode;
using HiwonderBus::LedTriggers;
using HiwonderBus::Packet;
using HiwonderBus::Response;
using HiwonderBus::SerialBus;
using HiwonderBus::Servo;
using HiwonderBus::ServoConfig;
using HiwonderBus::Telemetry;

namespace {

// ---------------------------------------------------------------------------
// Exception bridge
// ---------------------------------------------------------------------------

/// The exception classes are pure Python (see python/hiwonder_bus/exceptions.py)
/// so they can carry `id_` and subclass cleanly. Importing here, lazily and
/// only on the error path, keeps `import _hiwonder_bus` usable standalone.
constexpr const char* kExceptionModule = "hiwonder_bus.exceptions";

py::object exceptionClass(const char* class_name) {
  try {
    py::module_ module = py::module_::import(kExceptionModule);
    return module.attr(class_name);
  } catch (const py::error_already_set& e) {
    PyErr_Clear();  // package unavailable: degrade to a built-in exception
    return py::none();
  }
}

/// Pick the class name matching the concrete C++ exception.
const char* exceptionName(const HiwonderBus::ServoError& e) {
  if (dynamic_cast<const HiwonderBus::ServoTimeoutError*>(&e)) return "ServoTimeoutError";
  if (dynamic_cast<const HiwonderBus::ServoChecksumError*>(&e)) return "ServoChecksumError";
  if (dynamic_cast<const HiwonderBus::ServoArgumentError*>(&e)) return "ServoArgumentError";
  if (dynamic_cast<const HiwonderBus::ServoLogicalError*>(&e)) return "ServoLogicalError";
  if (dynamic_cast<const HiwonderBus::ServoIOError*>(&e)) return "ServoIOError";
  if (dynamic_cast<const HiwonderBus::ServoFirmwareError*>(&e)) return "ServoFirmwareError";
  return "ServoError";
}

[[noreturn]] void raiseFrom(const HiwonderBus::ServoError& e) {
  const py::object cls = exceptionClass(exceptionName(e));
  if (!cls.is_none()) {
    // Constructed exactly like PyLX-16A's: ServoTimeoutError(message, id_),
    // so `exc.id_` is populated for callers that inspect it.
    py::object id = e.id() == HiwonderBus::kNoServoId ? py::none() : py::cast(e.id());
    py::object instance = cls(std::string(e.what()), id);
    // Set then hand the error to pybind11's carrier: the default
    // error_already_set ctor fetches whatever PyErr_* just set. Works on both
    // pybind11 2.x and 3.x (no reliance on version-specific helpers).
    PyErr_SetObject(cls.ptr(), instance.ptr());
    throw py::error_already_set();
  }
  std::string message = e.what();
  if (e.id() != HiwonderBus::kNoServoId) {
    message += " (id=" + std::to_string(e.id()) + ")";
  }
  throw py::value_error(message);
}

/// Blocking call that may throw a C++ servo error:
///   * release the GIL for the duration of `fn`
///   * re-acquire it before the exception is translated
template <typename F>
void blocking_void(F&& fn) {
  try {
    py::gil_scoped_release guard;
    fn();
  } catch (const HiwonderBus::ServoError& e) {
    raiseFrom(e);
  }
}

/// Python-side validation errors must land in the PyLX-16A hierarchy, not in
/// bare ValueError, so that `except ServoError` keeps working.
[[noreturn]] void raiseArgument(const std::string& message, int servo_id) {
  raiseFrom(HiwonderBus::ServoArgumentError(message, servo_id));
}

[[noreturn]] void raiseLogical(const std::string& message, int servo_id) {
  raiseFrom(HiwonderBus::ServoLogicalError(message, servo_id));
}

template <typename R, typename F>
R blocking_value(F&& fn) {
  R result{};
  try {
    py::gil_scoped_release guard;
    result = fn();
  } catch (const HiwonderBus::ServoError& e) {
    raiseFrom(e);
  }
  return result;
}

/// `std::uint8_t` on a Python boundary is ambiguous (char vs. int), so every
/// id/opcode crosses as `int` and is narrowed here.
inline int toInt(std::uint8_t value) noexcept { return static_cast<int>(value); }
inline std::uint8_t toId(int value) {
  if (value < 0 || value > 0xFF) {
    raiseArgument("servo id must be in 0..255 (received " + std::to_string(value) + ")",
                  HiwonderBus::kNoServoId);
  }
  return static_cast<std::uint8_t>(value);
}

std::vector<int> toInts(const std::vector<std::uint8_t>& values) {
  std::vector<int> out;
  out.reserve(values.size());
  for (std::uint8_t v : values) out.push_back(toInt(v));
  return out;
}

std::uint8_t toUnicastId(int value) {
  if (value < 0 || value > HiwonderBus::kMaxServoId) {
    raiseArgument("servo id must be in 0..253", value);
  }
  return static_cast<std::uint8_t>(value);
}

std::uint16_t moveTime(int value, int id = HiwonderBus::kNoServoId) {
  if (value < 0 || value > HiwonderBus::kMaxMoveTimeMs) {
    raiseArgument("time must be in 0..30000 milliseconds", id);
  }
  return static_cast<std::uint16_t>(value);
}

Packet rawPacket(int id, int opcode, const std::vector<int>& args) {
  const auto target = toId(id);
  if (opcode < 0 || opcode > 255 || args.size() > 252) {
    raiseArgument("opcode must be a byte; payload must contain at most 252 bytes", id);
  }
  std::vector<std::uint8_t> payload;
  payload.reserve(args.size());
  for (int value : args) {
    if (value < 0 || value > 255) raiseArgument("payload values must be bytes", id);
    payload.push_back(static_cast<std::uint8_t>(value));
  }
  return Packet::make(target, static_cast<Command>(opcode), payload);
}

void checkBatchError(SerialBus& bus, ErrorCode code) {
  if (code != ErrorCode::kOk &&
      (code == ErrorCode::kInvalidArgument || code == ErrorCode::kInvalidState ||
       bus.errorMode() == ErrorMode::kStrict)) {
    HiwonderBus::detail::throwError(code, std::string("batch operation failed: ") +
                                  HiwonderBus::toString(code), HiwonderBus::kNoServoId);
  }
}

void validateServos(SerialBus& bus, const std::vector<Servo*>& servos) {
  for (const Servo* servo : servos) {
    if (!servo || &servo->bus() != &bus) {
      raiseArgument("all servos must belong to the supplied bus", HiwonderBus::kNoServoId);
    }
  }
}

std::vector<std::uint8_t> toIds(const std::vector<int>& values) {
  std::vector<std::uint8_t> out;
  out.reserve(values.size());
  for (int v : values) out.push_back(toUnicastId(v));
  return out;
}

ServoConfig configOrDefault(const py::object& config) {
  if (config && !config.is_none()) return config.cast<ServoConfig>();
  return ServoConfig::classic();
}

float quietNaN() noexcept { return std::numeric_limits<float>::quiet_NaN(); }

}  // namespace

// ===========================================================================
// Module
// ===========================================================================
PYBIND11_MODULE(_hiwonder_bus, m) {
  m.doc() = "Native Linux C++17 driver for the Hiwonder/LewanSoul classic bus "
            "servo protocol (0x55 0x55). Backs the hiwonder_bus and pylx16a packages.";
  m.attr("__version__") = HIWONDERBUS_VERSION_STRING;

  // --- Enums ---------------------------------------------------------------
  py::enum_<ErrorMode>(m, "ErrorMode",
                       "How hardware failures are reported.\n\n"
                       "STRICT throws (PyLX-16A behaviour); NON_THROWING records the\n"
                       "error and keeps going, so one dead servo cannot kill a loop.")
      .value("STRICT", ErrorMode::kStrict)
      .value("NON_THROWING", ErrorMode::kNonThrowing)
      .export_values();

  py::enum_<Command>(m, "Command", "Classic 0x55 0x55 opcodes.")
      .value("MOVE", Command::kMove)
      .value("READ_LAST_INSTANT_MOVE", Command::kReadLastInstantMove)
      .value("MOVE_WAIT", Command::kMoveWait)
      .value("READ_LAST_DELAYED_MOVE", Command::kReadLastDelayedMove)
      .value("MOVE_START", Command::kMoveStart)
      .value("MOVE_STOP", Command::kMoveStop)
      .value("WRITE_ID", Command::kWriteId)
      .value("READ_ID", Command::kReadId)
      .value("WRITE_ANGLE_OFFSET", Command::kWriteAngleOffset)
      .value("SAVE_ANGLE_OFFSET", Command::kSaveAngleOffset)
      .value("READ_ANGLE_OFFSET", Command::kReadAngleOffset)
      .value("WRITE_ANGLE_LIMITS", Command::kWriteAngleLimits)
      .value("READ_ANGLE_LIMITS", Command::kReadAngleLimits)
      .value("WRITE_VIN_LIMITS", Command::kWriteVinLimits)
      .value("READ_VIN_LIMITS", Command::kReadVinLimits)
      .value("WRITE_TEMP_LIMIT", Command::kWriteTempLimit)
      .value("READ_TEMP_LIMIT", Command::kReadTempLimit)
      .value("READ_TEMP", Command::kReadTemp)
      .value("READ_VIN", Command::kReadVin)
      .value("READ_PHYSICAL_ANGLE", Command::kReadPhysicalAngle)
      .value("WRITE_MOTOR_MODE", Command::kWriteMotorMode)
      .value("READ_MOTOR_MODE", Command::kReadMotorMode)
      .value("WRITE_TORQUE", Command::kWriteTorque)
      .value("READ_TORQUE", Command::kReadTorque)
      .value("WRITE_LED", Command::kWriteLed)
      .value("READ_LED", Command::kReadLed)
      .value("WRITE_LED_ERROR_TRIGGERS", Command::kWriteLedErrorTriggers)
      .value("READ_LED_ERROR_TRIGGERS", Command::kReadLedErrorTriggers)
      .export_values();

  py::enum_<ErrorCode>(m, "ErrorCode", "Machine-readable failure taxonomy.")
      .value("OK", ErrorCode::kOk)
      .value("NOT_OPEN", ErrorCode::kNotOpen)
      .value("TIMEOUT", ErrorCode::kTimeout)
      .value("SHORT_READ", ErrorCode::kShortRead)
      .value("BAD_CHECKSUM", ErrorCode::kBadChecksum)
      .value("BAD_FRAME", ErrorCode::kBadFrame)
      .value("FIRMWARE_STATUS", ErrorCode::kFirmwareStatus)
      .value("INVALID_ARGUMENT", ErrorCode::kInvalidArgument)
      .value("INVALID_STATE", ErrorCode::kInvalidState)
      .value("IO_ERROR", ErrorCode::kIoError)
      .value("PORT_BUSY", ErrorCode::kPortBusy)
      .value("UNSUPPORTED", ErrorCode::kUnsupported)
      .export_values();

  // --- Value types ---------------------------------------------------------
  py::class_<ServoConfig>(m, "ServoConfig",
                          "Angle <-> tick geometry of a servo family.\n\n"
                          "Default is the classic 1000 ticks / 0.0-240.0 deg.")
      .def(py::init<>())
      .def(py::init<std::uint16_t, float, float>(), py::arg("max_ticks"),
           py::arg("min_angle_deg"), py::arg("max_angle_deg"))
      .def_readwrite("max_ticks", &ServoConfig::max_ticks)
      .def_readwrite("min_angle_deg", &ServoConfig::min_angle_deg)
      .def_readwrite("max_angle_deg", &ServoConfig::max_angle_deg)
      .def_static("classic", &ServoConfig::classic,
                  "The classic geometry: 1000 ticks over 0.0-240.0 deg.")
      .def("deg_per_tick", &ServoConfig::degPerTick,
           "Degrees per tick (0.24 for the classic parts).")
      .def("span_deg", &ServoConfig::spanDeg, "Full-scale travel in degrees.")
      .def("to_ticks",
           [](const ServoConfig& self, float angle) {
             return static_cast<int>(HiwonderBus::angleToTicks(angle, self));
           },
           py::arg("angle_deg"), "Degrees -> tick count for this geometry.")
      .def("to_degrees",
           [](const ServoConfig& self, std::uint16_t ticks) {
             return HiwonderBus::ticksToAngle(ticks, self);
           },
           py::arg("ticks"), "Tick count -> degrees for this geometry.")
      .def("__eq__", [](const ServoConfig& a, const ServoConfig& b) {
        return a.max_ticks == b.max_ticks && a.min_angle_deg == b.min_angle_deg &&
               a.max_angle_deg == b.max_angle_deg;
      })
      .def("__repr__", [](const ServoConfig& c) {
        std::ostringstream oss;
        oss << "ServoConfig(max_ticks=" << c.max_ticks
            << ", min_angle_deg=" << c.min_angle_deg
            << ", max_angle_deg=" << c.max_angle_deg << ")";
        return oss.str();
      });

  py::class_<BusConfig>(m, "BusConfig", "Transport tunables for a SerialBus.")
      .def(py::init<>())
      .def(py::init<const std::string&>(), py::arg("device"))
      .def_readwrite("device", &BusConfig::device)
      .def_readwrite("baudrate", &BusConfig::baudrate)
      .def_readwrite("read_timeout_ms", &BusConfig::read_timeout_ms)
      .def_readwrite("write_timeout_ms", &BusConfig::write_timeout_ms)
      .def_readwrite("enable_low_latency", &BusConfig::enable_low_latency)
      .def_readwrite("flush_before_query", &BusConfig::flush_before_query)
      .def_readwrite("reject_tx_echo", &BusConfig::reject_tx_echo)
      .def_readwrite("max_retries", &BusConfig::max_retries)
      .def_readwrite("strict_status", &BusConfig::strict_status)
      .def_readwrite("exclusive", &BusConfig::exclusive)
      .def_readwrite("rx_buffer_size", &BusConfig::rx_buffer_size)
      .def_readwrite("error_mode", &BusConfig::error_mode)
      .def("__repr__", [](const BusConfig& c) {
        std::ostringstream oss;
        oss << "BusConfig(device='" << c.device << "', baudrate=" << c.baudrate
            << ", read_timeout_ms=" << c.read_timeout_ms
            << ", write_timeout_ms=" << c.write_timeout_ms
            << ", low_latency=" << (c.enable_low_latency ? "True" : "False") << ")";
        return oss.str();
      });

  py::class_<BusStats>(m, "BusStats", "Cumulative wire counters.")
      .def_readonly("tx_bytes", &BusStats::tx_bytes)
      .def_readonly("rx_bytes", &BusStats::rx_bytes)
      .def_readonly("writes", &BusStats::writes)
      .def_readonly("transactions", &BusStats::transactions)
      .def_readonly("timeouts", &BusStats::timeouts)
      .def_readonly("checksum_errors", &BusStats::checksum_errors)
      .def_readonly("io_errors", &BusStats::io_errors)
      .def_readonly("echo_drops", &BusStats::echo_drops)
      .def("__repr__", [](const BusStats& s) {
        std::ostringstream oss;
        oss << "BusStats(tx_bytes=" << s.tx_bytes << ", rx_bytes=" << s.rx_bytes
            << ", writes=" << s.writes << ", transactions=" << s.transactions
            << ", timeouts=" << s.timeouts
            << ", checksum_errors=" << s.checksum_errors << ")";
        return oss.str();
      });

  py::class_<LedTriggers>(m, "LedTriggers",
                          "Which faults make the servo's LED flash (opcode 35).")
      .def(py::init<>())
      .def(py::init<bool, bool, bool>(), py::arg("over_temperature"),
           py::arg("over_voltage"), py::arg("rotor_locked"))
      .def_readwrite("over_temperature", &LedTriggers::over_temperature)
      .def_readwrite("over_voltage", &LedTriggers::over_voltage)
      .def_readwrite("rotor_locked", &LedTriggers::rotor_locked)
      .def("to_mask", [](const LedTriggers& t) { return static_cast<int>(t.toMask()); })
      .def_static("from_mask",
                  [](int mask) { return LedTriggers::fromMask(static_cast<std::uint8_t>(mask)); })
      .def("__eq__", [](const LedTriggers& a, const LedTriggers& b) { return a == b; })
      .def("__repr__", [](const LedTriggers& t) {
        return std::string("LedTriggers(over_temperature=") +
               (t.over_temperature ? "True" : "False") + ", over_voltage=" +
               (t.over_voltage ? "True" : "False") + ", rotor_locked=" +
               (t.rotor_locked ? "True" : "False") + ")";
      });

  py::class_<Telemetry>(m, "Telemetry",
                        "Position + supply voltage + die temperature snapshot.")
      .def_property_readonly("id", [](const Telemetry& t) { return toInt(t.id); })
      .def_readonly("angle_deg", &Telemetry::angle_deg)
      .def_readonly("vin_mv", &Telemetry::vin_mv)
      .def_readonly("temp_c", &Telemetry::temp_c)
      .def_readonly("valid", &Telemetry::valid)
      .def("__repr__", [](const Telemetry& t) {
        std::ostringstream oss;
        oss << "Telemetry(id=" << toInt(t.id) << ", angle_deg=" << t.angle_deg
            << ", vin_mv=" << t.vin_mv << ", temp_c=" << t.temp_c
            << ", valid=" << (t.valid ? "True" : "False") << ")";
        return oss.str();
      });

  // --- SerialBus -----------------------------------------------------------
  py::class_<SerialBus> bus(m, "SerialBus",
                            "Linux POSIX serial bus for the classic servo protocol.\n\n"
                            "Wraps a termios raw 8N1 handle driven by poll(2), with\n"
                            "frame resync, echo rejection and single-syscall batching.");

  bus.def(py::init<>(), "Create an unopened bus; call open() afterwards.")
      .def(py::init([](const std::string& device, int baudrate, int read_timeout_ms,
                      int write_timeout_ms, bool low_latency, ErrorMode error_mode) {
             BusConfig cfg;
             cfg.device = device;
             cfg.baudrate = static_cast<std::uint32_t>(baudrate);
             cfg.read_timeout_ms = read_timeout_ms;
             cfg.write_timeout_ms = write_timeout_ms;
             cfg.enable_low_latency = low_latency;
             cfg.error_mode = error_mode;
             auto handle = std::make_unique<SerialBus>(cfg);
             if (!device.empty()) {
               blocking_void([&] { handle->open(device); });
             }
             return handle;
           }),
           py::arg("device") = std::string(), py::arg("baudrate") = 115200,
           py::arg("read_timeout_ms") = 20, py::arg("write_timeout_ms") = 100,
           py::arg("low_latency") = true, py::arg("error_mode") = ErrorMode::kStrict,
           "Open `device` eagerly. Pass an empty device to defer opening.")
      .def_property_readonly("config", &SerialBus::config,
                             "The active BusConfig (a copy; see update_config).")
      .def("update_config",
           [](SerialBus& self, const BusConfig& config) {
             self.setConfig(config);
             return self.applyConfig();
           },
           py::arg("config"),
           "Replace the BusConfig and re-apply it to a live descriptor.")
      .def_property(
          "baudrate",
          [](const SerialBus& self) { return static_cast<int>(self.config().baudrate); },
          [](SerialBus& self, int baudrate) {
            BusConfig cfg = self.config();
            cfg.baudrate = static_cast<std::uint32_t>(baudrate);
            self.setConfig(cfg);
            self.applyConfig();
          },
          "Line speed; the classic bus is 115200 8N1.")
      // -- lifecycle
      .def("open",
           [](SerialBus& self, const std::string& device) {
             blocking_void([&] { self.open(device); });
             return self.isOpen();
           },
           py::arg("device"), "Open the character device; returns True when it is live.")
      .def("close", [](SerialBus& self) { self.close(); },
           "Release the file descriptor. Safe to call more than once.")
      .def("is_open", &SerialBus::isOpen, "True while the port is held open.")
      .def_property_readonly("device", &SerialBus::device, "Character device path.")
      .def_property_readonly("transport", &SerialBus::transportName,
                             "Active transport backend name ('posix').")
      .def("last_hint", &SerialBus::lastHint,
           "Advisory note from the last open(), e.g. a refused low-latency ioctl.")
      .def("last_error", &SerialBus::lastError, "Message from the last failure.")
      .def("last_error_code", &SerialBus::lastErrorCode, "ErrorCode of the last failure.")
      .def("clear_last_error", &SerialBus::clearLastError)
      .def("stats", &SerialBus::stats, py::return_value_policy::copy)
      .def("reset_stats", &SerialBus::resetStats)
      // -- policy
      .def_property("read_timeout_ms", &SerialBus::readTimeoutMs,
                    &SerialBus::setReadTimeoutMs,
                    "poll(2) budget for one servo reply, in milliseconds.")
      .def_property("write_timeout_ms", &SerialBus::writeTimeoutMs,
                    &SerialBus::setWriteTimeoutMs,
                    "poll(2) budget for draining our own TX bytes.")
      .def_property("error_mode", &SerialBus::errorMode, &SerialBus::setErrorMode,
                    "ErrorMode.STRICT or ErrorMode.NON_THROWING.")
      .def("set_poll_hardware", &SerialBus::setPollHardware, py::arg("poll_hardware"),
           "false == non-throwing real-time mode (no exception on a dead servo).")
      .def("apply_config", &SerialBus::applyConfig,
           "Re-apply the current BusConfig to the live file descriptor.")
      // -- raw frames, for experiments and tests
      .def("command",
           [](SerialBus& self, int id, int opcode, const std::vector<int>& args) {
             const Packet frame = rawPacket(id, opcode, args);
             return blocking_value<std::size_t>([&] {
               return self.writeFrames({frame});
             });
           },
           py::arg("id"), py::arg("opcode"), py::arg("args") = std::vector<int>(),
           "Send a raw frame (fire and forget). Returns bytes written.")
      .def("query",
           [](SerialBus& self, int id, int opcode, const std::vector<int>& args,
              int timeout_ms) {
             toUnicastId(id);
             const Packet frame = rawPacket(id, opcode, args);
             return blocking_value<std::vector<int>>([&] {
               Response response;
               if (!self.tryTransact(frame, &response, 0, timeout_ms) || !response.ok()) {
                 return std::vector<int>{};
               }
               return toInts(response.data);
             });
           },
           py::arg("id"), py::arg("opcode"), py::arg("args") = std::vector<int>(),
           py::arg("timeout_ms") = -1,
           "Ask one servo for `opcode` and return its data bytes ([] on failure).")
      // -- batch primitives
      .def("move_sync",
           [](SerialBus& self, const std::vector<int>& ids,
              const std::vector<float>& angles_deg, int time_ms, bool start,
              const py::object& config) {
             const ServoConfig cfg = configOrDefault(config);
             const auto duration = moveTime(time_ms);
             if (ids.size() != angles_deg.size()) {
               raiseArgument("ids and angles must have the same length",
                             HiwonderBus::kNoServoId);
             }
             std::vector<std::uint8_t> targets = toIds(ids);
             std::vector<std::uint16_t> ticks;
             ticks.reserve(angles_deg.size());
             for (std::size_t i = 0; i < angles_deg.size(); ++i) {
               if (!(angles_deg[i] >= cfg.min_angle_deg && angles_deg[i] <= cfg.max_angle_deg)) {
                 raiseArgument("angle out of range for id " + std::to_string(ids[i]), ids[i]);
               }
               ticks.push_back(HiwonderBus::angleToTicks(angles_deg[i], cfg));
             }
             return blocking_value<std::size_t>([&] {
               ErrorCode code = ErrorCode::kOk;
               const auto written = self.queueMoves(targets, ticks, duration, start, &code);
               checkBatchError(self, code);
               return written;
             });
           },
           py::arg("ids"), py::arg("angles"), py::arg("time_ms") = 0,
           py::arg("start") = true, py::arg("config") = py::none(),
           "Queue synchronized moves for many servos and (optionally) broadcast the\n"
           "start. Everything -- including the 0xFE start frame -- leaves in one\n"
           "write(2) call. Returns bytes written.")
      .def("set_torque",
           [](SerialBus& self, const std::vector<int>& ids, bool enable) {
             const std::vector<std::uint8_t> targets = toIds(ids);
             return blocking_value<std::size_t>([&] {
               ErrorCode code = ErrorCode::kOk;
               const auto written = self.setTorqueBatch(targets, enable, &code);
               checkBatchError(self, code);
               return written;
             });
           },
           py::arg("ids"), py::arg("enable"),
           "Enable/disable torque on many servos in a single write(2).")
      .def("read_angles",
           [](SerialBus& self, const std::vector<int>& ids, const py::object& config) {
             const ServoConfig cfg = configOrDefault(config);
             const std::vector<std::uint8_t> targets = toIds(ids);
             return blocking_value<std::vector<float>>([&] {
               std::vector<float> angles;
               angles.reserve(targets.size());
               for (std::uint8_t id : targets) {
                 Response response;
                 const Packet frame = Packet::makeRead(id, Command::kReadPhysicalAngle);
                 angles.push_back(self.tryTransact(frame, &response, 2) && response.ok()
                                      ? response.physicalAngle(cfg)
                                      : quietNaN());
               }
               return angles;
             });
           },
           py::arg("ids"), py::arg("config") = py::none(),
           "Sweep physical angles. Failed reads come back as NaN so indices stay\n"
           "aligned with `ids`.")
      .def("read_telemetry",
           [](SerialBus& self, const std::vector<int>& ids, const py::object& config) {
             const ServoConfig cfg = configOrDefault(config);
             const std::vector<std::uint8_t> targets = toIds(ids);
             return blocking_value<std::vector<Telemetry>>([&] {
               std::vector<Telemetry> out;
               out.reserve(targets.size());
               for (std::uint8_t id : targets) {
                 Telemetry snapshot;
                 snapshot.id = id;
                 Response angle, vin, temp;
                 const bool angle_ok = self.tryTransact(
                     Packet::makeRead(id, Command::kReadPhysicalAngle), &angle, 2);
                 const bool vin_ok =
                     self.tryTransact(Packet::makeRead(id, Command::kReadVin), &vin, 2);
                 const bool temp_ok =
                     self.tryTransact(Packet::makeRead(id, Command::kReadTemp), &temp, 1);
                 snapshot.angle_deg = angle_ok && angle.ok() ? angle.physicalAngle(cfg) : quietNaN();
                 snapshot.vin_mv = vin_ok && vin.ok() ? vin.voltage() : 0;
                 snapshot.temp_c = temp_ok && temp.ok() ? temp[0] : -1;
                 snapshot.valid = angle_ok && vin_ok && temp_ok && angle.ok() && vin.ok() &&
                                  temp.ok();
                 out.push_back(snapshot);
               }
               return out;
             });
           },
           py::arg("ids"), py::arg("config") = py::none(),
           "Read angle, supply voltage and temperature for every id.")
      .def("broadcast_move_start",
           [](SerialBus& self) {
             blocking_void([&] { self.moveStart(); });
             return true;
           },
           "Broadcast SERVO_MOVE_START (0xFE, opcode 11) to release queued waits.")
      .def("broadcast_move_stop",
           [](SerialBus& self) {
             blocking_void([&] { self.moveStop(); });
             return true;
           },
           "Broadcast SERVO_MOVE_STOP (0xFE, opcode 12).")
      .def("flush_input",
           [](SerialBus& self) { return blocking_value<bool>([&] { return self.tryFlushInput(); }); },
           "tcflush(TCIFLUSH): drop stale received bytes.")
      .def("flush_output",
           [](SerialBus& self) {
             return blocking_value<bool>([&] { return self.tryFlushOutput(); });
           },
           "tcflush(TCOFLUSH): drop pending transmit bytes.")
      .def("scan_ids",
           [](SerialBus& self, int first, int last, int probe, int timeout_ms) {
             const auto begin = toUnicastId(first);
             const auto end = toUnicastId(last);
             if (probe < 0 || probe > 255) {
               raiseArgument("probe opcode must be a byte", HiwonderBus::kNoServoId);
             }
             const std::vector<std::uint8_t> found = blocking_value<std::vector<std::uint8_t>>([&] {
               return self.scanIds(begin, end, static_cast<Command>(probe), timeout_ms);
             });
             return toInts(found);
           },
           py::arg("first") = 0, py::arg("last") = 253,
           py::arg("probe") = static_cast<int>(Command::kReadPhysicalAngle),
           py::arg("timeout_ms") = 15,
           "Walk an id range and return every servo that answers.");

  // --- Servo ---------------------------------------------------------------
  py::class_<Servo> servo(m, "Servo",
                          "One classic-protocol servo, addressed by id.\n\n"
                          "Angles are always degrees; tick conversion is internal.\n"
                          "Getters are cached unless you pass poll_hardware=True.");

  servo.def(py::init([](SerialBus& bus, int id, const py::object& config) {
              if (id < 0 || id > 253) {
                raiseArgument("Servo ID must be between 0 and 253 inclusive",
                              HiwonderBus::kNoServoId);
              }
              return new Servo(bus, static_cast<std::uint8_t>(id), configOrDefault(config));
            }),
            py::arg("bus"), py::arg("id"), py::arg("config") = py::none(),
            py::keep_alive<1, 2>(), "Bind to `bus` (kept alive) at address `id`.")
      // -- identity
      .def_property_readonly("id", [](const Servo& self) { return toInt(self.id()); },
                             "Servo address 0..253.")
      .def_property_readonly("config", &Servo::config, "This servo's geometry.")
      .def("set_config", &Servo::setConfig, py::arg("config"),
           "Override the angle/tick geometry.")
      .def("bus", [](const Servo& self) -> SerialBus& { return self.bus(); },
           py::return_value_policy::reference_internal,
           "The bus this servo speaks to.")
      .def("initialize",
           [](Servo& self, bool disable_torque) {
             blocking_void([&] { self.initialize(disable_torque); });
           },
           py::arg("disable_torque") = false,
           "Pull every hardware register into the local mirror, then set torque.\n"
           "Mirrors LX16A.__init__ / LX16A::initialize().")
      .def("refresh",
           [](Servo& self) {
             return blocking_value<bool>([&] {
               ErrorCode code = ErrorCode::kOk;
               const bool ok = self.refresh(&code);
               return ok && code == ErrorCode::kOk;
             });
           },
           "Re-read every mirrored register; False when the servo is unreachable.")
      .def("get_id",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<int>(
                 [&] { return toInt(self.getID(poll_hardware)); });
           },
           py::arg("poll_hardware") = false,
           "Cached by default; poll_hardware=True asks the servo.")
      .def("set_id",
           [](Servo& self, int new_id) {
             if (new_id < 0 || new_id > 253) {
               raiseArgument("Servo ID must be between 0 and 253 inclusive", self.id());
             }
             blocking_void([&] { self.setID(static_cast<std::uint8_t>(new_id)); });
           },
           py::arg("new_id"),
           "Assign a new firmware id (this object follows it).")
      // -- motion
      .def("move",
           [](Servo& self, float angle, int time_ms, bool relative, bool wait) {
             const auto duration = moveTime(time_ms, self.id());
             blocking_void([&] {
               if (relative) self.moveRelative(angle, duration, wait);
               else self.move(angle, duration, wait);
             });
           },
           py::arg("angle"), py::arg("time") = 0, py::arg("relative") = false,
           py::arg("wait") = false,
           "Command a move to `angle` degrees over `time` ms.\n"
           "wait=True queues it (opcode 7) until move_start().")
      .def("move_to",
           [](Servo& self, float angle, int time_ms, bool wait) {
             const auto duration = moveTime(time_ms, self.id());
             blocking_void([&] {
               self.moveTo(angle, duration,
                           wait ? Servo::MoveMode::kWait : Servo::MoveMode::kInstant);
             });
           },
           py::arg("angle"), py::arg("time") = 0, py::arg("wait") = false,
           "Type-safe spelling of move().")
      .def("move_relative",
           [](Servo& self, float angle, int time_ms, bool wait) {
             const auto duration = moveTime(time_ms, self.id());
             blocking_void([&] {
               self.moveRelative(angle, duration, wait);
             });
           },
           py::arg("angle"), py::arg("time") = 0, py::arg("wait") = false,
           "Move relative to the last commanded angle.")
      .def("move_start",
           [](Servo& self) { blocking_void([&] { self.moveStart(); }); },
           "Release the target queued by move(..., wait=True).")
      .def("move_stop",
           [](Servo& self) { blocking_void([&] { self.moveStop(); }); },
           "Halt the current movement and re-sync the commanded angle.")
      .def("get_physical_angle",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<float>(
                 [&] { return self.getPhysicalAngle(poll_hardware); });
           },
           py::arg("poll_hardware") = true,
           "Shaft position in degrees from the potentiometer.")
      .def("get_commanded_angle", &Servo::getCommandedAngle,
           "Last angle we commanded, straight from the mirror (no wire traffic).")
      .def("get_waiting_angle",
           [](const Servo& self) {
             try {
               return self.getWaitingAngle();
             } catch (const HiwonderBus::ServoError& e) {
               raiseFrom(e);
             }
           },
           "Target queued by move(..., wait=True). Raises if none is queued.")
      .def("is_waiting_for_move", &Servo::isWaitingForMove)
      .def("get_last_instant_move",
           [](Servo& self) {
             std::pair<float, std::uint16_t> result{quietNaN(), 0};
             blocking_void([&] { result = self.getLastInstantMove(); });
             return py::make_tuple(result.first, static_cast<int>(result.second));
           },
           "Read (angle, time_ms) of the last instant move (opcode 2).")
      .def("get_last_delayed_move",
           [](Servo& self) {
             std::pair<float, std::uint16_t> result{quietNaN(), 0};
             blocking_void([&] { result = self.getLastDelayedMove(); });
             return py::make_tuple(result.first, static_cast<int>(result.second));
           },
           "Read (angle, time_ms) of the last queued move (opcode 8).")
      // -- torque & mode
      .def("enable_torque", [](Servo& self) { blocking_void([&] { self.enableTorque(); }); })
      .def("disable_torque",
           [](Servo& self) { blocking_void([&] { self.disableTorque(); }); })
      .def("is_torque_enabled",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<bool>([&] { return self.isTorqueEnabled(poll_hardware); });
           },
           py::arg("poll_hardware") = false)
      .def("set_motor_mode",
           [](Servo& self, int speed) {
             if (speed < -HiwonderBus::kMaxMotorSpeed || speed > HiwonderBus::kMaxMotorSpeed) {
               raiseArgument("motor speed must be in -1000..1000", self.id());
             }
             blocking_void([&] { self.setMotorMode(static_cast<std::int16_t>(speed)); });
           },
           py::arg("speed"), "Continuous-spin mode at `speed` (-1000..1000).")
      .def("set_servo_mode",
           [](Servo& self) { blocking_void([&] { self.setServoMode(); }); },
           "Return to position control.")
      .def("is_motor_mode",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<bool>([&] { return self.isMotorMode(poll_hardware); });
           },
           py::arg("poll_hardware") = false)
      .def("get_motor_speed",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<int>(
                 [&] { return self.getMotorSpeed(poll_hardware); });
           },
           py::arg("poll_hardware") = false,
           "Raises if the servo is not in motor mode (PyLX-16A parity).")
      // -- calibration & limits
      .def("set_angle_offset",
           [](Servo& self, float offset, bool permanent) {
             blocking_void([&] { self.setAngleOffset(offset, permanent); });
           },
           py::arg("offset"), py::arg("permanent") = false,
           "Shift commanded zero by +/-30 deg; permanent=True commits to EEPROM.")
      .def("get_angle_offset",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<float>([&] { return self.getAngleOffset(poll_hardware); });
           },
           py::arg("poll_hardware") = false)
      .def("set_angle_limits",
           [](Servo& self, float min_deg, float max_deg) {
             blocking_void([&] { self.setAngleLimits(min_deg, max_deg); });
           },
           py::arg("min"), py::arg("max"), "Range of motion bounds in degrees.")
      .def("get_angle_limits",
           [](Servo& self, bool poll_hardware) {
             const HiwonderBus::AngleLimits limits = blocking_value<HiwonderBus::AngleLimits>(
                 [&] { return self.getAngleLimits(poll_hardware); });
             return py::make_tuple(limits.min, limits.max);
           },
           py::arg("poll_hardware") = false, "Returns (min_deg, max_deg).")
      .def("set_vin_limits",
           [](Servo& self, int min_mv, int max_mv) {
             blocking_void([&] { self.setVinLimits(min_mv, max_mv); });
           },
           py::arg("min"), py::arg("max"), "Voltage protection window (mV).")
      .def("get_vin_limits",
           [](Servo& self, bool poll_hardware) {
             const HiwonderBus::VinLimits limits = blocking_value<HiwonderBus::VinLimits>(
                 [&] { return self.getVinLimits(poll_hardware); });
             return py::make_tuple(limits.min, limits.max);
           },
           py::arg("poll_hardware") = false, "Returns (min_mv, max_mv).")
      .def("set_temp_limit",
           [](Servo& self, int max_temp_c) {
             blocking_void([&] { self.setTempLimit(max_temp_c); });
           },
           py::arg("max_temp_c"), "Overheat ceiling in deg C (50..100).")
      .def("get_temp_limit",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<int>([&] { return self.getTempLimit(poll_hardware); });
           },
           py::arg("poll_hardware") = false)
      // -- telemetry
      .def("get_temp", [](Servo& self) { return blocking_value<int>([&] { return self.getTemp(); }); })
      .def("get_vin", [](Servo& self) { return blocking_value<int>([&] { return self.getVin(); }); })
      .def("read_telemetry",
           [](Servo& self) { return blocking_value<Telemetry>([&] { return self.readTelemetry(); }); },
           "One snapshot: angle, voltage and temperature.")
      // -- LED
      .def("led_power_on", [](Servo& self) { blocking_void([&] { self.ledPowerOn(); }); })
      .def("led_power_off", [](Servo& self) { blocking_void([&] { self.ledPowerOff(); }); })
      .def("is_led_power_on",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<bool>([&] { return self.isLedPowerOn(poll_hardware); });
           },
           py::arg("poll_hardware") = false)
      .def("set_led_error_triggers",
           [](Servo& self, bool over_temperature, bool over_voltage, bool rotor_locked) {
             blocking_void([&] {
               self.setLedErrorTriggers(
                   LedTriggers{over_temperature, over_voltage, rotor_locked});
             });
           },
           py::arg("over_temperature"), py::arg("over_voltage"), py::arg("rotor_locked"))
      .def("get_led_error_triggers",
           [](Servo& self, bool poll_hardware) {
             const LedTriggers triggers = blocking_value<LedTriggers>(
                 [&] { return self.getLedErrorTriggers(poll_hardware); });
             return py::make_tuple(triggers.over_temperature, triggers.over_voltage,
                                   triggers.rotor_locked);
           },
           py::arg("poll_hardware") = false,
           "Returns (over_temperature, over_voltage, rotor_locked).")
      // -- health / raw access
      .def("healthy", &Servo::healthy, "False after a failed wire operation.")
      .def("last_error", &Servo::lastError, "ErrorCode of the last failure.")
      .def("last_message", &Servo::lastMessage, "Human-readable last failure.")
      .def("try_move",
           [](Servo& self, float angle, int time_ms, bool relative, bool wait) {
             return blocking_value<std::pair<bool, ErrorCode>>([&] {
               ErrorCode code = ErrorCode::kOk;
               // An invalid duration is passed as a native-invalid sentinel,
               // never narrowed into an accidentally valid motion duration.
               const auto duration = time_ms < 0 || time_ms > HiwonderBus::kMaxMoveTimeMs
                   ? static_cast<std::uint16_t>(HiwonderBus::kMaxMoveTimeMs + 1)
                   : static_cast<std::uint16_t>(time_ms);
               const bool ok = relative ? self.tryMoveRelative(angle, duration, wait, &code)
                                        : self.tryMove(angle, duration, wait, &code);
               return std::make_pair(ok, code);
             });
           },
           py::arg("angle"), py::arg("time") = 0, py::arg("relative") = false,
           py::arg("wait") = false,
           "Exception-free move; returns (succeeded, ErrorCode).")
      .def("try_get_physical_angle",
           [](Servo& self, bool poll_hardware) {
             return blocking_value<std::pair<bool, float>>([&] {
               float angle = 0.0f;
               const bool ok = self.tryGetPhysicalAngle(&angle, poll_hardware);
               return std::make_pair(ok, angle);
             });
           },
           py::arg("poll_hardware") = true)
      .def("try_set_torque",
           [](Servo& self, bool enable) {
             return blocking_value<std::pair<bool, ErrorCode>>([&] {
               ErrorCode code = ErrorCode::kOk;
               const bool ok = self.trySetTorque(enable, &code);
               return std::make_pair(ok, code);
             });
           },
           py::arg("enable"))
      .def("angle_to_ticks",
           [](const Servo& self, float angle) {
             return static_cast<int>(self.angleToTicks(angle));
           },
           py::arg("angle_deg"), "Degrees -> ticks using this servo's geometry.")
      .def("ticks_to_angle",
           [](const Servo& self, std::uint16_t ticks) { return self.ticksToAngle(ticks); },
           py::arg("ticks"), "Ticks -> degrees using this servo's geometry.");

  // --- Module-level batch helpers over Servo objects -----------------------
  m.def("move_sync",
        [](SerialBus& bus, const std::vector<Servo*>& servos,
           const std::vector<float>& angles, int time_ms, bool start) {
          validateServos(bus, servos);
          const auto duration = moveTime(time_ms);
          return blocking_value<std::size_t>([&] {
            ErrorCode code = ErrorCode::kOk;
            const auto written = Servo::moveSync(bus, servos, angles, duration, start, &code);
            checkBatchError(bus, code);
            return written;
          });
        },
        py::arg("bus"), py::arg("servos"), py::arg("angles"), py::arg("time_ms") = 0,
        py::arg("start") = true,
        "Synchronized move across the given servos, queued with opcode 7 and\n"
        "released by one broadcast start -- all in a single write(2).");

  m.def("read_angles",
        [](SerialBus& bus, const std::vector<Servo*>& servos, bool poll_hardware) {
          validateServos(bus, servos);
          return blocking_value<std::vector<float>>(
              [&] { return Servo::readAngles(bus, servos, poll_hardware); });
        },
        py::arg("bus"), py::arg("servos"), py::arg("poll_hardware") = true,
        "Sweep physical angles across many servos.");

  m.def("read_telemetry",
        [](SerialBus& bus, const std::vector<Servo*>& servos) {
          validateServos(bus, servos);
          return blocking_value<std::vector<Telemetry>>(
              [&] { return Servo::readTelemetry(bus, servos); });
        },
        py::arg("bus"), py::arg("servos"),
        "Angle + voltage + temperature sweep across many servos.");

  m.def("set_torque",
        [](SerialBus& bus, const std::vector<Servo*>& servos, bool enable) {
          validateServos(bus, servos);
          return blocking_value<std::size_t>([&] {
            ErrorCode code = ErrorCode::kOk;
            const auto written = Servo::setTorque(bus, servos, enable, &code);
            checkBatchError(bus, code);
            return written;
          });
        },
        py::arg("bus"), py::arg("servos"), py::arg("enable"),
        "Torque enable/disable for many servos in one write(2).");

  // --- Protocol utilities --------------------------------------------------
  m.def("angle_to_ticks", &HiwonderBus::angleToTicks, py::arg("angle_deg"),
        py::arg("config") = ServoConfig::classic(), "Degrees -> ticks.");
  m.def("ticks_to_angle", &HiwonderBus::ticksToAngle, py::arg("ticks"),
        py::arg("config") = ServoConfig::classic(), "Ticks -> degrees.");
  m.def("checksum",
        [](const std::vector<int>& frame) {
          std::vector<std::uint8_t> bytes;
          bytes.reserve(frame.size());
          for (int b : frame) bytes.push_back(static_cast<std::uint8_t>(b));
          return static_cast<int>(HiwonderBus::calculateChecksum(bytes.data(), bytes.size()));
        },
        py::arg("frame"), "Inverted one's-complement checksum over a full frame.");
  m.def("build_packet",
        [](int id, int opcode, const std::vector<int>& args) {
          return toInts(rawPacket(id, opcode, args).raw());
        },
        py::arg("id"), py::arg("opcode"), py::arg("args") = std::vector<int>(),
        "Build one framed request (header + checksum) as a list of ints.");
  m.def("parse_response",
        [](const std::vector<int>& frame) {
          std::vector<std::uint8_t> bytes;
          bytes.reserve(frame.size());
          for (int b : frame) bytes.push_back(static_cast<std::uint8_t>(b));
          std::size_t consumed = 0;
          Response response;
          const HiwonderBus::FrameResult result = HiwonderBus::tryParseFrame(
              bytes.data(), bytes.size(), &consumed, &response);
          return py::make_tuple(static_cast<int>(result), toInt(response.id),
                                static_cast<int>(response.command), toInts(response.data),
                                response.ok());
        },
        py::arg("frame"),
        "Parse a frame: (result, id, command, data, ok). Result is 0=complete,\n"
        "1=incomplete, 2=bad checksum, 3=bad header.");

  m.def("version", &HiwonderBus::versionString,
        "Library banner, for logs and bug reports.");
  m.def("sleep_ms",
        [](int ms) {
          py::gil_scoped_release guard;
          std::this_thread::sleep_for(std::chrono::milliseconds(ms < 0 ? 0 : ms));
        },
        py::arg("ms"), "Helper used by the example control loops (releases the GIL).");

  // ---------------------------------------------------------------------------
  // In-process bus model, for tests and CI with no hardware attached
  // ---------------------------------------------------------------------------
  m.def("make_simulated_bus",
        [](const std::vector<int>& ids, const py::object& /*seed*/) {
          std::vector<std::uint8_t> targets = toIds(ids);
          auto simulator =
              std::unique_ptr<HiwonderBus::testing::SimulatedTransport>(
                  new HiwonderBus::testing::SimulatedTransport(targets));
          for (std::uint8_t id : targets) {
            if (HiwonderBus::testing::SimulatedServo* model = simulator->servo(id)) {
              model->physical_angle_ticks = static_cast<std::int16_t>(
                  (id * 137) % 1000);
              model->vin_mv = static_cast<std::uint16_t>(7350 - 11 * id);
              model->temp_c = static_cast<std::uint8_t>(32 + (id % 9));
              model->torque_enabled = true;
            }
          }
          BusConfig config;
          config.device = "simulator";
          config.read_timeout_ms = 50;
          auto* handle = new SerialBus(std::move(simulator), config);
          handle->open("simulator");
          return handle;
        },
        py::arg("ids"), py::arg("seed") = py::none(),
        "Return a SerialBus backed by the in-process servo model instead of a UART.\n"
        "An idealized register model, not a simulation of UART timing or physical motion.");

  m.def("simulate_disconnect",
        [](SerialBus& bus, int id, bool present) {
          const auto target = toUnicastId(id);
          return blocking_value<bool>([&] {
            bus.withTransport([&](HiwonderBus::Transport& transport) {
              auto* simulator = dynamic_cast<HiwonderBus::testing::SimulatedTransport*>(&transport);
              if (!simulator) {
                throw HiwonderBus::ServoArgumentError(
                    "this bus is not backed by the simulator", HiwonderBus::kNoServoId);
              }
              if (present) simulator->connect(target);
              else simulator->disconnect(target);
            });
            return true;
          });
        },
        py::arg("bus"), py::arg("id"), py::arg("present") = false,
        "Plug or unplug a simulated servo, so failure paths can be exercised.");

  // --- C++-style aliases (type aliases, not new classes) -------------------
  // HiwonderBus::LX16A == HiwonderBus::Servo, so these are the same class
  // object in Python too and isinstance() works across both spellings.
  m.attr("LX16A") = servo;
  m.attr("LX15D") = servo;
  m.attr("LX224") = servo;
  m.attr("LX225") = servo;
  m.attr("HTS35H") = servo;
  m.attr("HV206") = servo;
  m.attr("LX16ABus") = bus;
  m.attr("Bus") = bus;
}
