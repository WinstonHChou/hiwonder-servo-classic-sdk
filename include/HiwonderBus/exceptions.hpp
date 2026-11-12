// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// exceptions.hpp
//   The error hierarchy. Mirrors ethanlipson/PyLX-16A one-for-one so that
//   `except ServoError` keeps working, and adds a machine-readable ErrorCode
//   for the non-throwing real-time mode.
//
#ifndef HIWONDERBUS_EXCEPTIONS_HPP
#define HIWONDERBUS_EXCEPTIONS_HPP

#include <cstdint>
#include <stdexcept>
#include <string>

namespace HiwonderBus {

/// Machine-readable error taxonomy.
///
/// In `ErrorMode::NonThrowing` these values are returned from the `try*`
/// entry points and recorded in `SerialBus::lastErrorCode()` instead of being
/// thrown, so a dead servo on a 10-DOF biped never unwinds a control loop.
enum class ErrorCode : std::uint8_t {
  kOk = 0,
  kNotOpen,           ///< The port was never opened, or has been closed.
  kTimeout,           ///< Nothing arrived before the read deadline.
  kShortRead,         ///< Fewer bytes than a whole frame arrived.
  kBadChecksum,       ///< Frame checksum mismatch (line noise / collisions).
  kBadFrame,          ///< Unparseable framing, bad inner length field.
  kFirmwareStatus,    ///< Reserved compatibility code; unused by classic replies.
  kInvalidArgument,   ///< Out-of-range angle, id, time, speed, limit, ...
  kInvalidState,      ///< Command illegal in the current mode (motor mode,
                      ///< torque off, no queued move, ...).
  kIoError,           ///< write(2)/poll(2)/tcsetattr(3) failed.
  kPortBusy,          ///< Another process holds the port (EBUSY/EAGAIN).
  kUnsupported,       ///< Operation not available on this backend/mode.
};

/// Human-readable name, e.g. for logs and for exception messages.
inline const char* toString(ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::kOk: return "ok";
    case ErrorCode::kNotOpen: return "port not open";
    case ErrorCode::kTimeout: return "timeout";
    case ErrorCode::kShortRead: return "short read";
    case ErrorCode::kBadChecksum: return "bad checksum";
    case ErrorCode::kBadFrame: return "malformed frame";
    case ErrorCode::kFirmwareStatus: return "firmware reported an error";
    case ErrorCode::kInvalidArgument: return "invalid argument";
    case ErrorCode::kInvalidState: return "invalid state";
    case ErrorCode::kIoError: return "i/o error";
    case ErrorCode::kPortBusy: return "port busy";
    case ErrorCode::kUnsupported: return "unsupported";
  }
  return "unknown error";
}

/// True when retrying the exact same transaction has a chance of working
/// (i.e. the failure was transient line trouble rather than user error).
inline bool isTransient(ErrorCode code) noexcept {
  return code == ErrorCode::kTimeout || code == ErrorCode::kShortRead ||
         code == ErrorCode::kBadChecksum || code == ErrorCode::kBadFrame;
}

inline bool isHardwareError(ErrorCode code) noexcept {
  return code != ErrorCode::kOk && code != ErrorCode::kInvalidArgument &&
         code != ErrorCode::kInvalidState && code != ErrorCode::kUnsupported;
}

/// Sentinel for exceptions that are not attributable to one servo.
constexpr int kNoServoId = -1;

/// Base class of everything this SDK throws (or reports as an ErrorCode).
///
/// Every servo-attributable failure carries the offending id in `id()`,
/// matching PyLX-16A's `id_` attribute.
class ServoError : public std::runtime_error {
 public:
  explicit ServoError(const std::string& message, int servo_id = kNoServoId)
      : std::runtime_error(message), servo_id_(servo_id) {}

  /// Servo the failure came from, or `kNoServoId` for bus-level errors.
  ///
  /// The Python bindings expose this as the `id_` attribute, matching
  /// ethanlipson/PyLX-16A's `ServoError.id_`.
  int id() const noexcept { return servo_id_; }

  /// Prefix like `"Servo 7: "` when an id is known, empty otherwise. Used to
  /// build messages that read the same as the Python library's.
  static std::string formatMessage(const std::string& what, int servo_id) {
    if (servo_id == kNoServoId) return what;
    return "Servo " + std::to_string(servo_id) + ": " + what;
  }

 protected:
  int servo_id_;
};

/// The servo did not answer before the read deadline.
class ServoTimeoutError : public ServoError {
 public:
  explicit ServoTimeoutError(const std::string& message, int servo_id = kNoServoId)
      : ServoError(message, servo_id) {}
};

/// A reply arrived but its checksum was wrong.
class ServoChecksumError : public ServoError {
 public:
  explicit ServoChecksumError(const std::string& message, int servo_id = kNoServoId)
      : ServoError(message, servo_id) {}
};

/// An argument was outside the range the firmware accepts (never clamped in
/// strict mode -- PyLX-16A raises instead of silently doing something else).
class ServoArgumentError : public ServoError {
 public:
  explicit ServoArgumentError(const std::string& message, int servo_id = kNoServoId)
      : ServoError(message, servo_id) {}
};

/// The command was not legal given the servo's current state -- e.g. `move()`
/// while torque is off, or `getMotorSpeed()` while in servo mode.
class ServoLogicalError : public ServoError {
 public:
  explicit ServoLogicalError(const std::string& message, int servo_id = kNoServoId)
      : ServoError(message, servo_id) {}
};

/// The kernel rejected a serial syscall.
class ServoIOError : public ServoError {
 public:
  explicit ServoIOError(const std::string& message, int servo_id = kNoServoId)
      : ServoError(message, servo_id) {}
};

/// Reserved compatibility exception; classic replies have no status byte.
class ServoFirmwareError : public ServoError {
 public:
  explicit ServoFirmwareError(const std::string& message, int servo_id = kNoServoId)
      : ServoError(message, servo_id) {}
};

namespace detail {

/// Translate an ErrorCode into the matching exception type and throw it.
/// `[[noreturn]]` keeps the compiler's control-flow analysis quiet.
[[noreturn]] inline void throwError(ErrorCode code, const std::string& message,
                                    int servo_id = kNoServoId) {
  const std::string full = ServoError::formatMessage(message, servo_id);
  switch (code) {
    case ErrorCode::kTimeout: throw ServoTimeoutError(full, servo_id);
    case ErrorCode::kShortRead: throw ServoTimeoutError(full, servo_id);
    case ErrorCode::kBadChecksum: throw ServoChecksumError(full, servo_id);
    case ErrorCode::kBadFrame: throw ServoChecksumError(full, servo_id);
    case ErrorCode::kFirmwareStatus: throw ServoFirmwareError(full, servo_id);
    case ErrorCode::kInvalidArgument: throw ServoArgumentError(full, servo_id);
    case ErrorCode::kInvalidState: throw ServoLogicalError(full, servo_id);
    case ErrorCode::kIoError: throw ServoIOError(full, servo_id);
    case ErrorCode::kPortBusy: throw ServoIOError(full, servo_id);
    case ErrorCode::kNotOpen: throw ServoIOError(full, servo_id);
    case ErrorCode::kUnsupported: throw ServoLogicalError(full, servo_id);
    case ErrorCode::kOk: break;
  }
  throw ServoError(full, servo_id);
}

}  // namespace detail

}  // namespace HiwonderBus

#endif  // HIWONDERBUS_EXCEPTIONS_HPP
