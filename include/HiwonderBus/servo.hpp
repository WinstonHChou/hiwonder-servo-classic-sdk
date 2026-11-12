// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// servo.hpp
//   HiwonderBus::Servo -- a degree-based object for one servo on the classic
//   bus, plus the `LX16A`/`LX225`/... aliases that make the port from
//   alexxcode/LX16A-bus a rename rather than a rewrite.
//
#ifndef HIWONDERBUS_SERVO_HPP
#define HIWONDERBUS_SERVO_HPP

#include <cstdint>
#include <utility>
#include <vector>

#include "HiwonderBus/exceptions.hpp"
#include "HiwonderBus/protocol.hpp"
#include "HiwonderBus/serial_bus.hpp"

namespace HiwonderBus {

/// Snapshot of the analog telemetry of one servo.
struct Telemetry {
  std::uint8_t id = kInvalidServoId;
  float angle_deg = 0.0f;
  int vin_mv = 0;
  int temp_c = 0;
  bool valid = false;
};

/// One classic-protocol servo on a `SerialBus`.
///
/// Angles are always degrees in `[config().min_angle_deg,
/// config().max_angle_deg]` -- i.e. 0.0 .. 240.0 for the classic parts -- and
/// tick conversion is entirely internal. Mirrors are updated on every accepted
/// command, so `getCommandedAngle()`, `isTorqueEnabled()` and friends are
/// cache hits that never touch the wire; pass `poll_hardware = true` to get a
/// physical read.
///
/// Not itself thread-safe: give each servo object to one thread, but the
/// shared `SerialBus` is safe to drive from several threads at once.
class Servo {
 public:
  /// `bus` must outlive the servo. `id` is 0..253; 254/255 are reserved.
  explicit Servo(SerialBus& bus, std::uint8_t id,
                 const ServoConfig& config = ServoConfig::classic());

  // --- Identity -----------------------------------------------------------

  std::uint8_t id() const noexcept { return id_; }
  const ServoConfig& config() const noexcept { return config_; }
  void setConfig(const ServoConfig& config) noexcept { config_ = config; }
  SerialBus& bus() const noexcept { return *bus_; }

  /// Sync our mirrors with the physical servo (temperature/limits/mode/etc).
  ///
  /// Mirrors `LX16A::initialize()` from alexxcode/LX16A-bus; with a bus that
  /// owns the port there is nothing to open here, so this just pulls state.
  void initialize(bool disable_torque = false);

  /// Explicitly refresh every cached field from the hardware.
  bool refresh(ErrorCode* error = nullptr) noexcept;

  /// Change the firmware id and keep this object's id in sync.
  void setID(std::uint8_t new_id);
  bool trySetID(std::uint8_t new_id, ErrorCode* error = nullptr) noexcept;

  /// Cached by default; `poll_hardware = true` asks the servo itself.
  std::uint8_t getID(bool poll_hardware = false) noexcept(false);
  bool tryGetID(std::uint8_t* out, bool poll_hardware = false,
                ErrorCode* error = nullptr) noexcept;

  // --- Motion -------------------------------------------------------------

  /// Command a move to `angle_deg`.
  ///
  /// `time_ms` is the travel duration (0..30000). `wait` queues cmd 7 until
  /// moveStart(). The C++ third argument is wait, matching LX16A-bus.
  /// Use moveRelative() for signed deltas; Python retains its upstream
  /// move(angle, time, relative, wait) signature.
  void move(float angle_deg, std::uint16_t time_ms = 0, bool wait = false);

  bool tryMove(float angle_deg, std::uint16_t time_ms = 0, bool wait = false,
               ErrorCode* error = nullptr) noexcept;
  bool tryMoveRelative(float delta_deg, std::uint16_t time_ms = 0, bool wait = false,
                       ErrorCode* error = nullptr) noexcept;

  /// Type-safe spelling of the same command, handy in C++ control loops:
  /// `servo.moveTo(90.0f, 500, MoveMode::kWait)`.
  enum class MoveMode : std::uint8_t {
    kInstant = 0,  ///< opcode 1 -- start now
    kWait = 1,     ///< opcode 7 -- hold until moveStart()
  };
  void moveTo(float angle_deg, std::uint16_t time_ms = 0,
              MoveMode mode = MoveMode::kInstant);

  /// `move(angle + commanded_angle, ...)` -- PyLX's `relative=True`.
  void moveRelative(float angle_deg, std::uint16_t time_ms = 0, bool wait = false);

  /// Release targets queued with `move(..., wait = true)`.
  void moveStart();
  bool tryMoveStart(ErrorCode* error = nullptr) noexcept;

  /// Abort the current movement.
  void moveStop();
  bool tryMoveStop(ErrorCode* error = nullptr) noexcept;

  bool isWaitingForMove() const noexcept { return waiting_for_move_; }
  std::uint16_t waitingTicks() const noexcept { return waiting_angle_ticks_; }

  /// Angle set by the last `move(..., wait = true)`.
  float getWaitingAngle() const;

  /// Physical shaft position. `poll_hardware = false` returns the last value
  /// observed (or, when never polled, the last commanded angle).
  float getPhysicalAngle(bool poll_hardware = true) noexcept(false);
  bool tryGetPhysicalAngle(float* out, bool poll_hardware = true,
                           ErrorCode* error = nullptr) noexcept;

  /// Last angle we commanded, from the mirror (no wire traffic).
  float getCommandedAngle() const noexcept {
    return HiwonderBus::ticksToAngle(commanded_angle_ticks_, config_);
  }

  /// `(angle, time_ms)` of the last instant / delayed move, straight from the
  /// servo (opcodes 2 and 8).
  std::pair<float, std::uint16_t> getLastInstantMove();
  std::pair<float, std::uint16_t> getLastDelayedMove();

  // --- Torque & operating mode -------------------------------------------

  void enableTorque();
  void disableTorque();
  bool trySetTorque(bool enable, ErrorCode* error = nullptr) noexcept;

  /// Cached by default; `poll_hardware = true` asks the servo.
  bool isTorqueEnabled(bool poll_hardware = false) noexcept(false);
  bool tryIsTorqueEnabled(bool* out, bool poll_hardware = false,
                          ErrorCode* error = nullptr) noexcept;

  /// Continuous-spin mode at `speed` (-1000..1000; sign selects direction).
  void setMotorMode(std::int16_t speed);
  bool trySetMotorMode(std::int16_t speed, ErrorCode* error = nullptr) noexcept;

  /// Back to position control.
  void setServoMode();
  bool trySetServoMode(ErrorCode* error = nullptr) noexcept;

  bool isMotorMode(bool poll_hardware = false) noexcept(false);
  bool tryIsMotorMode(bool* out, bool poll_hardware = false,
                      ErrorCode* error = nullptr) noexcept;

  /// Speed mirror. Throws ServoLogicalError when not in motor mode (PyLX
  /// parity); returns `false` from the try variant.
  int getMotorSpeed(bool poll_hardware = false) noexcept(false);
  bool tryGetMotorSpeed(int* out, bool poll_hardware = false,
                        ErrorCode* error = nullptr) noexcept;

  // --- Calibration & limits ----------------------------------------------

  /// `offset_deg` is clamped to +/-30deg by the firmware. `permanent` writes
  /// it to EEPROM.
  void setAngleOffset(float offset_deg, bool permanent = false);
  bool trySetAngleOffset(float offset_deg, bool permanent = false,
                         ErrorCode* error = nullptr) noexcept;

  float getAngleOffset(bool poll_hardware = false) noexcept(false);
  bool tryGetAngleOffset(float* out, bool poll_hardware = false,
                         ErrorCode* error = nullptr) noexcept;

  void setAngleLimits(float min_deg, float max_deg);
  bool trySetAngleLimits(float min_deg, float max_deg,
                         ErrorCode* error = nullptr) noexcept;

  AngleLimits getAngleLimits(bool poll_hardware = false) noexcept(false);
  bool tryGetAngleLimits(AngleLimits* out, bool poll_hardware = false,
                         ErrorCode* error = nullptr) noexcept;

  /// Both bounds in millivolts, inside 4500..12000.
  void setVinLimits(int min_mv, int max_mv);
  bool trySetVinLimits(int min_mv, int max_mv, ErrorCode* error = nullptr) noexcept;

  VinLimits getVinLimits(bool poll_hardware = false) noexcept(false);
  bool tryGetVinLimits(VinLimits* out, bool poll_hardware = false,
                       ErrorCode* error = nullptr) noexcept;

  /// Overheat protection ceiling, deg C (50..100).
  void setTempLimit(int max_temp_c);
  bool trySetTempLimit(int max_temp_c, ErrorCode* error = nullptr) noexcept;

  int getTempLimit(bool poll_hardware = false) noexcept(false);
  bool tryGetTempLimit(int* out, bool poll_hardware = false,
                       ErrorCode* error = nullptr) noexcept;

  // --- Telemetry & diagnostics -------------------------------------------

  int getTemp() noexcept(false);
  bool tryGetTemp(int* out, ErrorCode* error = nullptr) noexcept;

  int getVin() noexcept(false);
  bool tryGetVin(int* out, ErrorCode* error = nullptr) noexcept;

  /// One round trip per field, but assembled into a `Telemetry` snapshot.
  Telemetry readTelemetry();

  // --- Status LED --------------------------------------------------------

  void ledPowerOn();
  void ledPowerOff();
  bool trySetLedPower(bool on, ErrorCode* error = nullptr) noexcept;
  bool isLedPowerOn(bool poll_hardware = false) noexcept(false);
  bool tryIsLedPowerOn(bool* out, bool poll_hardware = false,
                       ErrorCode* error = nullptr) noexcept;

  void setLedErrorTriggers(const LedTriggers& triggers);
  void setLedErrorTriggers(bool over_temperature, bool over_voltage,
                           bool rotor_locked);
  bool trySetLedErrorTriggers(const LedTriggers& triggers,
                              ErrorCode* error = nullptr) noexcept;

  LedTriggers getLedErrorTriggers(bool poll_hardware = false) noexcept(false);
  bool tryGetLedErrorTriggers(LedTriggers* out, bool poll_hardware = false,
                              ErrorCode* error = nullptr) noexcept;

  // --- Low-level helpers -------------------------------------------------

  /// Send a frame addressed to this servo and parse the reply.
  /// Never throws; inspect the returned code. `timeout_ms < 0` uses the
  /// bus's configured read timeout.
  bool tryTransact(Command cmd, Response* out, std::size_t expected_payload = 0,
                   int timeout_ms = -1, ErrorCode* error = nullptr) noexcept;

  /// Fire-and-forget write addressed to this servo.
  bool command(Command cmd, const std::vector<std::uint8_t>& args = {},
               ErrorCode* error = nullptr) noexcept;

  /// Tick conversions honouring this servo's `ServoConfig`.
  std::uint16_t angleToTicks(float angle_deg) const noexcept {
    return HiwonderBus::angleToTicks(angle_deg, config_);
  }
  float ticksToAngle(std::uint16_t ticks) const noexcept {
    return HiwonderBus::ticksToAngle(ticks, config_);
  }

  /// True when the last wire operation on this servo succeeded.
  bool healthy() const noexcept { return last_error_ == ErrorCode::kOk; }
  ErrorCode lastError() const noexcept { return last_error_; }

  /// Human-readable description of `lastError()`, empty when healthy.
  /// Populated in both error modes; strict mode also throws it.
  const std::string& lastMessage() const noexcept { return last_message_; }

  // --- Bus-level batch primitives (static, one syscall each) -------------

  /// Broadcast synchronized move: one cmd-7 frame per servo plus a single
  /// broadcast cmd-11, all written in one write(2).
  static std::size_t moveSync(SerialBus& bus, const std::vector<Servo*>& servos,
                              const std::vector<float>& angles,
                              std::uint16_t time_ms, bool broadcast_start = true,
                              ErrorCode* error = nullptr) noexcept;

  /// Batch physical-angle sweep.
  static std::vector<float> readAngles(SerialBus& bus,
                                       const std::vector<Servo*>& servos,
                                       bool poll_hardware = true,
                                       ErrorCode* error = nullptr) noexcept;

  /// Batch temperature/voltage/angle sweep.
  static std::vector<Telemetry> readTelemetry(SerialBus& bus,
                                              const std::vector<Servo*>& servos,
                                              ErrorCode* error = nullptr) noexcept;

  /// Batch torque enable/disable in one write(2).
  static std::size_t setTorque(SerialBus& bus, const std::vector<Servo*>& servos,
                               bool enable, ErrorCode* error = nullptr) noexcept;

  /// Set angle limits on many servos at once (one write(2) per servo).
  static std::size_t setAngleLimits(SerialBus& bus,
                                    const std::vector<Servo*>& servos,
                                    float min_deg, float max_deg,
                                    ErrorCode* error = nullptr) noexcept;

 private:
  /// Read `cmd`, validate the reply and copy out the payload. Honours
  /// ErrorMode: throws only in strict mode.
  bool readReg(Command cmd, Response* out, std::size_t expected_payload,
               ErrorCode* error = nullptr) noexcept;

  bool writeReg(Command cmd, const std::vector<std::uint8_t>& args,
                ErrorCode* error = nullptr) noexcept;

  bool tryMoveImpl(float angle_deg, std::uint16_t time_ms, bool relative, bool wait,
                   ErrorCode* error) noexcept;
  bool validateMoveTarget(float angle_deg, std::uint16_t time_ms,
                          ErrorCode* error) noexcept;
  bool tryReadTelemetry(Telemetry* out, ErrorCode* error) noexcept;

  /// Record an error on this servo. Never throws: the `try*` API is
  /// exception-free in both modes; only the public wrappers re-raise via
  /// `checkAndThrow()`.
  bool reportFailure(ErrorCode code, const std::string& message,
                     ErrorCode* error = nullptr) noexcept;

  /// Throw the recorded error when the bus is in strict mode.
  void checkAndThrow() const;

  SerialBus* bus_;
  std::uint8_t id_;
  ServoConfig config_;
  std::string last_message_;

  // Mirrors -- kept in tick units to avoid float drift, like PyLX-16A.
  std::uint16_t commanded_angle_ticks_ = 0;
  std::uint16_t waiting_angle_ticks_ = 0;
  bool waiting_for_move_ = false;
  std::int16_t angle_offset_ticks_ = 0;
  std::uint16_t angle_limit_min_ticks_ = 0;
  std::uint16_t angle_limit_max_ticks_ = 1000;
  int vin_limit_min_mv_ = kVinLimitMinMv;
  int vin_limit_max_mv_ = kVinLimitMaxMv;
  int temp_limit_c_ = 90;
  bool motor_mode_ = false;
  int motor_speed_ = 0;
  bool torque_enabled_ = false;
  bool led_powered_ = false;
  LedTriggers led_triggers_;
  float last_physical_angle_deg_ = 0.0f;
  bool has_physical_angle_ = false;

  ErrorCode last_error_ = ErrorCode::kOk;
};

/// ---------------------------------------------------------------------------
/// Compatibility aliases
/// ---------------------------------------------------------------------------
///
/// Everything on the classic bus speaks the same wire protocol, so the
/// concrete part names are spelling, not types. Migrating users can keep
/// writing `HiwonderBus::LX16A`, `LX225`, ...

using LX16A = Servo;
using LX15D = Servo;
using LX224 = Servo;
using LX225 = Servo;
using HTS35H = Servo;
using HV206 = Servo;
using LewanSoulServo = Servo;

using LX16ABus = SerialBus;
using LewanSoulBus = SerialBus;
using Bus = SerialBus;

}  // namespace HiwonderBus

#endif  // HIWONDERBUS_SERVO_HPP
