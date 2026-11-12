// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// src/servo.cpp
//   The degree-based servo object. Every command in alexxcode/LX16A-bus and
//   ethanlipson/PyLX-16A is implemented here with the same validation and
//   mirroring semantics, on top of the POSIX transport.
//
// Error-handling convention
// -------------------------
//   * Every `tryX()` is exception-free in both error modes. Failures are
//     recorded in `last_error_` / `last_message_` (and mirrored into the
//     caller's `ErrorCode*` when one is supplied).
//   * Every public throwing wrapper simply calls its `tryX()` and then
//     `checkAndThrow()`, which re-raises only when the bus is in
//     `ErrorMode::kStrict`.
//
#include "HiwonderBus/servo.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace HiwonderBus {
namespace {

/// Payload sizes the firmware returns for each read opcode.
std::size_t payloadSize(Command cmd) noexcept {
  switch (cmd) {
    case Command::kReadLastInstantMove:
    case Command::kReadLastDelayedMove:
    case Command::kReadAngleLimits:
    case Command::kReadVinLimits:
    case Command::kReadMotorMode:
      return 4;
    case Command::kReadVin:
    case Command::kReadPhysicalAngle:
      return 2;
    case Command::kReadId:
    case Command::kReadAngleOffset:
    case Command::kReadTempLimit:
    case Command::kReadTemp:
    case Command::kReadTorque:
    case Command::kReadLed:
    case Command::kReadLedErrorTriggers:
      return 1;
    default:
      return 0;  // write / fire-and-forget
  }
}

bool isFinite(float v) noexcept { return std::isfinite(v); }

/// Formats "<name> must be between <lo> and <hi> (received <got>)".
///
/// Everything goes through double so that uint8_t ids and int16_t speeds
/// print as numbers rather than as characters.
std::string rangeText(const char* name, double lo, double hi, double got) {
  std::ostringstream oss;
  oss.precision(6);
  oss << name << " must be between " << lo << " and " << hi << " (received " << got
      << ")";
  return oss.str();
}

float nanAngle() noexcept { return std::numeric_limits<float>::quiet_NaN(); }

}  // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Servo::Servo(SerialBus& bus, std::uint8_t id, const ServoConfig& config)
    : bus_(&bus), id_(id), config_(config) {
  if (id > kMaxServoId) {
    throw ServoArgumentError("Servo ID must be between 0 and 253 inclusive", id);
  }
  // A freshly-built object mirrors the classic safe defaults; initialize()
  // (or the first poll_hardware=true read) overwrites them from the servo.
  commanded_angle_ticks_ = 0;
  waiting_angle_ticks_ = 0;
  angle_limit_min_ticks_ = 0;
  angle_limit_max_ticks_ = config_.max_ticks;
  vin_limit_min_mv_ = kVinLimitMinMv;
  vin_limit_max_mv_ = kVinLimitMaxMv;
  temp_limit_c_ = 90;
}

// ---------------------------------------------------------------------------
// Failure plumbing
// ---------------------------------------------------------------------------

bool Servo::reportFailure(ErrorCode code, const std::string& message,
                          ErrorCode* error) noexcept {
  last_error_ = code;
  last_message_ = message;
  if (error) *error = code;
  return false;
}

void Servo::checkAndThrow() const {
  if (last_error_ != ErrorCode::kOk &&
      (bus_->errorMode() == ErrorMode::kStrict ||
       last_error_ == ErrorCode::kInvalidArgument || last_error_ == ErrorCode::kInvalidState)) {
    detail::throwError(last_error_, last_message_, id_);
  }
}

// ---------------------------------------------------------------------------
// Wire helpers
// ---------------------------------------------------------------------------

bool Servo::tryTransact(Command cmd, Response* out, std::size_t expected_payload,
                        int timeout_ms, ErrorCode* error) noexcept {
  if (expected_payload == 0) expected_payload = payloadSize(cmd);
  Response response;
  ErrorCode code = ErrorCode::kOk;
  const Packet request = Packet::makeRead(id_, cmd);
  if (!bus_->tryTransact(request, &response, expected_payload, timeout_ms, &code) ||
      !response.ok()) {
    if (code == ErrorCode::kOk) code = ErrorCode::kBadChecksum;
    return reportFailure(
        code,
        std::string(toString(code)) + " while reading opcode " +
            std::to_string(static_cast<int>(cmd)),
        error);
  }
  last_error_ = ErrorCode::kOk;
  last_message_.clear();
  if (out) *out = std::move(response);
  if (error) *error = ErrorCode::kOk;
  return true;
}

bool Servo::command(Command cmd, const std::vector<std::uint8_t>& args,
                    ErrorCode* error) noexcept {
  const Packet request = Packet::make(id_, cmd, args);
  ErrorCode code = ErrorCode::kOk;
  if (!bus_->tryCommand(request, &code)) {
    return reportFailure(
        code,
        std::string(toString(code)) + " while sending opcode " +
            std::to_string(static_cast<int>(cmd)),
        error);
  }
  last_error_ = ErrorCode::kOk;
  last_message_.clear();
  if (error) *error = ErrorCode::kOk;
  return true;
}

bool Servo::readReg(Command cmd, Response* out, std::size_t expected_payload,
                    ErrorCode* error) noexcept {
  return tryTransact(cmd, out, expected_payload, -1, error);
}

bool Servo::writeReg(Command cmd, const std::vector<std::uint8_t>& args,
                     ErrorCode* error) noexcept {
  return command(cmd, args, error);
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

void Servo::initialize(bool disable_torque) {
  ErrorCode code = ErrorCode::kOk;

  // Seed the commanded-angle mirror from reality, exactly like PyLX-16A's
  // __init__ (`_commanded_angle = to_servo_range(get_physical_angle())`).
  // Best-effort: a servo that will not answer is reported by refresh().
  float angle = 0.0f;
  if (tryGetPhysicalAngle(&angle, true)) {
    commanded_angle_ticks_ = angleToTicks(angle);
  }

  if (!refresh(&code)) checkAndThrow();

  if (disable_torque) {
    if (!trySetTorque(false, &code)) checkAndThrow();
  } else {
    if (!trySetTorque(true, &code)) checkAndThrow();
  }
}

bool Servo::refresh(ErrorCode* error) noexcept {
  Response response;

  auto readInto = [&](Command cmd, std::size_t payload) -> bool {
    ErrorCode code = ErrorCode::kOk;
    if (!readReg(cmd, &response, payload, &code)) {
      reportFailure(code, last_message_, error);
      return false;
    }
    return true;
  };

  if (!readInto(Command::kReadId, 1)) return false;
  if (response[0] > kMaxServoId || response[0] != id_) {
    return reportFailure(ErrorCode::kBadFrame, "invalid identity in ID reply", error);
  }

  if (!readInto(Command::kReadAngleOffset, 1)) return false;
  angle_offset_ticks_ =
      static_cast<std::int16_t>(response[0] > 0x7F ? response[0] - 0x100 : response[0]);

  if (!readInto(Command::kReadAngleLimits, 4)) return false;
  angle_limit_min_ticks_ = response.u16(0);
  angle_limit_max_ticks_ = response.u16(2);

  if (!readInto(Command::kReadVinLimits, 4)) return false;
  vin_limit_min_mv_ = response.u16(0);
  vin_limit_max_mv_ = response.u16(2);

  if (!readInto(Command::kReadTempLimit, 1)) return false;
  temp_limit_c_ = response[0];

  if (!readInto(Command::kReadMotorMode, 4)) return false;
  motor_mode_ = response[0] == 1;
  motor_speed_ = motor_mode_ ? static_cast<int>(response.u16(2)) : 0;
  if (motor_mode_ && motor_speed_ > 32767) motor_speed_ -= 65536;

  if (!readInto(Command::kReadTorque, 1)) return false;
  torque_enabled_ = response[0] == 1;

  // The LED register is active-low: 0 means "powered".
  if (!readInto(Command::kReadLed, 1)) return false;
  led_powered_ = response[0] == 0;

  if (!readInto(Command::kReadLedErrorTriggers, 1)) return false;
  led_triggers_ = LedTriggers::fromMask(response[0]);

  last_error_ = ErrorCode::kOk;
  if (error) *error = ErrorCode::kOk;
  return true;
}

bool Servo::trySetID(std::uint8_t new_id, ErrorCode* error) noexcept {
  if (new_id > kMaxServoId) {
    return reportFailure(ErrorCode::kInvalidArgument,
                         rangeText("servo ID", kMinServoId, kMaxServoId, new_id), error);
  }
  const std::vector<std::uint8_t> args{new_id};
  if (!writeReg(Command::kWriteId, args, error)) return false;
  id_ = new_id;
  return true;
}

void Servo::setID(std::uint8_t new_id) {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetID(new_id, &code)) checkAndThrow();
}

std::uint8_t Servo::getID(bool poll_hardware) noexcept(false) {
  std::uint8_t value = id_;
  if (!tryGetID(&value, poll_hardware)) checkAndThrow();
  return value;
}

bool Servo::tryGetID(std::uint8_t* out, bool poll_hardware, ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) *out = id_;
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadId, &response, 1, error)) return false;
  if (response[0] > kMaxServoId || response[0] != id_) {
    return reportFailure(ErrorCode::kBadFrame, "invalid identity in ID reply", error);
  }
  if (out) *out = response[0];
  return true;
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

bool Servo::validateMoveTarget(float angle_deg, std::uint16_t time_ms,
                               ErrorCode* error) noexcept {
  if (!torque_enabled_) {
    return reportFailure(ErrorCode::kInvalidState,
                         "torque must be enabled to move", error);
  }
  if (motor_mode_) {
    return reportFailure(ErrorCode::kInvalidState,
                         "motor mode must be disabled to control movement", error);
  }
  if (!isFinite(angle_deg)) {
    return reportFailure(ErrorCode::kInvalidArgument,
                         "angle must be a finite number", error);
  }
  if (angle_deg < config_.min_angle_deg || angle_deg > config_.max_angle_deg) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        rangeText("angle", config_.min_angle_deg, config_.max_angle_deg, angle_deg),
        error);
  }
  if (time_ms > kMaxMoveTimeMs) {
    return reportFailure(ErrorCode::kInvalidArgument,
                         rangeText("time", 0, kMaxMoveTimeMs, time_ms), error);
  }

  const float limit_lo = ticksToAngle(angle_limit_min_ticks_);
  const float limit_hi = ticksToAngle(angle_limit_max_ticks_);
  if (angle_deg < limit_lo || angle_deg > limit_hi) {
    return reportFailure(ErrorCode::kInvalidArgument,
                         rangeText("angle", limit_lo, limit_hi, angle_deg), error);
  }

  if (error) *error = ErrorCode::kOk;
  return true;
}

bool Servo::tryMoveImpl(float angle_deg, std::uint16_t time_ms, bool relative, bool wait,
                        ErrorCode* error) noexcept {
  // Compute the final destination in degrees, not unsigned delta ticks. This
  // supports negative deltas and nonzero geometry origins without wrapping.
  const float target = relative ? getCommandedAngle() + angle_deg : angle_deg;
  if (!validateMoveTarget(target, time_ms, error)) return false;
  const std::uint16_t ticks = angleToTicks(target);

  std::uint8_t args[4];
  putU16(args, ticks);
  putU16(args + 2, time_ms);
  const std::vector<std::uint8_t> v(args, args + 4);

  if (!writeReg(wait ? Command::kMoveWait : Command::kMove, v, error)) return false;

  if (wait) {
    waiting_angle_ticks_ = ticks;
    waiting_for_move_ = true;
  } else {
    commanded_angle_ticks_ = ticks;
  }
  return true;
}

bool Servo::tryMove(float angle_deg, std::uint16_t time_ms, bool wait,
                    ErrorCode* error) noexcept {
  return tryMoveImpl(angle_deg, time_ms, false, wait, error);
}

bool Servo::tryMoveRelative(float delta_deg, std::uint16_t time_ms, bool wait,
                            ErrorCode* error) noexcept {
  return tryMoveImpl(delta_deg, time_ms, true, wait, error);
}

void Servo::move(float angle_deg, std::uint16_t time_ms, bool wait) {
  if (!tryMove(angle_deg, time_ms, wait)) checkAndThrow();
}

void Servo::moveTo(float angle_deg, std::uint16_t time_ms, MoveMode mode) {
  ErrorCode code = ErrorCode::kOk;
  const bool wait = (mode == MoveMode::kWait);
  if (!tryMove(angle_deg, time_ms, wait, &code)) checkAndThrow();
}

void Servo::moveRelative(float angle_deg, std::uint16_t time_ms, bool wait) {
  ErrorCode code = ErrorCode::kOk;
  if (!tryMoveRelative(angle_deg, time_ms, wait, &code)) checkAndThrow();
}

bool Servo::tryMoveStart(ErrorCode* error) noexcept {
  if (!waiting_for_move_) {
    return reportFailure(ErrorCode::kInvalidState, "not waiting for move", error);
  }
  if (!torque_enabled_) {
    return reportFailure(ErrorCode::kInvalidState, "torque must be enabled to move",
                         error);
  }
  if (motor_mode_) {
    return reportFailure(ErrorCode::kInvalidState,
                         "motor mode must be disabled to control movement", error);
  }
  if (!writeReg(Command::kMoveStart, {}, error)) return false;
  commanded_angle_ticks_ = waiting_angle_ticks_;
  waiting_for_move_ = false;
  return true;
}

void Servo::moveStart() {
  ErrorCode code = ErrorCode::kOk;
  if (!tryMoveStart(&code)) checkAndThrow();
}

bool Servo::tryMoveStop(ErrorCode* error) noexcept {
  if (motor_mode_) {
    return reportFailure(ErrorCode::kInvalidState,
                         "motor mode must be disabled to control movement", error);
  }
  if (!writeReg(Command::kMoveStop, {}, error)) return false;

  // Re-sync the commanded angle to reality, exactly like PyLX-16A's
  // move_stop(). The stop is already on the wire at this point, so a failed
  // read is reported as that read's error rather than pretending nothing was
  // sent.
  Response response;
  if (!readReg(Command::kReadPhysicalAngle, &response, 2, error)) return false;
  const std::int16_t raw = response.rawAngle();
  last_physical_angle_deg_ = response.physicalAngle(config_);
  has_physical_angle_ = true;
  commanded_angle_ticks_ = static_cast<std::uint16_t>(raw < 0 ? 0 : raw);
  if (error) *error = ErrorCode::kOk;
  return true;
}

void Servo::moveStop() {
  ErrorCode code = ErrorCode::kOk;
  if (!tryMoveStop(&code)) checkAndThrow();
}

float Servo::getWaitingAngle() const {
  if (!waiting_for_move_) {
    if (bus_->errorMode() == ErrorMode::kStrict) {
      detail::throwError(ErrorCode::kInvalidState, "not waiting for move", id_);
    }
    return nanAngle();
  }
  return ticksToAngle(waiting_angle_ticks_);
}

bool Servo::tryGetPhysicalAngle(float* out, bool poll_hardware,
                                ErrorCode* error) noexcept {
  if (!poll_hardware) {
    // Cache hit. The commanded angle is the best available estimate when we
    // have never seen a real one, which is what keeps a control loop alive
    // after a servo drops off the bus.
    if (out) {
      *out = has_physical_angle_
                 ? last_physical_angle_deg_
                 : ticksToAngle(commanded_angle_ticks_);
    }
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadPhysicalAngle, &response, 2, error)) return false;
  last_physical_angle_deg_ = response.physicalAngle(config_);
  has_physical_angle_ = true;
  if (out) *out = last_physical_angle_deg_;
  return true;
}

float Servo::getPhysicalAngle(bool poll_hardware) noexcept(false) {
  float angle = 0.0f;
  if (!tryGetPhysicalAngle(&angle, poll_hardware)) {
    checkAndThrow();
    return nanAngle();
  }
  return angle;
}

std::pair<float, std::uint16_t> Servo::getLastInstantMove() {
  Response response;
  if (!readReg(Command::kReadLastInstantMove, &response, 4)) {
    checkAndThrow();
    return {nanAngle(), 0};
  }
  return {response.physicalAngle(config_), response.u16(2)};
}

std::pair<float, std::uint16_t> Servo::getLastDelayedMove() {
  Response response;
  if (!readReg(Command::kReadLastDelayedMove, &response, 4)) {
    checkAndThrow();
    return {nanAngle(), 0};
  }
  return {response.physicalAngle(config_), response.u16(2)};
}

// ---------------------------------------------------------------------------
// Torque & operating mode
// ---------------------------------------------------------------------------

bool Servo::trySetTorque(bool enable, ErrorCode* error) noexcept {
  const std::vector<std::uint8_t> args{static_cast<std::uint8_t>(enable ? 1 : 0)};
  if (!writeReg(Command::kWriteTorque, args, error)) return false;
  torque_enabled_ = enable;
  return true;
}

void Servo::enableTorque() {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetTorque(true, &code)) checkAndThrow();
}

void Servo::disableTorque() {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetTorque(false, &code)) checkAndThrow();
}

bool Servo::tryIsTorqueEnabled(bool* out, bool poll_hardware,
                              ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) *out = torque_enabled_;
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadTorque, &response, 1, error)) return false;
  torque_enabled_ = response[0] == 1;
  if (out) *out = torque_enabled_;
  return true;
}

bool Servo::isTorqueEnabled(bool poll_hardware) noexcept(false) {
  bool value = torque_enabled_;
  if (!tryIsTorqueEnabled(&value, poll_hardware)) {
    checkAndThrow();
    return false;
  }
  return value;
}

bool Servo::trySetMotorMode(std::int16_t speed, ErrorCode* error) noexcept {
  if (!torque_enabled_) {
    return reportFailure(ErrorCode::kInvalidState,
                         "torque must be enabled to control movement", error);
  }
  if (speed < -kMaxMotorSpeed || speed > kMaxMotorSpeed) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        rangeText("motor speed", -kMaxMotorSpeed, kMaxMotorSpeed, speed), error);
  }
  const std::uint16_t encoded =
      speed < 0 ? static_cast<std::uint16_t>(speed + 65536)
                : static_cast<std::uint16_t>(speed);
  std::uint8_t raw[4];
  raw[0] = 1;  // mode = motor
  raw[1] = 0;
  putU16(raw + 2, encoded);
  const std::vector<std::uint8_t> args(raw, raw + 4);
  if (!writeReg(Command::kWriteMotorMode, args, error)) return false;
  motor_mode_ = true;
  motor_speed_ = speed;
  return true;
}

void Servo::setMotorMode(std::int16_t speed) {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetMotorMode(speed, &code)) checkAndThrow();
}

bool Servo::trySetServoMode(ErrorCode* error) noexcept {
  const std::vector<std::uint8_t> args{0, 0, 0, 0};
  if (!writeReg(Command::kWriteMotorMode, args, error)) return false;
  motor_mode_ = false;
  motor_speed_ = 0;
  return true;
}

void Servo::setServoMode() {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetServoMode(&code)) checkAndThrow();
}

bool Servo::tryIsMotorMode(bool* out, bool poll_hardware, ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) *out = motor_mode_;
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadMotorMode, &response, 4, error)) return false;
  motor_mode_ = response[0] == 1;
  motor_speed_ = motor_mode_ ? static_cast<int>(normalizeRawAngle(response.u16(2))) : 0;
  if (out) *out = motor_mode_;
  return true;
}

bool Servo::isMotorMode(bool poll_hardware) noexcept(false) {
  bool value = motor_mode_;
  if (!tryIsMotorMode(&value, poll_hardware)) {
    checkAndThrow();
    return false;
  }
  return value;
}

bool Servo::tryGetMotorSpeed(int* out, bool poll_hardware, ErrorCode* error) noexcept {
  if (poll_hardware) {
    bool motor = false;
    if (!tryIsMotorMode(&motor, true, error)) return false;
  }
  if (!motor_mode_) {
    return reportFailure(ErrorCode::kInvalidState, "not in motor mode", error);
  }
  if (out) *out = motor_speed_;
  if (error) *error = ErrorCode::kOk;
  return true;
}

int Servo::getMotorSpeed(bool poll_hardware) noexcept(false) {
  int value = motor_speed_;
  if (!tryGetMotorSpeed(&value, poll_hardware)) {
    checkAndThrow();
    return 0;
  }
  return value;
}

// ---------------------------------------------------------------------------
// Calibration & limits
// ---------------------------------------------------------------------------

bool Servo::trySetAngleOffset(float offset_deg, bool permanent,
                              ErrorCode* error) noexcept {
  if (!isFinite(offset_deg)) {
    return reportFailure(ErrorCode::kInvalidArgument,
                         "angle offset must be a finite number", error);
  }
  if (offset_deg < kAngleOffsetMinDeg || offset_deg > kAngleOffsetMaxDeg) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        rangeText("angle offset", kAngleOffsetMinDeg, kAngleOffsetMaxDeg, offset_deg),
        error);
  }
  const std::uint8_t reg = angleOffsetToRegister(offset_deg, config_);
  const std::vector<std::uint8_t> args{reg};
  if (!writeReg(Command::kWriteAngleOffset, args, error)) return false;

  const float per_tick = config_.degPerTick() > 0.0f ? config_.degPerTick() : 1.0f;
  angle_offset_ticks_ = static_cast<std::int16_t>(offset_deg / per_tick);

  if (permanent && !writeReg(Command::kSaveAngleOffset, {}, error)) return false;
  return true;
}

void Servo::setAngleOffset(float offset_deg, bool permanent) {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetAngleOffset(offset_deg, permanent, &code)) checkAndThrow();
}

bool Servo::tryGetAngleOffset(float* out, bool poll_hardware,
                              ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) *out = static_cast<float>(angle_offset_ticks_) * config_.degPerTick();
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadAngleOffset, &response, 1, error)) return false;
  const float degrees = registerToAngleOffset(response[0], config_);
  angle_offset_ticks_ =
      static_cast<std::int16_t>(response[0] > 0x7F ? response[0] - 0x100 : response[0]);
  if (out) *out = degrees;
  return true;
}

float Servo::getAngleOffset(bool poll_hardware) noexcept(false) {
  float value = 0.0f;
  if (!tryGetAngleOffset(&value, poll_hardware)) {
    checkAndThrow();
    return nanAngle();
  }
  return value;
}

bool Servo::trySetAngleLimits(float min_deg, float max_deg,
                              ErrorCode* error) noexcept {
  if (!isFinite(min_deg) || !isFinite(max_deg)) {
    return reportFailure(ErrorCode::kInvalidArgument,
                         "angle limits must be finite numbers", error);
  }
  if (min_deg < config_.min_angle_deg || min_deg > config_.max_angle_deg) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        rangeText("lower limit", config_.min_angle_deg, config_.max_angle_deg, min_deg),
        error);
  }
  if (max_deg < config_.min_angle_deg || max_deg > config_.max_angle_deg) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        rangeText("upper limit", config_.min_angle_deg, config_.max_angle_deg, max_deg),
        error);
  }
  if (max_deg < min_deg) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        "lower limit (received " + std::to_string(min_deg) +
            ") must be less than upper limit (received " + std::to_string(max_deg) + ")",
        error);
  }
  const std::uint16_t lo = angleToTicks(min_deg);
  const std::uint16_t hi = angleToTicks(max_deg);
  std::uint8_t raw[4];
  putU16(raw, lo);
  putU16(raw + 2, hi);
  const std::vector<std::uint8_t> args(raw, raw + 4);
  if (!writeReg(Command::kWriteAngleLimits, args, error)) return false;
  angle_limit_min_ticks_ = lo;
  angle_limit_max_ticks_ = hi;
  return true;
}

void Servo::setAngleLimits(float min_deg, float max_deg) {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetAngleLimits(min_deg, max_deg, &code)) checkAndThrow();
}

bool Servo::tryGetAngleLimits(AngleLimits* out, bool poll_hardware,
                              ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) {
      *out = AngleLimits{ticksToAngle(angle_limit_min_ticks_),
                        ticksToAngle(angle_limit_max_ticks_)};
    }
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadAngleLimits, &response, 4, error)) return false;
  angle_limit_min_ticks_ = response.u16(0);
  angle_limit_max_ticks_ = response.u16(2);
  if (out) {
    *out = AngleLimits{ticksToAngle(angle_limit_min_ticks_),
                      ticksToAngle(angle_limit_max_ticks_)};
  }
  return true;
}

AngleLimits Servo::getAngleLimits(bool poll_hardware) noexcept(false) {
  AngleLimits limits{ticksToAngle(angle_limit_min_ticks_),
                     ticksToAngle(angle_limit_max_ticks_)};
  if (!tryGetAngleLimits(&limits, poll_hardware)) {
    checkAndThrow();
    return AngleLimits{nanAngle(), nanAngle()};
  }
  return limits;
}

bool Servo::trySetVinLimits(int min_mv, int max_mv, ErrorCode* error) noexcept {
  if (min_mv < kVinLimitMinMv || min_mv > kVinLimitMaxMv) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        rangeText("lower limit", kVinLimitMinMv, kVinLimitMaxMv, min_mv), error);
  }
  if (max_mv < kVinLimitMinMv || max_mv > kVinLimitMaxMv) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        rangeText("upper limit", kVinLimitMinMv, kVinLimitMaxMv, max_mv), error);
  }
  if (max_mv < min_mv) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        "lower limit (received " + std::to_string(min_mv) +
            ") must be less than upper limit (received " + std::to_string(max_mv) + ")",
        error);
  }
  std::uint8_t raw[4];
  putU16(raw, static_cast<std::uint16_t>(min_mv));
  putU16(raw + 2, static_cast<std::uint16_t>(max_mv));
  const std::vector<std::uint8_t> args(raw, raw + 4);
  if (!writeReg(Command::kWriteVinLimits, args, error)) return false;
  vin_limit_min_mv_ = min_mv;
  vin_limit_max_mv_ = max_mv;
  return true;
}

void Servo::setVinLimits(int min_mv, int max_mv) {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetVinLimits(min_mv, max_mv, &code)) checkAndThrow();
}

bool Servo::tryGetVinLimits(VinLimits* out, bool poll_hardware,
                            ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) *out = VinLimits{vin_limit_min_mv_, vin_limit_max_mv_};
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadVinLimits, &response, 4, error)) return false;
  vin_limit_min_mv_ = response.u16(0);
  vin_limit_max_mv_ = response.u16(2);
  if (out) *out = VinLimits{vin_limit_min_mv_, vin_limit_max_mv_};
  return true;
}

VinLimits Servo::getVinLimits(bool poll_hardware) noexcept(false) {
  VinLimits limits{vin_limit_min_mv_, vin_limit_max_mv_};
  if (!tryGetVinLimits(&limits, poll_hardware)) {
    checkAndThrow();
    return VinLimits{0, 0};
  }
  return limits;
}

bool Servo::trySetTempLimit(int max_temp_c, ErrorCode* error) noexcept {
  if (max_temp_c < kTempLimitMinC || max_temp_c > kTempLimitMaxC) {
    return reportFailure(
        ErrorCode::kInvalidArgument,
        rangeText("temperature limit", kTempLimitMinC, kTempLimitMaxC, max_temp_c),
        error);
  }
  const std::vector<std::uint8_t> args{static_cast<std::uint8_t>(max_temp_c)};
  if (!writeReg(Command::kWriteTempLimit, args, error)) return false;
  temp_limit_c_ = max_temp_c;
  return true;
}

void Servo::setTempLimit(int max_temp_c) {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetTempLimit(max_temp_c, &code)) checkAndThrow();
}

bool Servo::tryGetTempLimit(int* out, bool poll_hardware, ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) *out = temp_limit_c_;
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadTempLimit, &response, 1, error)) return false;
  temp_limit_c_ = response[0];
  if (out) *out = temp_limit_c_;
  return true;
}

int Servo::getTempLimit(bool poll_hardware) noexcept(false) {
  int value = temp_limit_c_;
  if (!tryGetTempLimit(&value, poll_hardware)) {
    checkAndThrow();
    return 0;
  }
  return value;
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

bool Servo::tryGetTemp(int* out, ErrorCode* error) noexcept {
  Response response;
  if (!readReg(Command::kReadTemp, &response, 1, error)) return false;
  if (out) *out = response[0];
  return true;
}

int Servo::getTemp() noexcept(false) {
  int value = 0;
  if (!tryGetTemp(&value)) {
    checkAndThrow();
    return -1;
  }
  return value;
}

bool Servo::tryGetVin(int* out, ErrorCode* error) noexcept {
  Response response;
  if (!readReg(Command::kReadVin, &response, 2, error)) return false;
  if (out) *out = response.voltage();
  return true;
}

int Servo::getVin() noexcept(false) {
  int value = 0;
  if (!tryGetVin(&value)) {
    checkAndThrow();
    return -1;
  }
  return value;
}

bool Servo::tryReadTelemetry(Telemetry* out, ErrorCode* error) noexcept {
  Telemetry snapshot;
  snapshot.id = id_;
  snapshot.angle_deg = nanAngle();
  snapshot.temp_c = -1;
  ErrorCode first = ErrorCode::kOk;
  std::string message;
  auto capture = [&](bool ok) {
    if (!ok && first == ErrorCode::kOk) {
      first = last_error_;
      message = last_message_;
    }
  };
  capture(tryGetPhysicalAngle(&snapshot.angle_deg, true));
  capture(tryGetVin(&snapshot.vin_mv));
  capture(tryGetTemp(&snapshot.temp_c));
  snapshot.valid = first == ErrorCode::kOk;
  if (out) *out = snapshot;
  if (!snapshot.valid) return reportFailure(first, message, error);
  if (error) *error = ErrorCode::kOk;
  return true;
}

Telemetry Servo::readTelemetry() {
  Telemetry snapshot;
  if (!tryReadTelemetry(&snapshot, nullptr)) checkAndThrow();
  return snapshot;
}

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------

bool Servo::trySetLedPower(bool on, ErrorCode* error) noexcept {
  // Active-low register: 0 powers the LED, 1 extinguishes it.
  const std::vector<std::uint8_t> args{static_cast<std::uint8_t>(on ? 0 : 1)};
  if (!writeReg(Command::kWriteLed, args, error)) return false;
  led_powered_ = on;
  return true;
}

void Servo::ledPowerOn() {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetLedPower(true, &code)) checkAndThrow();
}

void Servo::ledPowerOff() {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetLedPower(false, &code)) checkAndThrow();
}

bool Servo::tryIsLedPowerOn(bool* out, bool poll_hardware, ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) *out = led_powered_;
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadLed, &response, 1, error)) return false;
  led_powered_ = response[0] == 0;
  if (out) *out = led_powered_;  // active-low register
  return true;
}

bool Servo::isLedPowerOn(bool poll_hardware) noexcept(false) {
  bool value = led_powered_;
  if (!tryIsLedPowerOn(&value, poll_hardware)) {
    checkAndThrow();
    return false;
  }
  return value;
}

bool Servo::trySetLedErrorTriggers(const LedTriggers& triggers,
                                   ErrorCode* error) noexcept {
  const std::vector<std::uint8_t> args{triggers.toMask()};
  if (!writeReg(Command::kWriteLedErrorTriggers, args, error)) return false;
  led_triggers_ = triggers;
  return true;
}

void Servo::setLedErrorTriggers(const LedTriggers& triggers) {
  ErrorCode code = ErrorCode::kOk;
  if (!trySetLedErrorTriggers(triggers, &code)) checkAndThrow();
}

void Servo::setLedErrorTriggers(bool over_temperature, bool over_voltage,
                                bool rotor_locked) {
  setLedErrorTriggers(LedTriggers{over_temperature, over_voltage, rotor_locked});
}

bool Servo::tryGetLedErrorTriggers(LedTriggers* out, bool poll_hardware,
                                   ErrorCode* error) noexcept {
  if (!poll_hardware) {
    if (out) *out = led_triggers_;
    if (error) *error = ErrorCode::kOk;
    return true;
  }
  Response response;
  if (!readReg(Command::kReadLedErrorTriggers, &response, 1, error)) return false;
  led_triggers_ = LedTriggers::fromMask(response[0]);
  if (out) *out = led_triggers_;
  return true;
}

LedTriggers Servo::getLedErrorTriggers(bool poll_hardware) noexcept(false) {
  LedTriggers triggers = led_triggers_;
  if (!tryGetLedErrorTriggers(&triggers, poll_hardware)) {
    checkAndThrow();
    return LedTriggers{};
  }
  return triggers;
}

// ---------------------------------------------------------------------------
// Batch primitives
// ---------------------------------------------------------------------------

namespace {
bool sameBus(const Servo* servo, const SerialBus* bus) noexcept {
  return servo != nullptr && &servo->bus() == bus;
}
}  // namespace

std::size_t Servo::moveSync(SerialBus& bus, const std::vector<Servo*>& servos,
                            const std::vector<float>& angles, std::uint16_t time_ms,
                            bool broadcast_start, ErrorCode* error) noexcept {
  if (servos.empty() || servos.size() != angles.size()) {
    if (error) *error = ErrorCode::kInvalidArgument;
    return 0;
  }
  if (time_ms > kMaxMoveTimeMs) {
    if (error) *error = ErrorCode::kInvalidArgument;
    return 0;
  }

  std::vector<std::uint8_t> ids;
  std::vector<std::uint16_t> ticks;
  ids.reserve(servos.size());
  ticks.reserve(servos.size());

  for (std::size_t i = 0; i < servos.size(); ++i) {
    Servo* servo = servos[i];
    if (!sameBus(servo, &bus)) {
      if (error) *error = ErrorCode::kInvalidArgument;
      return 0;
    }
    const float angle = angles[i];
    if (!servo->validateMoveTarget(angle, time_ms, error)) return 0;
    ids.push_back(servo->id());
    ticks.push_back(servo->angleToTicks(angle));
  }

  const std::size_t written = bus.queueMoves(ids, ticks, time_ms, broadcast_start, error);
  if (written == 0) return 0;

  // The targets are queued with cmd 7; a broadcast start executes them now.
  for (std::size_t i = 0; i < servos.size(); ++i) {
    Servo* servo = servos[i];
    servo->waiting_angle_ticks_ = ticks[i];
    servo->waiting_for_move_ = !broadcast_start;
    if (broadcast_start) servo->commanded_angle_ticks_ = ticks[i];
    servo->last_error_ = ErrorCode::kOk;
  }
  return written;
}

std::vector<float> Servo::readAngles(SerialBus& bus, const std::vector<Servo*>& servos,
                                    bool poll_hardware, ErrorCode* error) noexcept {
  std::vector<float> angles(servos.size(), nanAngle());
  ErrorCode first = ErrorCode::kOk;
  bool seen_failure = false;

  for (std::size_t i = 0; i < servos.size(); ++i) {
    Servo* servo = servos[i];
    if (!sameBus(servo, &bus)) {
      if (!seen_failure) first = ErrorCode::kInvalidArgument;
      seen_failure = true;
      continue;
    }
    float angle = 0.0f;
    if (servo->tryGetPhysicalAngle(&angle, poll_hardware)) {
      angles[i] = angle;
    } else {
      if (!seen_failure) first = servo->lastError();
      seen_failure = true;
    }
  }
  if (error) *error = seen_failure ? first : ErrorCode::kOk;
  return angles;
}

std::vector<Telemetry> Servo::readTelemetry(SerialBus& bus,
                                            const std::vector<Servo*>& servos,
                                            ErrorCode* error) noexcept {
  std::vector<Telemetry> out(servos.size());
  ErrorCode first = ErrorCode::kOk;
  bool seen_failure = false;

  for (std::size_t i = 0; i < servos.size(); ++i) {
    Servo* servo = servos[i];
    Telemetry& snapshot = out[i];
    snapshot.id = servo ? servo->id() : kInvalidServoId;
    if (!sameBus(servo, &bus)) {
      if (!seen_failure) first = ErrorCode::kInvalidArgument;
      seen_failure = true;
      continue;
    }
    ErrorCode code = ErrorCode::kOk;
    if (!servo->tryReadTelemetry(&snapshot, &code)) {
      if (!seen_failure) first = code;
      seen_failure = true;
    }
  }
  if (error) *error = seen_failure ? first : ErrorCode::kOk;
  return out;
}

std::size_t Servo::setTorque(SerialBus& bus, const std::vector<Servo*>& servos,
                             bool enable, ErrorCode* error) noexcept {
  std::vector<std::uint8_t> ids;
  ids.reserve(servos.size());
  for (const Servo* servo : servos) {
    if (!sameBus(servo, &bus)) {
      if (error) *error = ErrorCode::kInvalidArgument;
      return 0;
    }
    ids.push_back(servo->id());
  }
  const std::size_t written = bus.setTorqueBatch(ids, enable, error);
  if (written != 0) {
    for (Servo* servo : servos) {
      servo->torque_enabled_ = enable;
      servo->last_error_ = ErrorCode::kOk;
    }
  }
  return written;
}

std::size_t Servo::setAngleLimits(SerialBus& bus, const std::vector<Servo*>& servos,
                                  float min_deg, float max_deg,
                                  ErrorCode* error) noexcept {
  if (servos.empty()) {
    if (error) *error = ErrorCode::kInvalidArgument;
    return 0;
  }
  std::vector<Packet> frames;
  frames.reserve(servos.size());
  std::vector<std::pair<std::uint16_t, std::uint16_t>> mirror;
  mirror.reserve(servos.size());

  for (const Servo* servo : servos) {
    if (!sameBus(servo, &bus)) {
      if (error) *error = ErrorCode::kInvalidArgument;
      return 0;
    }
    const ServoConfig cfg = servo->config();
    if (!isFinite(min_deg) || !isFinite(max_deg) || min_deg < cfg.min_angle_deg ||
        max_deg > cfg.max_angle_deg || max_deg < min_deg) {
      if (error) *error = ErrorCode::kInvalidArgument;
      return 0;
    }
    const std::uint16_t lo = servo->angleToTicks(min_deg);
    const std::uint16_t hi = servo->angleToTicks(max_deg);
    std::uint8_t raw[4];
    putU16(raw, lo);
    putU16(raw + 2, hi);
    frames.push_back(Packet::make(servo->id(), Command::kWriteAngleLimits, raw, 4));
    mirror.emplace_back(lo, hi);
  }

  const std::size_t written = bus.tryWriteFrames(frames, error);
  if (written != 0) {
    for (std::size_t i = 0; i < servos.size(); ++i) {
      Servo* servo = servos[i];
      servo->angle_limit_min_ticks_ = mirror[i].first;
      servo->angle_limit_max_ticks_ = mirror[i].second;
      servo->last_error_ = ErrorCode::kOk;
    }
  }
  return written;
}

}  // namespace HiwonderBus
