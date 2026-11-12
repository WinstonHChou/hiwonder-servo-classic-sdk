// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// protocol.hpp
//   Wire format, opcodes, packet framing, checksum and the configurable
//   angle <-> tick geometry shared by every "classic" Hiwonder / LewanSoul
//   servo: LX-16A, LX-15D, LX-224, LX-225, HTS-35H, HV-206, ...
//
// The protocol is a half-duplex, 8N1, 115200-baud UART bus. Every frame is
// bracketed by a two-byte header and terminated by an inverted one's-complement
// checksum that covers everything *after* the header:
//
//   request :  0x55 0x55 | id | length | cmd  | args...            | checksum
//   response:  0x55 0x55 | id | length | cmd  | data...            | checksum
//                     \__/   \_____________________________/ \___/  \_____/
//                   header    inner payload (covered)         checksum
//
// Replies echo the request command; there is no firmware status byte.
// `length` counts the inner payload bytes (id + length + cmd + args),
// i.e. it excludes the 0x55 0x55 header and the checksum itself, so a frame
// is always exactly `length + 3` bytes long.
//
#ifndef HIWONDERBUS_PROTOCOL_HPP
#define HIWONDERBUS_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace HiwonderBus {

// ---------------------------------------------------------------------------
// Wire constants
// ---------------------------------------------------------------------------

/// Second byte of the two-byte frame header (both bytes are 0x55).
constexpr uint8_t kFrameHeader0 = 0x55;
constexpr uint8_t kFrameHeader1 = 0x55;
constexpr std::size_t kFrameHeaderSize = 2;

/// Bytes of a frame that are *not* part of the inner `length` field
/// (the 0x55 0x55 header + the trailing checksum).
constexpr std::size_t kFrameOverhead = 3;

/// Offsets inside a frame: [55][55][id][len][cmd][data...][cksum].
constexpr std::size_t kFrameIdOffset = 2;
constexpr std::size_t kFrameLengthOffset = 3;
constexpr std::size_t kFrameOpcodeOffset = 4;
constexpr std::size_t kFrameDataOffset = 5;

/// Broadcast address accepted by every servo on the bus. Servos never answer
/// a frame addressed to the broadcast id, so it must only be used for writes.
constexpr uint8_t kBroadcastId = 0xFE;
/// Unicast ids are 0..253; 254 is broadcast and 255 is reserved.
constexpr uint8_t kMinServoId = 0;
constexpr uint8_t kMaxServoId = 253;
constexpr uint8_t kInvalidServoId = 0xFF;

/// Nominal baud rate of the classic bus.
constexpr uint32_t kDefaultBaudrate = 115200;

/// Largest move time accepted by the classic controllers, in milliseconds.
constexpr uint16_t kMaxMoveTimeMs = 30000;

/// Largest value the 16-bit motor-speed field can encode.
constexpr int16_t kMaxMotorSpeed = 1000;

/// Field ranges enforced by the controller firmware.
constexpr int kAngleOffsetMinDeg = -30;
constexpr int kAngleOffsetMaxDeg = 30;
constexpr int kVinLimitMinMv = 4500;
constexpr int kVinLimitMaxMv = 12000;
constexpr int kTempLimitMinC = 50;
constexpr int kTempLimitMaxC = 100;

// ---------------------------------------------------------------------------
// Opcodes
// ---------------------------------------------------------------------------

/// Command bytes of the classic 0x55 0x55 protocol.
///
/// The numbering is the one used by the Hiwonder/LewanSoul controllers and by
/// both reference implementations (alecxcode/LX16A-bus and
/// ethanlipson/PyLX-16A).
enum class Command : uint8_t {
  // --- Motion -------------------------------------------------------------
  kMove = 1,                ///< Instant move: angle + time, executes now.
  kReadLastInstantMove = 2,  ///< Angle/time of the last kMove.
  kMoveWait = 7,             ///< Move: angle + time, held until kMoveStart.
  kReadLastDelayedMove = 8,  ///< Angle/time of the last kMoveWait.
  kMoveStart = 11,           ///< Release queued kMoveWait targets.
  kMoveStop = 12,            ///< Halt the current movement.

  // --- Identity -----------------------------------------------------------
  kWriteId = 13,  ///< Assign a new servo id.
  kReadId = 14,   ///< Read the firmware id.

  // --- Calibration --------------------------------------------------------
  kWriteAngleOffset = 17,   ///< Temporarily shift the commanded zero.
  kSaveAngleOffset = 18,    ///< Commit the offset to EEPROM.
  kReadAngleOffset = 19,    ///< Read the offset.
  kWriteAngleLimits = 20,   ///< Range of motion lower/upper bounds.
  kReadAngleLimits = 21,    ///< Read the range-of-motion bounds.
  kWriteVinLimits = 22,     ///< Protection voltage window (mV).
  kReadVinLimits = 23,      ///< Read the voltage window.
  kWriteTempLimit = 24,     ///< Protection temperature ceiling (deg C).
  kReadTempLimit = 25,      ///< Read the temperature ceiling.

  // --- Telemetry ----------------------------------------------------------
  kReadTemp = 26,        ///< Die temperature, deg C.
  kReadVin = 27,         ///< Supply voltage, mV.
  kReadPhysicalAngle = 28,  ///< Shaft position from the potentiometer.

  // --- Operating mode -----------------------------------------------------
  kWriteMotorMode = 29,  ///< Select motor (continuous) or servo mode.
  kReadMotorMode = 30,   ///< Read mode + speed.
  kWriteTorque = 31,     ///< Enable/disable output torque.
  kReadTorque = 32,      ///< Read torque state.

  // --- Status LED ---------------------------------------------------------
  kWriteLed = 33,              ///< Power the status LED on/off.
  kReadLed = 34,               ///< Read LED state.
  kWriteLedErrorTriggers = 35,  ///< Choose which faults flash the LED.
  kReadLedErrorTriggers = 36,   ///< Read the fault-trigger mask.
};

/// Which LED error conditions latch. Mirrors the 3-bit mask at opcode 35.
struct LedTriggers {
  bool over_temperature = false;
  bool over_voltage = false;
  bool rotor_locked = false;

  constexpr bool operator==(const LedTriggers& o) const noexcept {
    return over_temperature == o.over_temperature &&
           over_voltage == o.over_voltage && rotor_locked == o.rotor_locked;
  }

  /// Pack into the on-wire bit mask: bit0 temp, bit1 voltage, bit2 lock.
  constexpr uint8_t toMask() const noexcept {
    return static_cast<uint8_t>((rotor_locked ? 4u : 0u) |
                                (over_voltage ? 2u : 0u) |
                                (over_temperature ? 1u : 0u));
  }

  static constexpr LedTriggers fromMask(uint8_t mask) noexcept {
    LedTriggers t;
    t.over_temperature = (mask & 0x01u) != 0;
    t.over_voltage = (mask & 0x02u) != 0;
    t.rotor_locked = (mask & 0x04u) != 0;
    return t;
  }
};

/// Lower/upper bound pair, already expressed in engineering units.
template <typename T>
struct Limits {
  T min{};
  T max{};

  constexpr bool operator==(const Limits& o) const noexcept {
    return min == o.min && max == o.max;
  }
  constexpr bool contains(T value) const noexcept {
    return value >= min && value <= max;
  }
};

using AngleLimits = Limits<float>;
using VinLimits = Limits<int>;

// ---------------------------------------------------------------------------
// Angle <-> tick geometry
// ---------------------------------------------------------------------------

/// Travel geometry of a servo family.
///
/// The classic controllers all use 1000 ticks for 0.0deg..240.0deg, i.e.
/// 0.24 deg/tick, but a handful of classic-protocol variants advertise a
/// different sweep. Everything in this SDK goes through a `ServoConfig`, so a
/// non-default part is a one-line change rather than a fork.
struct ServoConfig {
  /// Number of ticks spanning the full commanded travel.
  uint16_t max_ticks = 1000;
  /// Commanded angle range, degrees.
  float min_angle_deg = 0.0f;
  float max_angle_deg = 240.0f;

  constexpr ServoConfig() noexcept = default;
  constexpr ServoConfig(uint16_t ticks, float min_deg, float max_deg) noexcept
      : max_ticks(ticks), min_angle_deg(min_deg), max_angle_deg(max_deg) {}

  /// Default classic geometry: 1000 ticks / 0..240 deg / 0.24 deg per tick.
  static constexpr ServoConfig classic() noexcept {
    return ServoConfig(1000, 0.0f, 240.0f);
  }

  /// Degrees represented by a single tick (0.24f for the classic parts).
  constexpr float degPerTick() const noexcept {
    return max_ticks == 0 ? 0.0f : (max_angle_deg - min_angle_deg) /
                                       static_cast<float>(max_ticks);
  }

  /// Full-scale span in degrees (240.0f for the classic parts).
  constexpr float spanDeg() const noexcept {
    return max_angle_deg - min_angle_deg;
  }
};

/// Bytes of an unsigned 16-bit little-endian field (the bus is LSB-first).
inline void putU16(uint8_t* dst, uint16_t value) noexcept {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

/// Reads a little-endian u16 out of a payload (already header-stripped).
inline uint16_t getU16(const uint8_t* src) noexcept {
  return static_cast<uint16_t>(static_cast<uint16_t>(src[0]) |
                               (static_cast<uint16_t>(src[1]) << 8));
}

/// Interpret a raw 16-bit angle register as a signed tick count.
///
/// The controllers report negative positions as two's complement but expose a
/// 0..32767 window; anything above 32767 wraps to the negative side.
inline int16_t normalizeRawAngle(uint16_t raw) noexcept {
  return static_cast<int16_t>(raw > 0x7FFF ? raw - 0x10000 : raw);
}

/// degrees -> ticks, rounded to the nearest whole tick.
///
/// Values outside the configured travel are clamped to `[0, max_ticks]` so a
/// stray float can never command a servo beyond its own range. Callers that
/// need rejection should use `Servo::move()`, which validates first.
inline uint16_t angleToTicks(float angle_deg,
                             const ServoConfig& cfg = ServoConfig::classic()) noexcept {
  const float span = cfg.spanDeg();
  if (!(span > 0.0f) || cfg.max_ticks == 0) return 0;
  float ticks = (angle_deg - cfg.min_angle_deg) / span *
                static_cast<float>(cfg.max_ticks);
  if (!(ticks > 0.0f)) return 0;             // also catches NaN
  if (ticks > static_cast<float>(cfg.max_ticks)) return cfg.max_ticks;
  return static_cast<uint16_t>(ticks + 0.5f);
}

/// ticks -> degrees.
inline float ticksToAngle(uint16_t ticks,
                          const ServoConfig& cfg = ServoConfig::classic()) noexcept {
  return cfg.min_angle_deg +
         static_cast<float>(ticks) * cfg.degPerTick();
}

/// Signed ticks -> degrees, so a shaft past the zero mark reads negative
/// (this is what ethanlipson/PyLX-16A returns for a raw register > 0x7FFF).
inline float signedTicksToAngle(int16_t ticks,
                                const ServoConfig& cfg = ServoConfig::classic()) noexcept {
  return cfg.min_angle_deg +
         static_cast<float>(ticks) * cfg.degPerTick();
}

/// Raw signed angle register -> degrees.
inline float rawAngleToDegrees(
    uint16_t raw, const ServoConfig& cfg = ServoConfig::classic()) noexcept {
  return signedTicksToAngle(normalizeRawAngle(raw), cfg);
}

/// Encodes a signed angle offset (degrees) into the 8-bit two's complement
/// register used by opcode 17.
inline uint8_t angleOffsetToRegister(
    float offset_deg, const ServoConfig& cfg = ServoConfig::classic()) noexcept {
  const float per_tick = cfg.degPerTick() > 0.0f ? cfg.degPerTick() : 1.0f;
  int ticks = static_cast<int>(offset_deg / per_tick + (offset_deg >= 0 ? 0.5f : -0.5f));
  return static_cast<uint8_t>(static_cast<uint16_t>(
      static_cast<int16_t>(ticks)) & 0xFF);
}

/// Decodes the 8-bit angle-offset register back into degrees.
inline float registerToAngleOffset(
    uint8_t reg, const ServoConfig& cfg = ServoConfig::classic()) noexcept {
  const int16_t ticks = static_cast<int16_t>(reg > 0x7F ? reg - 0x100 : reg);
  return static_cast<float>(ticks) * cfg.degPerTick();
}

// ---------------------------------------------------------------------------
// Checksum
// ---------------------------------------------------------------------------

/// Inverted one's-complement sum over the inner payload, i.e. everything
/// after the 0x55 0x55 header and before the checksum byte itself.
inline uint8_t calculateChecksum(const uint8_t* frame, std::size_t size) noexcept {
  unsigned sum = 0;
  for (std::size_t i = kFrameHeaderSize; i < size; ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(~sum) & 0xFFu;
}

/// Same checksum, computed directly over an inner payload (no header bytes).
inline uint8_t checksumOfPayload(const uint8_t* payload, std::size_t size) noexcept {
  unsigned sum = 0;
  for (std::size_t i = 0; i < size; ++i) sum += payload[i];
  return static_cast<uint8_t>(~sum) & 0xFFu;
}

// ---------------------------------------------------------------------------
// Packet building
// ---------------------------------------------------------------------------

/// A fully-framed, checksum-terminated request, ready for `write(2)`.
class Packet {
 public:
  Packet() = default;

  /// Build `0x55 0x55 id len cmd args... checksum`.
  static Packet make(uint8_t id, Command cmd, const uint8_t* args = nullptr,
                     std::size_t arg_count = 0) {
    Packet p;
    p.bytes_.reserve(kFrameOverhead + 2 + arg_count);
    p.bytes_.push_back(kFrameHeader0);
    p.bytes_.push_back(kFrameHeader1);
    p.bytes_.push_back(id);
    p.bytes_.push_back(static_cast<uint8_t>(3 + arg_count));  // inner length
    p.bytes_.push_back(static_cast<uint8_t>(cmd));
    for (std::size_t i = 0; i < arg_count; ++i) p.bytes_.push_back(args[i]);
    p.bytes_.push_back(calculateChecksum(p.bytes_.data(), p.bytes_.size()));
    return p;
  }

  static Packet make(uint8_t id, Command cmd, std::vector<uint8_t> args) {
    return make(id, cmd, args.empty() ? nullptr : args.data(), args.size());
  }

  /// Build a frame addressed to the broadcast id. No servo will ever answer.
  static Packet broadcast(Command cmd, const uint8_t* args = nullptr,
                          std::size_t arg_count = 0) {
    return make(kBroadcastId, cmd, args, arg_count);
  }

  /// Broadcast a "release the queued waits" frame to every servo on the bus.
  static Packet broadcastMoveStart() { return broadcast(Command::kMoveStart); }

  /// Convenience: an instant-move frame (`cmd 1`).
  static Packet makeMove(uint8_t id, uint16_t ticks, uint16_t time_ms,
                         bool wait = false) {
    uint8_t args[4];
    putU16(args, ticks);
    putU16(args + 2, time_ms);
    return make(id, wait ? Command::kMoveWait : Command::kMove, args, 4);
  }

  /// Convenience: a read frame (`cmd 3` shape, no arguments).
  static Packet makeRead(uint8_t id, Command cmd) { return make(id, cmd); }

  const uint8_t* data() const noexcept { return bytes_.data(); }
  const std::uint8_t* begin() const noexcept { return bytes_.data(); }
  const std::uint8_t* end() const noexcept { return bytes_.data() + bytes_.size(); }
  std::size_t size() const noexcept { return bytes_.size(); }
  bool empty() const noexcept { return bytes_.empty(); }
  uint8_t operator[](std::size_t i) const noexcept { return bytes_[i]; }

  std::vector<uint8_t>& raw() noexcept { return bytes_; }
  const std::vector<uint8_t>& raw() const noexcept { return bytes_; }

  /// `0x55 0x55 01 03 1C 13` -- handy for logs and for unit tests.
  std::string toHexString() const {
    static const char* kHex = "0123456789ABCDEF";
    std::string out;
    out.reserve(bytes_.size() * 3);
    for (std::size_t i = 0; i < bytes_.size(); ++i) {
      if (i) out.push_back(' ');
      out.push_back(kHex[bytes_[i] >> 4]);
      out.push_back(kHex[bytes_[i] & 0x0F]);
    }
    return out;
  }

 private:
  std::vector<uint8_t> bytes_;
};

// ---------------------------------------------------------------------------
// Response parsing
// ---------------------------------------------------------------------------

/// Outcome of handing a slice of the receive stream to `tryParseFrame()`.
enum class FrameResult {
  kComplete,    ///< A full, checksum-valid frame was consumed.
  kIncomplete,  ///< Need more bytes before a decision can be made.
  kBadChecksum, ///< Full frame, but the checksum did not match.
  kBadHeader,   ///< Bytes were skipped while hunting for `0x55 0x55`.
};

/// A parsed servo reply.
struct Response {
  uint8_t id = kInvalidServoId;
  uint8_t length = 0;
  uint8_t command = 0;  ///< Echoed request opcode, not a status code.
  std::vector<uint8_t> data;
  bool checksum_ok = false;
  bool complete = false;

  Response() = default;
  Response(uint8_t servo_id, uint8_t len, uint8_t cmd, std::vector<uint8_t> d)
      : id(servo_id), length(len), command(cmd), data(std::move(d)) {}

  /// Frame integrity only. SerialBus additionally matches id and command to
  /// the outstanding request before returning a response.
  bool ok() const noexcept { return complete && checksum_ok; }

  bool success() const noexcept { return ok(); }

  std::size_t size() const noexcept { return data.size(); }
  bool empty() const noexcept { return data.empty(); }

  uint8_t operator[](std::size_t i) const noexcept { return data[i]; }

  /// Little-endian u16 at `offset` in the data payload.
  uint16_t u16(std::size_t offset = 0) const noexcept {
    if (offset + 2 > data.size()) return 0;
    return getU16(data.data() + offset);
  }

  /// Signed physical angle, in ticks.
  int16_t rawAngle() const noexcept {
    return data.size() >= 2 ? normalizeRawAngle(u16(0)) : 0;
  }

  /// Position of `servo_id` in degrees, using the classic geometry.
  float physicalAngle(const ServoConfig& cfg = ServoConfig::classic()) const noexcept {
    return signedTicksToAngle(rawAngle(), cfg);
  }

  /// Temperature in deg C.
  uint8_t temperature() const noexcept { return data.empty() ? 0 : data[0]; }

  /// Supply voltage in mV.
  uint16_t voltage() const noexcept { return u16(0); }

  /// LED/flag style boolean (register 0 == `expected`).
  bool asFlag(uint8_t expected = 1) const noexcept {
    return !data.empty() && data[0] == expected;
  }

  std::string toHexString() const {
    static const char* kHex = "0123456789ABCDEF";
    std::string out;
    for (std::size_t i = 0; i < data.size(); ++i) {
      if (i) out.push_back(' ');
      out.push_back(kHex[data[i] >> 4]);
      out.push_back(kHex[data[i] & 0x0F]);
    }
    return out;
  }
};

/// Try to parse one frame out of `bytes[0..available)`.
///
/// On `kComplete` `*consumed` holds the number of bytes to discard from the
/// stream. Leading garbage (bus noise, or a USB-TTL adapter echoing our own
/// TX back at us) is silently skipped; the returned `consumed` counts those
/// bytes too, which is what makes half-duplex operation robust.
inline FrameResult tryParseFrame(const uint8_t* bytes, std::size_t available,
                                 std::size_t* consumed, Response* out) {
  std::size_t i = 0;

  // Hunt for the two-byte sync marker.
  while (i + 1 < available) {
    if (bytes[i] == kFrameHeader0 && bytes[i + 1] == kFrameHeader1) break;
    ++i;
  }
  const bool dropped_leading = i > 0;

  // Need the header plus the id and length bytes before we can size a frame.
  if (available - i < kFrameDataOffset - 1) {
    if (consumed) *consumed = dropped_leading ? i : 0;
    return dropped_leading ? FrameResult::kBadHeader : FrameResult::kIncomplete;
  }

  const std::size_t frame_start = i;
  const uint8_t length = bytes[frame_start + kFrameLengthOffset];
  if (length < 3) {  // cannot be shorter than id + length + cmd
    if (consumed) *consumed = frame_start + 1;
    return FrameResult::kBadHeader;
  }

  const std::size_t frame_size = static_cast<std::size_t>(length) + kFrameOverhead;
  if (available - frame_start < frame_size) {
    if (consumed) *consumed = dropped_leading ? frame_start : 0;
    return dropped_leading ? FrameResult::kBadHeader : FrameResult::kIncomplete;
  }

  const uint8_t* frame = bytes + frame_start;
  // NB: the checksum covers [2, frame_size-1), i.e. it excludes itself.
  const uint8_t want_cksum = calculateChecksum(frame, frame_size - 1);

  if (consumed) *consumed = frame_start + frame_size;

  if (out) {
    out->id = frame[kFrameIdOffset];
    out->length = length;
    out->command = frame[kFrameOpcodeOffset];
    out->data.assign(frame + kFrameDataOffset, frame + frame_size - 1);
    out->complete = true;
    out->checksum_ok = (frame[frame_size - 1] == want_cksum);
  }

  if (!out || !out->checksum_ok) {
    return FrameResult::kBadChecksum;
  }
  // A valid frame was found, even if junk was skipped in front of it: the
  // caller drops `*consumed` (garbage included) and the stream is aligned.
  return FrameResult::kComplete;
}

/// True when `frame` looks like a plausible servo reply (used to validate a
/// stream without committing it).
inline bool looksLikeFrame(const uint8_t* bytes, std::size_t available) noexcept {
  return available >= 2 && bytes[0] == kFrameHeader0 && bytes[1] == kFrameHeader1;
}

}  // namespace HiwonderBus

#endif  // HIWONDERBUS_PROTOCOL_HPP
