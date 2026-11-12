// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// tests/test_protocol.cpp
//   Unit tests for the classic 0x55 0x55 framing, checksums, angle/tick
//   conversions and the end-to-end stack, driven by the header-only servo
//   simulator in tests/fake_transport.hpp. No hardware, no serial port,
//   fully deterministic.
//
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "HiwonderBus/HiwonderBus.hpp"
#include "fake_transport.hpp"

using namespace HiwonderBus;
using hiwonder_test::FakeTransport;
using hiwonder_test::SimulatedServo;

namespace {

int g_checks = 0;
int g_failures = 0;
std::string g_current_test;

/// Streaming for the enum classes, so the check macros can print them.
std::ostream& operator<<(std::ostream& os, ErrorCode code) { return os << toString(code); }
[[maybe_unused]] std::ostream& operator<<(std::ostream& os, FrameResult result) {
  return os << static_cast<int>(result);
}
[[maybe_unused]] std::ostream& operator<<(std::ostream& os, Command cmd) {
  return os << static_cast<int>(cmd);
}

void reportFailure(const char* file, int line, const std::string& expression,
                   const std::string& extra) {
  ++g_failures;
  std::cerr << "  FAIL [" << g_current_test << "] " << file << ":" << line << ": "
            << expression;
  if (!extra.empty()) std::cerr << "  -> " << extra;
  std::cerr << "\n";
}

#define CHECK(expr)                                                            \
  do {                                                                        \
    ++g_checks;                                                               \
    if (!(expr)) reportFailure(__FILE__, __LINE__, #expr, std::string());     \
  } while (0)

#define CHECK_MSG(expr, msg)                                                   \
  do {                                                                        \
    ++g_checks;                                                               \
    if (!(expr)) reportFailure(__FILE__, __LINE__, #expr, (msg));             \
  } while (0)

#define CHECK_EQ(a, b)                                                         \
  do {                                                                        \
    ++g_checks;                                                               \
    if (!((a) == (b))) {                                                      \
      std::ostringstream oss;                                                 \
      oss << #a " == " #b " (lhs=" << (a) << ", rhs=" << (b) << ")";          \
      reportFailure(__FILE__, __LINE__, #a " == " #b, oss.str());             \
    }                                                                         \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                                  \
  do {                                                                        \
    ++g_checks;                                                               \
    if (!(std::fabs((a) - (b)) <= (eps))) {                                   \
      std::ostringstream oss;                                                 \
      oss << #a " ~= " << (b) << " (got " << (a) << ")";                      \
      reportFailure(__FILE__, __LINE__, #a, oss.str());                       \
    }                                                                         \
  } while (0)

#define CHECK_THROWS(expr, ExceptionType)                                      \
  do {                                                                        \
    ++g_checks;                                                               \
    bool caught = false;                                                      \
    try {                                                                     \
      expr;                                                                   \
    } catch (const ExceptionType& e) {                                        \
      caught = true;                                                          \
    } catch (const std::exception& e) {                                       \
      reportFailure(__FILE__, __LINE__, #expr,                                \
                    std::string("wrong exception: ") + e.what());             \
      caught = true;                                                          \
    }                                                                         \
    if (!caught) reportFailure(__FILE__, __LINE__, #expr,                     \
                               "expected " #ExceptionType);                   \
  } while (0)

#define CHECK_NO_THROW(expr)                                                   \
  do {                                                                        \
    ++g_checks;                                                               \
    try {                                                                     \
      expr;                                                                   \
    } catch (const std::exception& e) {                                       \
      reportFailure(__FILE__, __LINE__, #expr,                                \
                    std::string("unexpected exception: ") + e.what());        \
    }                                                                         \
  } while (0)

std::string hex(const std::vector<std::uint8_t>& bytes) {
  static const char* kHex = "0123456789ABCDEF";
  std::string out;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i) out += ' ';
    out += kHex[bytes[i] >> 4];
    out += kHex[bytes[i] & 0x0F];
  }
  return out;
}

std::vector<std::uint8_t> bytesOf(const Packet& p) {
  return std::vector<std::uint8_t>(p.begin(), p.end());
}

/// A bus wired to a simulator instead of a UART.
std::unique_ptr<SerialBus> makeSimulatedBus(std::unique_ptr<FakeTransport> transport,
                                            ErrorMode mode = ErrorMode::kStrict) {
  BusConfig config;
  config.device = "simulator";
  config.read_timeout_ms = 50;
  config.write_timeout_ms = 50;
  config.error_mode = mode;
  auto bus = std::make_unique<SerialBus>(std::move(transport), config);
  if (!bus->open("simulator")) {
    reportFailure(__FILE__, __LINE__, "bus->open(\"simulator\")", bus->lastError());
  }
  return bus;
}

// ===========================================================================
// Framing & checksums
// ===========================================================================

void test_reference_frame_matches_known_good_bytes() {
  // The exact frame alexxcode/LX16A-bus puts on the wire for
  //   move(id=1, angle=90 deg, time=500 ms)
  // 90 deg == 375 ticks == 0x0177 -> 77 01 (little-endian) ; 500 -> F4 01
  const Packet packet = Packet::makeMove(1, 375, 500, /*wait=*/false);
  std::vector<std::uint8_t> expected{0x55, 0x55, 0x01, 0x07, 0x01, 0x77,
                                     0x01, 0xF4, 0x01, 0x00};
  std::vector<std::uint8_t> got = bytesOf(packet);
  // The final byte is the checksum; recompute it for the comparison.
  expected.back() = calculateChecksum(expected.data(), expected.size() - 1);
  CHECK_MSG(got == expected, "frame is " + hex(got) + ", expected " + hex(expected));
}

void test_checksum_covers_everything_after_the_header() {
  // Sanity against the reference implementations: sum over bytes [2..n-1],
  // inverted and truncated to 8 bits.
  std::vector<std::uint8_t> frame{0x55, 0x55, 0x01, 0x03, 0x1C};
  const unsigned sum = 0x01 + 0x03 + 0x1C;
  CHECK_EQ(calculateChecksum(frame.data(), frame.size()),
           static_cast<std::uint8_t>((~sum) & 0xFF));
}

void test_packet_length_field_counts_the_inner_payload() {
  // {id, length, cmd} -- no args -- so length == 3 and the frame is 6 bytes.
  const Packet read = Packet::make(7, Command::kReadPhysicalAngle);
  CHECK_EQ(read.size(), 6u);
  CHECK_EQ(read[3], 3);

  // One argument => length == 4, frame == 7 bytes (the setID shape).
  const std::uint8_t arg = 42;
  const Packet write = Packet::make(7, Command::kWriteId, &arg, 1);
  CHECK_EQ(write.size(), 7u);
  CHECK_EQ(write[3], 4);
  CHECK_EQ(write[6], calculateChecksum(write.data(), 6));

  // Broadcast frames address 0xFE and are never answered.
  const Packet broadcast = Packet::broadcastMoveStart();
  CHECK_EQ(broadcast[2], kBroadcastId);
  CHECK_EQ(broadcast[4], static_cast<std::uint8_t>(Command::kMoveStart));
}

void test_packet_hex_and_data_accessors() {
  const Packet packet = Packet::make(1, Command::kReadId);
  CHECK_MSG(packet.toHexString().find("55 55") == 0, packet.toHexString());
  CHECK_EQ(packet.size(), 6u);
  CHECK(!packet.empty());
}

// ===========================================================================
// Angle <-> tick geometry
// ===========================================================================

void test_classic_angle_tick_conversion() {
  // 1000 ticks over 240 deg => 0.24 deg/tick, exactly 25 ticks/deg.
  CHECK_NEAR(ServoConfig::classic().degPerTick(), 0.24f, 1e-6f);
  CHECK_EQ(angleToTicks(0.0f), 0);
  CHECK_EQ(angleToTicks(240.0f), 1000);
  CHECK_EQ(angleToTicks(90.0f), 375);
  CHECK_EQ(angleToTicks(180.0f), 750);
  CHECK_NEAR(ticksToAngle(375), 90.0f, 1e-5f);
  CHECK_NEAR(ticksToAngle(1000), 240.0f, 1e-5f);
}

void test_conversion_round_trips() {
  const ServoConfig cfg = ServoConfig::classic();
  for (float degrees = 0.0f; degrees <= 240.0f; degrees += 0.5f) {
    const std::uint16_t ticks = angleToTicks(degrees, cfg);
    const float back = ticksToAngle(ticks, cfg);
    CHECK_MSG(std::fabs(back - degrees) <= 0.5f * cfg.degPerTick() + 1e-4f,
              "round trip drifted at " + std::to_string(degrees));
  }
}

void test_conversion_clamps_and_survives_nan() {
  CHECK_EQ(angleToTicks(-1.0f), 0);
  CHECK_EQ(angleToTicks(1000.0f), 1000);
  CHECK_EQ(angleToTicks(std::numeric_limits<float>::quiet_NaN()), 0);
}

void test_non_default_geometry_is_respected() {
  // A hypothetical classic-protocol variant with 2000 ticks over 300 deg.
  const ServoConfig custom(2000, 0.0f, 300.0f);
  CHECK_NEAR(custom.degPerTick(), 0.15f, 1e-6f);
  CHECK_EQ(angleToTicks(150.0f, custom), 1000);
  CHECK_NEAR(ticksToAngle(1000, custom), 150.0f, 1e-5f);
  // ...and the SDK-wide default still means the classic part.
  CHECK_EQ(angleToTicks(150.0f), 625);
}

void test_signed_physical_angle() {
  // Registers above 0x7FFF are two's-complement negatives.
  CHECK_EQ(normalizeRawAngle(0x0177), 375);
  CHECK_EQ(normalizeRawAngle(0xFE88), -376);
  CHECK_NEAR(rawAngleToDegrees(0xFE88), -90.24f, 1e-3f);
  const Response negative(1, 4, 0, {0x88, 0xFE});
  CHECK_NEAR(negative.physicalAngle(), -90.24f, 1e-3f);
}

void test_angle_offset_register_round_trip() {
  for (int degrees = -30; degrees <= 30; ++degrees) {
    const std::uint8_t reg = angleOffsetToRegister(static_cast<float>(degrees));
    const float decoded = registerToAngleOffset(reg);
    CHECK_MSG(std::fabs(decoded - degrees) <= 0.25f,
              std::to_string(degrees) + " -> " + std::to_string(reg) + " -> " +
                  std::to_string(decoded));
  }
}

void test_led_triggers_mask() {
  const LedTriggers all{true, true, true};
  CHECK_EQ(all.toMask(), 7);
  CHECK(LedTriggers::fromMask(7).over_temperature);
  CHECK(LedTriggers::fromMask(7).over_voltage);
  CHECK(LedTriggers::fromMask(7).rotor_locked);
  const LedTriggers only_temp{true, false, false};
  CHECK_EQ(only_temp.toMask(), 1);
  CHECK_EQ(LedTriggers::fromMask(4).rotor_locked, true);
  CHECK_EQ(LedTriggers::fromMask(0).over_voltage, false);
}

// ===========================================================================
// Response parsing / resync
// ===========================================================================

std::vector<std::uint8_t> replyFrame(std::uint8_t id, std::uint8_t status,
                                     const std::vector<std::uint8_t>& data) {
  std::vector<std::uint8_t> frame{0x55, 0x55, id, static_cast<std::uint8_t>(3 + data.size()),
                                  status};
  frame.insert(frame.end(), data.begin(), data.end());
  frame.push_back(calculateChecksum(frame.data(), frame.size()));
  return frame;
}

void test_parse_complete_frame() {
  // 90 deg = 375 ticks = 0x0177 -> 77 01
  const std::vector<std::uint8_t> frame = replyFrame(3, 0, {0x77, 0x01});
  std::size_t consumed = 0;
  Response response;
  const FrameResult result = tryParseFrame(frame.data(), frame.size(), &consumed, &response);
  CHECK_EQ(static_cast<int>(result), static_cast<int>(FrameResult::kComplete));
  CHECK_EQ(consumed, frame.size());
  CHECK_EQ(response.id, 3);
  CHECK(response.ok());
  CHECK_EQ(response.command, 0);  // raw parser preserves the opcode byte
  CHECK_NEAR(response.physicalAngle(), 90.0f, 1e-4f);
  CHECK_EQ(response.u16(0), 0x0177u);
}

void test_parse_rejects_bad_checksum() {
  std::vector<std::uint8_t> frame = replyFrame(3, 0, {0x77, 0x01});
  frame.back() ^= 0x5A;  // corrupt the checksum
  std::size_t consumed = 0;
  Response response;
  const FrameResult result = tryParseFrame(frame.data(), frame.size(), &consumed, &response);
  CHECK_EQ(static_cast<int>(result), static_cast<int>(FrameResult::kBadChecksum));
  CHECK(!response.ok());
}

void test_parse_waits_for_more_bytes() {
  const std::vector<std::uint8_t> frame = replyFrame(3, 0, {0xEB, 0x01});
  std::size_t consumed = 0;
  Response response;
  for (std::size_t i = 0; i + 1 < frame.size(); ++i) {
    const FrameResult result = tryParseFrame(frame.data(), i, &consumed, &response);
    CHECK_MSG(result == FrameResult::kIncomplete || result == FrameResult::kBadHeader,
              "truncated at " + std::to_string(i) + " gave " +
                  std::to_string(static_cast<int>(result)));
  }
}

void test_parse_skips_leading_garbage() {
  // Half-duplex bridges and line noise put junk in front of a real reply.
  std::vector<std::uint8_t> stream{0x00, 0xFF, 0x12};
  const std::vector<std::uint8_t> frame = replyFrame(9, 0, {0x2A});
  stream.insert(stream.end(), frame.begin(), frame.end());

  std::size_t consumed = 0;
  Response response;
  // Same resync loop SerialBus::consumeFrameLocked() runs: the primitive
  // reports one step at a time, the caller drops bytes and tries again.
  FrameResult result = FrameResult::kIncomplete;
  std::size_t offset = 0;
  for (int guard = 0; guard < 8; ++guard) {
    consumed = 0;
    result = tryParseFrame(stream.data() + offset, stream.size() - offset, &consumed,
                           &response);
    offset += consumed;
    if (result == FrameResult::kComplete || consumed == 0) break;
  }
  CHECK_EQ(static_cast<int>(result), static_cast<int>(FrameResult::kComplete));
  CHECK_EQ(offset, stream.size());
  CHECK(response.ok());
  CHECK_EQ(response.id, 9);
  CHECK_EQ(response[0], 0x2A);
}

void test_parse_rejects_nonsense_length() {
  const std::vector<std::uint8_t> frame{0x55, 0x55, 0x01, 0x00, 0x00, 0xFF};
  std::size_t consumed = 0;
  Response response;
  const FrameResult result = tryParseFrame(frame.data(), frame.size(), &consumed, &response);
  CHECK_EQ(static_cast<int>(result), static_cast<int>(FrameResult::kBadHeader));
  CHECK_EQ(consumed, 1u);  // skip the bogus sync and keep hunting
}

// ===========================================================================
// End-to-end over the simulator
// ===========================================================================

void test_scan_ids_finds_only_living_servos() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{2, 5, 9});
  transport->disconnect(5);
  auto bus = makeSimulatedBus(std::move(transport));

  const std::vector<std::uint8_t> ids = bus->scanIds(0, 15);
  CHECK_EQ(ids.size(), 2u);
  if (ids.size() == 2) {
    CHECK_EQ(ids[0], 2);
    CHECK_EQ(ids[1], 9);
  }
  bus->close();
}

void test_torque_and_move_round_trip() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 1);
  CHECK(servo.trySetTorque(true));
  CHECK(servo.isTorqueEnabled());

  servo.move(90.0f, 500);
  CHECK_NEAR(servo.getCommandedAngle(), 90.0f, 1e-3f);
  CHECK_NEAR(servo.getPhysicalAngle(false), 90.0f, 1e-3f);

  SimulatedServo* simulated = simulator->servo(1);
  CHECK_MSG(simulated != nullptr, "simulated servo vanished");
  if (simulated) {
    CHECK_EQ(static_cast<int>(simulated->physical_angle_ticks), 375);
    CHECK_EQ(static_cast<int>(simulated->last_instant_time), 500);
    CHECK_EQ(static_cast<int>(simulated->torque_enabled), 1);
  }

  const auto frames = simulator->txFrames();
  CHECK_EQ(frames.size(), 2u);
  if (frames.size() >= 2) {
    CHECK_EQ(frames[1][4], static_cast<std::uint8_t>(Command::kMove));
    CHECK_EQ(static_cast<int>(frames[1][5]), 0x77);  // low tick byte of 375
  }
  bus->close();
}

void test_queued_move_and_broadcast_start() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 1);
  servo.enableTorque();

  // Queue it with opcode 7...
  servo.move(120.0f, 400, /*wait=*/true);
  CHECK(servo.isWaitingForMove());
  CHECK_NEAR(servo.getWaitingAngle(), 120.0f, 1e-3f);
  SimulatedServo* simulated = simulator->servo(1);
  if (simulated) {
    CHECK_MSG(simulated->has_pending, "the target was not queued");
    CHECK_EQ(static_cast<int>(simulated->physical_angle_ticks), 0);  // not started yet
  }

  // ...and release it.
  servo.moveStart();
  CHECK(!servo.isWaitingForMove());
  if (simulated) {
    CHECK_EQ(static_cast<int>(simulated->physical_angle_ticks), 500);  // 120 deg
  }
  CHECK_NEAR(servo.getCommandedAngle(), 120.0f, 1e-2f);
  bus->close();
}

void test_move_sync_is_a_single_write() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1, 2, 3, 4});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  std::vector<Servo*> servos;
  for (std::uint8_t id : {1, 2, 3, 4}) {
    auto* servo = new Servo(*bus, id);
    servo->enableTorque();
    servos.push_back(servo);
  }

  simulator->reset();
  ErrorCode code = ErrorCode::kOk;
  const std::size_t written = Servo::moveSync(
      *bus, servos, {10.0f, 20.0f, 30.0f, 40.0f}, /*time_ms=*/250, /*start=*/true, &code);
  CHECK_EQ(code, ErrorCode::kOk);
  CHECK(written > 0);

  // The whole synchronized move -- four cmd-7 frames plus the 0xFE cmd-11
  // broadcast -- must leave in exactly one write(2).
  CHECK_MSG(simulator->writeCalls() == 1,
            "expected a single write, got " + std::to_string(simulator->writeCalls()));
  const auto frames = simulator->txFrames();
  CHECK_EQ(frames.size(), 5u);
  if (frames.size() == 5) {
    for (std::size_t i = 0; i < 4; ++i) {
      CHECK_EQ(frames[i][4], static_cast<std::uint8_t>(Command::kMoveWait));
      CHECK_EQ(frames[i][2], static_cast<std::uint8_t>(i + 1));
    }
    CHECK_EQ(frames[4][2], kBroadcastId);
    CHECK_EQ(frames[4][4], static_cast<std::uint8_t>(Command::kMoveStart));
  }

  // And the servos really moved.
  for (std::size_t i = 0; i < servos.size(); ++i) {
    const float commanded = 10.0f * (i + 1);
    SimulatedServo* simulated = simulator->servo(static_cast<std::uint8_t>(i + 1));
    if (simulated) {
      CHECK_EQ(static_cast<int>(simulated->physical_angle_ticks),
               static_cast<int>(angleToTicks(commanded)));
    }
    // One tick is 0.24 deg, so the mirror must agree with the quantised target.
    CHECK_NEAR(servos[i]->getCommandedAngle(),
               ticksToAngle(angleToTicks(commanded)), 1e-3f);
    delete servos[i];
  }
  bus->close();
}

void test_batch_reads_and_telemetry() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1, 2, 3});
  FakeTransport* simulator = transport.get();
  for (std::uint8_t id : {1, 2, 3}) {
    if (SimulatedServo* s = simulator->servo(id)) {
      s->physical_angle_ticks = static_cast<std::int16_t>(100 * id);
      s->vin_mv = static_cast<std::uint16_t>(7000 + id);
      s->temp_c = static_cast<std::uint8_t>(30 + id);
      s->torque_enabled = true;
    }
  }
  auto bus = makeSimulatedBus(std::move(transport));

  std::vector<Servo*> servos;
  for (std::uint8_t id : {1, 2, 3}) servos.push_back(new Servo(*bus, id));

  ErrorCode code = ErrorCode::kOk;
  const std::vector<float> angles = Servo::readAngles(*bus, servos, /*poll=*/true, &code);
  CHECK_EQ(code, ErrorCode::kOk);
  CHECK_EQ(angles.size(), 3u);
  for (std::size_t i = 0; i < angles.size(); ++i) {
    CHECK_NEAR(angles[i], ticksToAngle(static_cast<std::uint16_t>(100 * (i + 1))), 1e-4f);
  }

  const std::vector<Telemetry> telemetry = Servo::readTelemetry(*bus, servos, &code);
  CHECK_EQ(telemetry.size(), 3u);
  for (std::size_t i = 0; i < telemetry.size(); ++i) {
    CHECK(telemetry[i].valid);
    CHECK_EQ(telemetry[i].vin_mv, static_cast<int>(7000 + i + 1));
    CHECK_EQ(telemetry[i].temp_c, static_cast<int>(30 + i + 1));
    CHECK_EQ(telemetry[i].id, static_cast<std::uint8_t>(i + 1));
  }

  // A single dead servo must not take the whole sweep down with it.
  simulator->disconnect(2);
  const std::vector<float> partial = Servo::readAngles(*bus, servos, true, &code);
  CHECK_EQ(partial.size(), 3u);
  CHECK(std::isnan(partial[1]));
  CHECK(!std::isnan(partial[0]));
  CHECK(!std::isnan(partial[2]));
  CHECK(code == ErrorCode::kTimeout);

  for (Servo* servo : servos) delete servo;
  bus->close();
}

void test_register_round_trips_through_the_simulator() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{7});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 7);
  servo.enableTorque();

  servo.setAngleLimits(20.0f, 200.0f);
  const AngleLimits limits = servo.getAngleLimits(/*poll_hardware=*/true);
  // One tick is 0.24 deg, so register round-trips quantise to a quarter degree.
  CHECK_NEAR(limits.min, 20.0f, 0.25f);
  CHECK_NEAR(limits.max, 200.0f, 0.25f);

  servo.setVinLimits(5000, 11000);
  const VinLimits vin = servo.getVinLimits(true);
  CHECK_EQ(vin.min, 5000);
  CHECK_EQ(vin.max, 11000);

  servo.setTempLimit(85);
  CHECK_EQ(servo.getTempLimit(true), 85);

  servo.setAngleOffset(-5.0f);
  CHECK_NEAR(servo.getAngleOffset(true), -5.0f, 0.05f);

  servo.setLedErrorTriggers(/*over_temperature=*/true, /*over_voltage=*/false,
                            /*rotor_locked=*/true);
  const LedTriggers triggers = servo.getLedErrorTriggers(true);
  CHECK(triggers.over_temperature);
  CHECK(!triggers.over_voltage);
  CHECK(triggers.rotor_locked);

  servo.ledPowerOn();
  CHECK(servo.isLedPowerOn(true));
  servo.ledPowerOff();
  CHECK(!servo.isLedPowerOn(true));

  servo.setMotorMode(-512);
  CHECK(servo.isMotorMode(true));
  CHECK_EQ(servo.getMotorSpeed(true), -512);
  servo.setServoMode();
  CHECK(!servo.isMotorMode(true));

  SimulatedServo* simulated = simulator->servo(7);
  if (simulated) {
    CHECK_EQ(static_cast<int>(simulated->temp_limit_c), 85);
    CHECK_EQ(static_cast<int>(simulated->vin_limit_min_mv), 5000);
  }

  // And a new id survives the round trip.
  servo.setID(12);
  CHECK_EQ(servo.getID(true), 12);
  CHECK_EQ(servo.id(), 12);
  bus->close();
}

void test_timeouts_and_exception_mapping() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{4});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));  // strict

  Servo servo(*bus, 4);
  simulator->disconnect(4);

  // Strict mode: the mirror is trusted, so cached reads still work...
  CHECK(servo.trySetTorque(true));
  CHECK_NEAR(servo.getCommandedAngle(), 0.0f, 1e-6f);
  CHECK(servo.healthy());

  // ...and a hardware read on a dead servo throws.
  bool threw = false;
  try {
    servo.getPhysicalAngle(true);
  } catch (const ServoTimeoutError& e) {
    threw = true;
    CHECK_MSG(e.id() == 4, "exception must carry the servo id");
  }
  CHECK(threw);

  // The try* API is exception-free in both modes.
  float angle = 1.0f;
  CHECK(!servo.tryGetPhysicalAngle(&angle, true));
  CHECK(servo.lastError() == ErrorCode::kTimeout);

  CHECK_THROWS(servo.getTemp(), ServoTimeoutError);
  bus->close();
}

void test_checksum_error_is_reported() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{6});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 6);
  simulator->corruptResponses(6, 1);

  bool threw = false;
  try {
    servo.getTemp();
  } catch (const ServoChecksumError& e) {
    threw = true;
    CHECK(e.id() == 6);
  }
  CHECK(threw);
  CHECK(servo.lastError() == ErrorCode::kBadChecksum);
  bus->close();
}

void test_mirrors_never_touch_the_wire() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 1);
  servo.enableTorque();
  servo.setAngleLimits(0.0f, 240.0f);
  const std::uint64_t transactions_before = bus->stats().transactions;

  // All of these must be pure cache hits.
  CHECK(servo.isTorqueEnabled(false));
  CHECK_EQ(servo.isMotorMode(false), false);
  (void)servo.getAngleLimits(false);
  (void)servo.getAngleOffset(false);
  (void)servo.getTempLimit(false);
  (void)servo.getVinLimits(false);
  (void)servo.getID(false);
  (void)servo.isLedPowerOn(false);
  (void)servo.getLedErrorTriggers(false);
  CHECK_EQ(servo.getCommandedAngle(), 0.0f);

  CHECK_MSG(bus->stats().transactions == transactions_before,
            "cached getters issued wire traffic: " +
                std::to_string(bus->stats().transactions - transactions_before));
  CHECK_EQ(simulator->writeCalls(), 2u);  // the two writes we actually sent
  bus->close();
}

void test_validation_rules() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1});
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 1);

  // Torque must be enabled before anything moves.
  CHECK_THROWS(servo.move(90.0f, 100), ServoLogicalError);

  servo.enableTorque();

  // Out-of-range angles are rejected, never clamped.
  CHECK_THROWS(servo.move(241.0f), ServoArgumentError);
  CHECK_THROWS(servo.move(-0.5f), ServoArgumentError);
  CHECK_THROWS(servo.move(std::numeric_limits<float>::quiet_NaN()), ServoArgumentError);

  // Limits gate the commanded angle as well.
  servo.setAngleLimits(50.0f, 100.0f);
  CHECK_THROWS(servo.move(49.0f), ServoArgumentError);
  CHECK_THROWS(servo.move(101.0f), ServoArgumentError);
  CHECK_NO_THROW(servo.move(75.0f));

  // Range checks on the other setters.
  CHECK_THROWS(servo.setTempLimit(10), ServoArgumentError);
  CHECK_THROWS(servo.setVinLimits(100, 12000), ServoArgumentError);
  CHECK_THROWS(servo.setAngleOffset(-45.0f), ServoArgumentError);
  CHECK_THROWS(servo.setAngleOffset(30.5f), ServoArgumentError);

  // Motor mode blocks position control and vice versa.
  servo.enableTorque();
  servo.setMotorMode(500);
  CHECK_THROWS(servo.move(60.0f, 100), ServoLogicalError);
  CHECK(servo.getMotorSpeed(false) == 500);
  servo.setServoMode();
  CHECK_THROWS(servo.getMotorSpeed(false), ServoLogicalError);  // not in motor mode

  // Waiting without queuing, or queuing and never starting.
  CHECK_THROWS(servo.moveStart(), ServoLogicalError);
  bus->close();
}

void test_echo_does_not_break_a_transaction() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{3});
  FakeTransport* simulator = transport.get();
  simulator->echoTx(true);  // the bridge hands us our own bytes back
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 3);
  CHECK(servo.trySetTorque(true));
  const BusStats stats_before = bus->stats();
  CHECK(servo.isTorqueEnabled(true));
  CHECK_MSG(bus->stats().echo_drops > stats_before.echo_drops,
            "the echoed frame should have been dropped");
  bus->close();
}

void test_non_throwing_mode_keeps_the_loop_alive() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1, 2});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport), ErrorMode::kNonThrowing);

  Servo live(*bus, 1);
  Servo dead(*bus, 2);
  CHECK(live.trySetTorque(true));
  simulator->disconnect(2);

  // A dead servo must not throw, and must not stop its neighbour.
  CHECK(!dead.isTorqueEnabled(true));
  CHECK(dead.lastError() == ErrorCode::kTimeout);
  (void)dead.getTemp();
  CHECK(dead.lastError() == ErrorCode::kTimeout);

  // The live servo is unaffected.
  live.move(30.0f, 100);
  CHECK(live.healthy());
  CHECK_NEAR(live.getCommandedAngle(), ticksToAngle(angleToTicks(30.0f)), 1e-3f);

  // try_move on the dead one reports the failure without unwinding. Writes are
  // fire-and-forget (no ack is read), so the honest failure signal is a read.
  ErrorCode code = ErrorCode::kOk;
  float dead_angle = 0.0f;
  CHECK(!dead.tryGetPhysicalAngle(&dead_angle, true, &code));
  CHECK(code == ErrorCode::kTimeout);
  bus->close();
}

void test_batch_torque_and_limit_helpers() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1, 2, 3});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  std::vector<Servo*> servos;
  for (std::uint8_t id : {1, 2, 3}) servos.push_back(new Servo(*bus, id));

  simulator->reset();
  const std::size_t torque_bytes = Servo::setTorque(*bus, servos, true);
  CHECK(torque_bytes > 0);
  CHECK_MSG(simulator->writeCalls() == 1,
            "setTorque should batch into one write");
  for (Servo* servo : servos) CHECK(servo->isTorqueEnabled(false));

  simulator->reset();
  const std::size_t limit_bytes = Servo::setAngleLimits(*bus, servos, 10.0f, 170.0f);
  CHECK(limit_bytes > 0);
  CHECK_MSG(simulator->writeCalls() == 1, "setAngleLimits should batch into one write");
  for (Servo* servo : servos) {
    const AngleLimits limits = servo->getAngleLimits(true);
    CHECK_NEAR(limits.min, 10.0f, 0.25f);
    CHECK_NEAR(limits.max, 170.0f, 0.25f);
  }

  for (Servo* servo : servos) delete servo;
  bus->close();
}

void test_bus_stats_and_flush_counters() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1});
  auto bus = makeSimulatedBus(std::move(transport));
  bus->resetStats();

  Servo servo(*bus, 1);
  servo.enableTorque();
  const BusStats stats = bus->stats();
  CHECK(stats.writes >= 1);
  CHECK(stats.tx_bytes >= 6);
  CHECK_EQ(static_cast<int>(bus->transportName()[0]), 's');  // "simulator"

  bus->flushInput();
  CHECK(bus->isOpen());
  bus->close();
  CHECK(!bus->isOpen());
  bus->open("simulator");
  CHECK(bus->isOpen());
}

void test_relative_move_and_last_move_registers() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 1);
  servo.enableTorque();
  servo.move(100.0f, 50);
  servo.moveRelative(50.0f, 25);  // 100 + 50 => 150 deg
  CHECK_NEAR(servo.getCommandedAngle(), 150.0f, 1e-2f);
  if (SimulatedServo* s = simulator->servo(1)) {
    CHECK_EQ(static_cast<int>(s->physical_angle_ticks), 625);
  }

  const auto instant = servo.getLastInstantMove();
  CHECK_NEAR(instant.first, 150.0f, 0.05f);
  CHECK_EQ(static_cast<int>(instant.second), 25);
  bus->close();
}

void test_move_stop_rereads_the_position() {
  auto transport = std::make_unique<FakeTransport>(std::vector<std::uint8_t>{1});
  FakeTransport* simulator = transport.get();
  auto bus = makeSimulatedBus(std::move(transport));

  Servo servo(*bus, 1);
  servo.enableTorque();
  if (SimulatedServo* s = simulator->servo(1)) s->physical_angle_ticks = 250;
  servo.moveStop();
  CHECK_NEAR(servo.getCommandedAngle(), 60.0f, 1e-2f);  // 250 ticks * 0.24
  bus->close();
}

void test_version_and_free_functions() {
  CHECK(versionString().find("HiwonderBus") == 0);
  // Round-tripping through the 0.24 deg tick grid costs at most half a tick.
  CHECK_NEAR(ticksToAngle(angleToTicks(45.0f)), 45.0f, 0.25f);
  CHECK_EQ(static_cast<int>(kBroadcastId), 254);
  CHECK_EQ(static_cast<int>(kMaxMoveTimeMs), 30000);
  CHECK(isTransient(ErrorCode::kTimeout));
  CHECK(!isTransient(ErrorCode::kInvalidArgument));
  CHECK(toString(ErrorCode::kTimeout) != nullptr);
}

}  // namespace

// ===========================================================================
// Runner
// ===========================================================================

#define RUN(test)                                     \
  do {                                                \
    g_current_test = #test;                           \
    const int before = g_failures;                    \
    std::cout << "[ RUN ] " << #test << "\n";         \
    test();                                           \
    std::cout << (g_failures == before ? "[ OK ]  " : "[FAIL] ") << #test << "\n"; \
  } while (0)

int main() {
  std::cout << "hiwonder_servo_classic protocol tests\n";

  // framing / checksum
  RUN(test_reference_frame_matches_known_good_bytes);
  RUN(test_checksum_covers_everything_after_the_header);
  RUN(test_packet_length_field_counts_the_inner_payload);
  RUN(test_packet_hex_and_data_accessors);

  // geometry
  RUN(test_classic_angle_tick_conversion);
  RUN(test_conversion_round_trips);
  RUN(test_conversion_clamps_and_survives_nan);
  RUN(test_non_default_geometry_is_respected);
  RUN(test_signed_physical_angle);
  RUN(test_angle_offset_register_round_trip);
  RUN(test_led_triggers_mask);

  // response parsing
  RUN(test_parse_complete_frame);
  RUN(test_parse_rejects_bad_checksum);
  RUN(test_parse_waits_for_more_bytes);
  RUN(test_parse_skips_leading_garbage);
  RUN(test_parse_rejects_nonsense_length);

  // end to end over the simulator
  RUN(test_scan_ids_finds_only_living_servos);
  RUN(test_torque_and_move_round_trip);
  RUN(test_queued_move_and_broadcast_start);
  RUN(test_move_sync_is_a_single_write);
  RUN(test_batch_reads_and_telemetry);
  RUN(test_register_round_trips_through_the_simulator);
  RUN(test_timeouts_and_exception_mapping);
  RUN(test_checksum_error_is_reported);
  RUN(test_mirrors_never_touch_the_wire);
  RUN(test_validation_rules);
  RUN(test_echo_does_not_break_a_transaction);
  RUN(test_non_throwing_mode_keeps_the_loop_alive);
  RUN(test_batch_torque_and_limit_helpers);
  RUN(test_bus_stats_and_flush_counters);
  RUN(test_relative_move_and_last_move_registers);
  RUN(test_move_stop_rereads_the_position);
  RUN(test_version_and_free_functions);

  std::cout << "\n"
            << (g_failures == 0 ? "" : "FAILED ") << g_checks << " checks, "
            << g_failures << " failure" << (g_failures == 1 ? "" : "s") << "\n";
  if (g_failures == 0) {
    std::cout << "All tests passed\n";
    return 0;
  }
  return 1;
}
