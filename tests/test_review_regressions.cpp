// SPDX-License-Identifier: MIT
#include <HiwonderBus/HiwonderBus.hpp>
#include <HiwonderBus/testing/simulator.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace HiwonderBus;
using testing::SimulatedTransport;

void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
template<class Exception, class F> void throws(F fn) {
  bool caught = false;
  try { fn(); } catch (const Exception&) { caught = true; }
  require(caught, "expected typed exception");
}

// Independent byte pipe: does not generate responses using Packet/Response or
// the servo simulator. A script receives the request and returns literal bytes.
class ScriptedTransport final : public Transport {
 public:
  bool opened = false;
  std::size_t chunk = 128;
  bool partial_write = false;
  bool repeat_unrelated = false;
  int writes = 0;
  std::vector<std::uint8_t> rx;
  std::function<std::vector<std::uint8_t>(const std::uint8_t*, std::size_t)> script;
  bool open(std::string*) override { opened = true; return true; }
  void close() noexcept override { opened = false; }
  bool isOpen() const noexcept override { return opened; }
  void flushInput() noexcept override { rx.clear(); }
  long writeBytes(const std::uint8_t* bytes, std::size_t count, int, std::string*) override {
    ++writes;
    if (partial_write) return static_cast<long>(count / 2);
    rx = script ? script(bytes, count) : std::vector<std::uint8_t>{};
    return static_cast<long>(count);
  }
  long readSome(std::uint8_t* out, std::size_t cap, int, std::string*) override {
    if (rx.empty() && repeat_unrelated) rx = {0x55,0x55,2,5,28,0x77,1,0x64};
    const auto n = std::min({chunk, cap, rx.size()});
    std::copy_n(rx.begin(), n, out);
    rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(n));
    return static_cast<long>(n);
  }
};

void test_correlation_and_diagnostics() {
  auto pipe = std::make_unique<ScriptedTransport>();
  auto* io = pipe.get();
  io->chunk = 1;
  io->script = [](const std::uint8_t*, std::size_t) {
    return std::vector<std::uint8_t>{
      0x01,0xaa, // leading garbage
      0x55,0x55,2,5,28,0x77,1,0x64, // wrong ID, correct command/shape
      0x55,0x55,1,4,26,40,0xb8, // wrong command AND wrong shape
      0x55,0x55,1,5,28,0x77,1,0x65 // correct ID/command/shape
    };
  };
  BusConfig config;
  config.strict_status = true; // deprecated no-op must not reject command 28
  SerialBus bus(std::move(pipe), config);
  bus.open("script");
  const auto request = Packet::makeRead(1, Command::kReadPhysicalAngle);
  auto response = bus.transact(request, 2);
  require(response.ok() && response.id == 1 && response.command == 28 && response.u16() == 375,
          "failed to skip unrelated fragmented frames");
  io->script = [](const std::uint8_t*, std::size_t) {
    return std::vector<std::uint8_t>{0x55,0x55,2,5,28,0x77,1,0x64};
  };
  ErrorCode error = ErrorCode::kOk;
  require(!bus.tryTransact(request, &response, 2, 10, &error), "accepted wrong servo");
  require(error == ErrorCode::kTimeout && !response.ok(), "wrong-frame timeout/response not reset");
  require(bus.lastErrorCode() == error && !bus.lastError().empty(), "missing cached timeout");
  io->script = [](const std::uint8_t*, std::size_t) {
    return std::vector<std::uint8_t>{0x55,0x55,1,5,28,0x77,1,0x65};
  };
  require(bus.tryTransact(request, &response, 2, -1, &error), "recovery failed");
  require(bus.lastErrorCode() == ErrorCode::kOk && bus.lastError().empty(), "stale recovery diagnostic");
  io->partial_write = true;
  require(!bus.tryWrite(request, &error) && error == ErrorCode::kTimeout, "partial write accepted");
  require(bus.lastErrorCode() == error, "missing partial-write diagnostic");
  io->partial_write = false;
  auto cfg = bus.config();
  cfg.read_timeout_ms = 999;
  require(bus.readTimeoutMs() != 999, "config must be a snapshot");

  cfg.max_retries = 1;
  cfg.read_timeout_ms = 10;
  bus.setConfig(cfg);
  int attempts = 0;
  io->script = [&](const std::uint8_t*, std::size_t) {
    return std::vector<std::uint8_t>{0x55,0x55,1,5,28,0x77,1,
                                    static_cast<std::uint8_t>(++attempts == 1 ? 0 : 0x65)};
  };
  bus.resetStats();
  require(bus.tryTransact(request, &response, 2, -1, &error), "checksum retry failed");
  require(attempts == 2 && bus.stats().checksum_errors == 1 &&
          bus.lastErrorCode() == ErrorCode::kOk, "retry counters/error recovery wrong");
  io->partial_write = true;
  const auto before = io->writes;
  require(!bus.tryTransact(request, &response, 2, -1, &error), "partial request write accepted");
  require(io->writes == before + 1, "partially transmitted request was retried");
  io->partial_write = false;
  cfg.max_retries = 0;
  bus.setConfig(cfg);
  io->script = nullptr;
  io->repeat_unrelated = true;
  const auto start = SerialBus::Clock::now();
  require(!bus.tryTransact(request, &response, 2, 10, &error) && error == ErrorCode::kTimeout,
          "unrelated stream prevented timeout");
  require(SerialBus::Clock::now() - start < std::chrono::milliseconds(500),
          "skipping unrelated replies extended deadline");
}

void test_telemetry_preserves_first_failure() {
  auto pipe = std::make_unique<ScriptedTransport>();
  pipe->script = [](const std::uint8_t* request, std::size_t) {
    switch (request[4]) {
      case 27: return std::vector<std::uint8_t>{0x55,0x55,1,5,27,0x58,0x1b,0x6b};
      case 26: return std::vector<std::uint8_t>{0x55,0x55,1,4,26,40,0xb8};
      default: return std::vector<std::uint8_t>{}; // position times out
    }
  };
  SerialBus bus(std::move(pipe));
  bus.open("script");
  Servo servo(bus, 1);
  throws<ServoTimeoutError>([&] { servo.readTelemetry(); });
  require(servo.lastError() == ErrorCode::kTimeout, "later telemetry success erased failure");
  ErrorCode error = ErrorCode::kOk;
  const auto snapshots = Servo::readTelemetry(bus, {&servo}, &error);
  require(!snapshots[0].valid && snapshots[0].vin_mv == 7000 && snapshots[0].temp_c == 40,
          "telemetry did not preserve successful fields");
  require(error == ErrorCode::kTimeout && servo.lastError() == error, "lost aggregate failure");
}

void test_motion_and_broadcast_safety() {
  auto pipe = std::make_unique<SimulatedTransport>(std::vector<std::uint8_t>{1,2});
  auto* io = pipe.get();
  SerialBus bus(std::move(pipe));
  bus.open("model");
  throws<ServoArgumentError>([&] { Servo bad(bus, 254); });
  throws<ServoArgumentError>([&] { Servo bad(bus, 255); });
  Servo servo(bus, 1);
  servo.enableTorque();
  servo.move(230);
  const auto writes = io->writeCalls();
  throws<ServoArgumentError>([&] { servo.moveRelative(20); });
  require(io->writeCalls() == writes, "invalid relative move transmitted");
  servo.moveRelative(-20);
  require(std::abs(servo.getCommandedAngle() - 209.92f) < 0.25f, "negative relative move failed");
  // The requested C++ compatibility signature: third argument means WAIT.
  servo.move(90, 123, true);
  require(servo.isWaitingForMove() && io->txFrames().back()[4] == 7,
          "third C++ move argument must be wait, not relative");
  servo.moveStart();
  servo.setAngleLimits(50, 100);
  ErrorCode error = ErrorCode::kOk;
  const auto before = io->writeCalls();
  require(Servo::moveSync(bus, {&servo}, {200}, 100, true, &error) == 0 &&
          error == ErrorCode::kInvalidArgument, "batch bypassed travel limits");
  require(io->writeCalls() == before, "invalid batch partially transmitted");
  require(bus.queueMoves({1,254}, {100,100}, 10, true, &error) == 0 &&
          error == ErrorCode::kInvalidArgument, "reserved batch ID accepted");
  require(io->writeCalls() == before, "invalid raw batch partially transmitted");

  // Hardware polls must reconcile authorization state and speed mirrors.
  auto* model = io->servo(1);
  model->torque_enabled = false;
  require(!servo.isTorqueEnabled(true) && !servo.isTorqueEnabled(false), "torque mirror stale");
  throws<ServoLogicalError>([&] { servo.move(75); });
  model->torque_enabled = true;
  require(servo.isTorqueEnabled(true), "torque hardware read failed");
  model->motor_mode = true;
  model->motor_speed = -500;
  require(servo.getMotorSpeed(true) == -500 && servo.isMotorMode(false), "mode/speed mirrors stale");
  throws<ServoLogicalError>([&] { servo.move(75); });
  model->motor_mode = false;
  require(!servo.isMotorMode(true) && !servo.isMotorMode(false), "servo-mode mirror stale");

  // A disconnected model must not hear even broadcast commands.
  model->physical_angle_ticks = 0;
  io->disconnect(1);
  bus.command(Packet::makeMove(kBroadcastId, 500, 0, true));
  bus.moveStart();
  io->connect(1);
  require(model->physical_angle_ticks == 0, "disconnected model heard broadcast");
  require(io->servo(2)->physical_angle_ticks == 500, "connected model missed broadcast");

  Servo centered(bus, 2, ServoConfig(1000, -120, 120));
  centered.enableTorque();
  centered.move(60);
  centered.moveRelative(-20);
  require(std::abs(centered.getCommandedAngle() - 40) < 0.25f,
          "relative move ignored geometry origin");

  // Nonthrowing mode suppresses hardware errors only, not invalid commands.
  bus.setErrorMode(ErrorMode::kNonThrowing);
  throws<ServoArgumentError>([&] { servo.move(999); });
}

void test_shared_bus_snapshots() {
  auto pipe = std::make_unique<SimulatedTransport>(std::vector<std::uint8_t>{1});
  SerialBus bus(std::move(pipe));
  bus.open("model");
  std::thread writer([&] {
    for (int i = 0; i < 300; ++i) {
      Response response;
      bus.tryTransact(Packet::makeRead(1, Command::kReadTemp), &response, 1);
      bus.setErrorMode(i % 2 ? ErrorMode::kStrict : ErrorMode::kNonThrowing);
    }
  });
  for (int i = 0; i < 300; ++i) {
    (void)bus.config(); (void)bus.device(); (void)bus.lastError();
    (void)bus.lastErrorCode(); (void)bus.stats(); (void)bus.errorMode();
    bus.setReadTimeoutMs(20);
  }
  writer.join();
}

int main() {
  try {
    test_correlation_and_diagnostics();
    test_telemetry_preserves_first_failure();
    test_motion_and_broadcast_safety();
    test_shared_bus_snapshots();
    std::cout << "Review regression tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
