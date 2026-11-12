// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// include/HiwonderBus/testing/simulator.hpp
//   A header-only, in-process model of the classic bus. It implements
//   `HiwonderBus::Transport` and speaks the real protocol, so `SerialBus` and
//   `Servo` run completely unmodified against it -- no serial port, no
//   hardware, fully deterministic.
//
//   This is the same machinery the SDK's own unit tests run on, exposed so
//   that application code can be tested too:
//
//     #include <HiwonderBus/testing/simulator.hpp>
//     auto sim = std::make_unique<HiwonderBus::testing::SimulatedTransport>(
//         std::vector<std::uint8_t>{1, 2, 3});
//     HiwonderBus::BusConfig config;
//     config.device = "simulator";
//     HiwonderBus::SerialBus bus(std::move(sim), config);
//     bus.open("simulator");   // the same SerialBus/Servo code as on real hardware
//
//   What it models per servo:
//     * every readable register (id, offset, limits, temp, vin, torque, led, ...)
//     * the write commands the SDK actually sends, including MOVE_WAIT and the
//       broadcast MOVE_START that releases them
//     * absence:   `disconnect(id)` makes every request to that id time out
//     * corruption:`corruptResponses(id, n)` flips the checksum of `n` replies
//     * timeout:   `hangResponses(id, n)` withholds `n` replies
//     * echo:      `echoTx(true)` hands our own TX bytes back, which the bus
//       must survive thanks to frame resync + echo rejection
//
#ifndef HIWONDERBUS_TESTING_SIMULATOR_HPP
#define HIWONDERBUS_TESTING_SIMULATOR_HPP

#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "HiwonderBus/HiwonderBus.hpp"

namespace HiwonderBus {
namespace testing {

using HiwonderBus::Command;
using HiwonderBus::Packet;

/// Everything one simulated servo remembers.
struct SimulatedServo {
  std::uint8_t id = 0;
  std::int16_t physical_angle_ticks = 0;
  std::int16_t angle_offset_ticks = 0;
  std::uint16_t angle_limit_min_ticks = 0;
  std::uint16_t angle_limit_max_ticks = 1000;
  std::uint16_t vin_limit_min_mv = 4500;
  std::uint16_t vin_limit_max_mv = 12000;
  std::uint8_t temp_limit_c = 90;
  std::uint8_t temp_c = 37;
  std::uint16_t vin_mv = 7400;
  bool torque_enabled = false;
  bool motor_mode = false;
  std::int16_t motor_speed = 0;
  bool led_powered = true;
  std::uint8_t led_error_triggers = 0;
  std::uint16_t last_instant_ticks = 0;
  std::uint16_t last_instant_time = 0;
  std::uint16_t last_delayed_ticks = 0;
  std::uint16_t last_delayed_time = 0;

  bool present = true;   ///< false == answered to nobody, ever
  int hang_count = 0;    ///< replies are withheld for this many requests

  /* Set by MOVE_WAIT (opcode 7) and applied on MOVE_START (opcode 11). */
  std::uint16_t pending_ticks = 0;
  std::uint16_t pending_time = 0;
  bool has_pending = false;

  /// A servo with sane defaults for a fresh object.
  static SimulatedServo make(std::uint8_t servo_id, std::int16_t angle_ticks = 0) {
    SimulatedServo s;
    s.id = servo_id;
    s.physical_angle_ticks = angle_ticks;
    return s;
  }
};

/// A `Transport` that answers like a real classic bus servo.
class SimulatedTransport final : public HiwonderBus::Transport {
 public:
  SimulatedTransport() = default;

  explicit SimulatedTransport(const std::vector<std::uint8_t>& ids) {
    for (std::uint8_t id : ids) addServo(SimulatedServo::make(id));
  }

  // --- Transport ---------------------------------------------------------

  bool open(std::string* error) override {
    (void)error;
    opened_ = true;
    return true;
  }

  void close() noexcept override { opened_ = false; }

  bool isOpen() const noexcept override { return opened_; }

  const char* name() const noexcept override { return "simulator"; }

  long writeBytes(const std::uint8_t* data, std::size_t count, int timeout_ms,
                  std::string* error) override {
    (void)timeout_ms;
    if (!opened_) {
      if (error) *error = "simulator is not open";
      return -1;
    }
    ++write_calls_;
    tx_bytes_written_ += count;
    tx_log_.insert(tx_log_.end(), data, data + count);

    if (echo_tx_) {
      // Some USB-TTL bridges hand our own bytes straight back.
      rx_.insert(rx_.end(), data, data + count);
    }

    std::size_t offset = 0;
    while (offset + 4 <= count) {
      if (data[offset] != 0x55 || data[offset + 1] != 0x55) {
        ++offset;
        continue;
      }
      // [55][55][id][len][cmd][data...][cksum] -- the length byte lives
      // at +3, and the frame is exactly len + 3 bytes long.
      const std::uint8_t length = data[offset + 3];
      if (length < 3) {
        ++offset;
        continue;
      }
      const std::size_t frame_size = static_cast<std::size_t>(length) + 3;
      if (offset + frame_size > count) break;

      handleFrame(data + offset, frame_size);
      offset += frame_size;
    }
    return static_cast<long>(count);
  }

  long readSome(std::uint8_t* out, std::size_t cap, int timeout_ms,
                std::string* error) override {
    (void)timeout_ms;
    (void)error;
    if (!opened_) return -1;
    if (rx_.empty()) return 0;  // nothing queued == "the deadline passed"
    const std::size_t n = std::min<std::size_t>(cap, rx_.size());
    std::copy_n(rx_.begin(), n, out);
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(n));
    return static_cast<long>(n);
  }

  void flushInput() noexcept override { rx_.clear(); }
  void flushOutput() noexcept override {}

  /// The model has no line discipline to reconfigure, but it accepts the new
  /// config so `SerialBus::applyConfig()`/`update_config()` behave the same as
  /// they do against a real UART.
  bool applyConfig(const HiwonderBus::BusConfig& config, std::string* error) override {
    (void)error;
    config_ = config;
    return true;
  }

  // --- Test control ------------------------------------------------------

  SimulatedServo& addServo(SimulatedServo servo) {
    return servos_[servo.id] = std::move(servo);
  }

  SimulatedServo* servo(std::uint8_t id) {
    auto it = servos_.find(id);
    return it == servos_.end() ? nullptr : &it->second;
  }

  const SimulatedServo* servo(std::uint8_t id) const {
    auto it = servos_.find(id);
    return it == servos_.end() ? nullptr : &it->second;
  }

  void disconnect(std::uint8_t id) {
    if (SimulatedServo* s = servo(id)) s->present = false;
  }

  void connect(std::uint8_t id) {
    if (SimulatedServo* s = servo(id)) s->present = true;
  }

  /// Make the next `n` responses for `id` come back with a broken checksum.
  void corruptResponses(std::uint8_t id, int n = 1) {
    corrupt_counts_[id] = n;
  }

  /// Make the next `n` responses for `id` never arrive (timeout).
  void hangResponses(std::uint8_t id, int n = 1) {
    if (SimulatedServo* s = servo(id)) s->hang_count = n;
  }

  void echoTx(bool enable) { echo_tx_ = enable; }

  std::size_t writeCalls() const noexcept { return write_calls_; }
  std::size_t txBytesWritten() const noexcept { return tx_bytes_written_; }
  const std::vector<std::uint8_t>& txLog() const noexcept { return tx_log_; }

  /// Everything the SDK has sent so far, split into frames.
  std::vector<Packet> txFrames() const {
    std::vector<Packet> frames;
    std::size_t offset = 0;
    const std::vector<std::uint8_t>& data = tx_log_;
    while (offset + 4 <= data.size()) {
      if (data[offset] != 0x55 || data[offset + 1] != 0x55) {
        ++offset;
        continue;
      }
      const std::uint8_t length = data[offset + 3];
      if (length < 3) {
        ++offset;
        continue;
      }
      const std::size_t frame_size = static_cast<std::size_t>(length) + 3;
      if (offset + frame_size > data.size()) break;
      std::vector<std::uint8_t> bytes(data.begin() + static_cast<std::ptrdiff_t>(offset),
                                      data.begin() +
                                          static_cast<std::ptrdiff_t>(offset + frame_size));
      Packet frame;
      frame.raw() = std::move(bytes);
      frames.push_back(std::move(frame));
      offset += frame_size;
    }
    return frames;
  }

  /// Bytes the simulator queued but the bus has not read yet.
  const std::vector<std::uint8_t>& pendingRx() const noexcept { return rx_; }

  void reset() {
    rx_.clear();
    tx_log_.clear();
    write_calls_ = 0;
    tx_bytes_written_ = 0;
  }

 private:
  /// Build a reply frame: 0x55 0x55 id length command data... checksum
  static std::vector<std::uint8_t> makeReply(std::uint8_t id, std::uint8_t command,
                                             const std::vector<std::uint8_t>& data) {
    std::vector<std::uint8_t> frame{0x55, 0x55, id,
                                    static_cast<std::uint8_t>(3 + data.size()), command};
    frame.insert(frame.end(), data.begin(), data.end());
    frame.push_back(HiwonderBus::calculateChecksum(frame.data(), frame.size()));
    return frame;
  }

  static std::vector<std::uint8_t> u16le(std::uint16_t v) {
    return {static_cast<std::uint8_t>(v & 0xFF), static_cast<std::uint8_t>((v >> 8) & 0xFF)};
  }

  void handleFrame(const std::uint8_t* frame, std::size_t size) {
    const std::uint8_t id = frame[2];
    const std::uint8_t cmd = frame[4];
    const std::vector<std::uint8_t> args(frame + 5, frame + size - 1);

    const bool broadcast = (id == HiwonderBus::kBroadcastId);

    auto targets = [&](void) -> std::vector<SimulatedServo*> {
      std::vector<SimulatedServo*> out;
      if (broadcast) {
        for (auto& entry : servos_) {
          if (entry.second.present) out.push_back(&entry.second);
        }
      } else {
        if (SimulatedServo* s = servo(id)) out.push_back(s);
      }
      return out;
    };

    // Unicast: only the addressed servo can answer.
    std::vector<SimulatedServo*> touched = targets();
    if (touched.empty()) return;
    if (!broadcast) {
      if (touched.empty()) return;  // addressed to a servo we never gave a model
      if (!touched.front()->present) return;
      if (touched.front()->hang_count > 0) {
        touched.front()->hang_count -= 1;
        return;
      }
    }

    std::vector<std::uint8_t> reply;

    switch (static_cast<Command>(cmd)) {
      // ---- motion ----------------------------------------------------
      case Command::kMove: {
        if (args.size() < 4) return;
        const std::uint16_t ticks = static_cast<std::uint16_t>(args[0] | (args[1] << 8));
        const std::uint16_t time = static_cast<std::uint16_t>(args[2] | (args[3] << 8));
        for (SimulatedServo* s : touched) {
          if (s->motor_mode) continue;
          s->physical_angle_ticks = static_cast<std::int16_t>(ticks);
          s->last_instant_ticks = ticks;
          s->last_instant_time = time;
        }
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kMoveWait: {
        if (args.size() < 4) return;
        const std::uint16_t ticks = static_cast<std::uint16_t>(args[0] | (args[1] << 8));
        const std::uint16_t time = static_cast<std::uint16_t>(args[2] | (args[3] << 8));
        for (SimulatedServo* s : touched) {
          if (s->motor_mode) continue;
          s->last_delayed_ticks = ticks;
          s->last_delayed_time = time;
          s->pending_ticks = ticks;
          s->pending_time = time;
          s->has_pending = true;
        }
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kMoveStart: {
        for (SimulatedServo* s : touched) {
          if (s->motor_mode || !s->has_pending) continue;
          s->physical_angle_ticks = static_cast<std::int16_t>(s->pending_ticks);
          s->last_instant_ticks = s->pending_ticks;
          s->last_instant_time = s->pending_time;
          s->has_pending = false;
        }
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kMoveStop: {
        for (SimulatedServo* s : touched) s->has_pending = false;
        reply = makeReply(id, 0, {});
        break;
      }

      // ---- identity -------------------------------------------------
      case Command::kWriteId: {
        if (args.empty()) return;
        const std::uint8_t new_id = args[0];
        if (broadcast) {
          for (SimulatedServo* s : touched) s->id = new_id;
        } else {
          auto it = servos_.find(id);
          if (it == servos_.end()) return;
          SimulatedServo moved = it->second;  // copy before the key goes away
          servos_.erase(it);
          moved.id = new_id;
          servos_[new_id] = moved;
        }
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kReadId:
        reply = makeReply(id, 0, {id});
        break;

      // ---- calibration ----------------------------------------------
      case Command::kWriteAngleOffset: {
        if (args.empty()) return;
        const std::int8_t value = static_cast<std::int8_t>(args[0]);
        for (SimulatedServo* s : touched) s->angle_offset_ticks = value;
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kSaveAngleOffset:
        reply = makeReply(id, 0, {});
        break;
      case Command::kReadAngleOffset:
        reply = makeReply(
            id, 0,
            {static_cast<std::uint8_t>(static_cast<std::uint16_t>(touched.front()->angle_offset_ticks) &
                                       0xFF)});
        break;

      case Command::kWriteAngleLimits: {
        if (args.size() < 4) return;
        const std::uint16_t lo = static_cast<std::uint16_t>(args[0] | (args[1] << 8));
        const std::uint16_t hi = static_cast<std::uint16_t>(args[2] | (args[3] << 8));
        for (SimulatedServo* s : touched) {
          s->angle_limit_min_ticks = lo;
          s->angle_limit_max_ticks = hi;
        }
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kReadAngleLimits: {
        const SimulatedServo* s = touched.front();
        std::vector<std::uint8_t> data = u16le(s->angle_limit_min_ticks);
        const std::vector<std::uint8_t> hi = u16le(s->angle_limit_max_ticks);
        data.insert(data.end(), hi.begin(), hi.end());
        reply = makeReply(id, 0, data);
        break;
      }

      case Command::kWriteVinLimits: {
        if (args.size() < 4) return;
        const std::uint16_t lo = static_cast<std::uint16_t>(args[0] | (args[1] << 8));
        const std::uint16_t hi = static_cast<std::uint16_t>(args[2] | (args[3] << 8));
        for (SimulatedServo* s : touched) {
          s->vin_limit_min_mv = lo;
          s->vin_limit_max_mv = hi;
        }
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kReadVinLimits: {
        const SimulatedServo* s = touched.front();
        std::vector<std::uint8_t> data = u16le(s->vin_limit_min_mv);
        const std::vector<std::uint8_t> hi = u16le(s->vin_limit_max_mv);
        data.insert(data.end(), hi.begin(), hi.end());
        reply = makeReply(id, 0, data);
        break;
      }

      case Command::kWriteTempLimit: {
        if (args.empty()) return;
        for (SimulatedServo* s : touched) s->temp_limit_c = args[0];
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kReadTempLimit:
        reply = makeReply(id, 0, {touched.front()->temp_limit_c});
        break;

      // ---- telemetry ------------------------------------------------
      case Command::kReadTemp:
        reply = makeReply(id, 0, {touched.front()->temp_c});
        break;
      case Command::kReadVin: {
        std::vector<std::uint8_t> data = u16le(touched.front()->vin_mv);
        reply = makeReply(id, 0, data);
        break;
      }
      case Command::kReadPhysicalAngle: {
        const std::uint16_t raw = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(touched.front()->physical_angle_ticks) & 0xFFFF);
        reply = makeReply(id, 0, u16le(raw));
        break;
      }
      case Command::kReadLastInstantMove: {
        const SimulatedServo* s = touched.front();
        std::vector<std::uint8_t> data = u16le(s->last_instant_ticks);
        const std::vector<std::uint8_t> time = u16le(s->last_instant_time);
        data.insert(data.end(), time.begin(), time.end());
        reply = makeReply(id, 0, data);
        break;
      }
      case Command::kReadLastDelayedMove: {
        const SimulatedServo* s = touched.front();
        std::vector<std::uint8_t> data = u16le(s->last_delayed_ticks);
        const std::vector<std::uint8_t> time = u16le(s->last_delayed_time);
        data.insert(data.end(), time.begin(), time.end());
        reply = makeReply(id, 0, data);
        break;
      }

      // ---- operating mode -------------------------------------------
      case Command::kWriteMotorMode: {
        if (args.size() < 4) return;
        const bool motor = args[0] == 1;
        const std::uint16_t raw = static_cast<std::uint16_t>(args[2] | (args[3] << 8));
        const std::int16_t speed = static_cast<std::int16_t>(raw);
        for (SimulatedServo* s : touched) {
          s->motor_mode = motor;
          s->motor_speed = motor ? speed : 0;
        }
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kReadMotorMode: {
        const SimulatedServo* s = touched.front();
        const std::uint16_t raw = static_cast<std::uint16_t>(s->motor_speed);
        std::vector<std::uint8_t> data{
            static_cast<std::uint8_t>(s->motor_mode ? 1 : 0), 0};
        const std::vector<std::uint8_t> speed = u16le(raw);
        data.insert(data.end(), speed.begin(), speed.end());
        reply = makeReply(id, 0, data);
        break;
      }

      case Command::kWriteTorque: {
        if (args.empty()) return;
        const bool enable = args[0] == 1;
        for (SimulatedServo* s : touched) s->torque_enabled = enable;
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kReadTorque:
        reply = makeReply(id, 0, {static_cast<std::uint8_t>(
                                     touched.front()->torque_enabled ? 1 : 0)});
        break;

      case Command::kWriteLed: {
        if (args.empty()) return;
        // Active-low: 0 == powered.
        const bool powered = args[0] == 0;
        for (SimulatedServo* s : touched) s->led_powered = powered;
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kReadLed:
        reply = makeReply(id, 0,
                          {static_cast<std::uint8_t>(touched.front()->led_powered ? 0 : 1)});
        break;

      case Command::kWriteLedErrorTriggers: {
        if (args.empty()) return;
        for (SimulatedServo* s : touched) s->led_error_triggers = args[0];
        reply = makeReply(id, 0, {});
        break;
      }
      case Command::kReadLedErrorTriggers:
        reply = makeReply(id, 0, {touched.front()->led_error_triggers});
        break;

      default:
        return;  // classic protocol defines no status/error acknowledgement
    }

    // Only read commands answer, and never when addressed to broadcast.
    if (broadcast || reply.empty()) return;
    switch (static_cast<Command>(cmd)) {
      case Command::kReadId:
      case Command::kReadAngleOffset:
      case Command::kReadAngleLimits:
      case Command::kReadVinLimits:
      case Command::kReadTempLimit:
      case Command::kReadTemp:
      case Command::kReadVin:
      case Command::kReadPhysicalAngle:
      case Command::kReadLastInstantMove:
      case Command::kReadLastDelayedMove:
      case Command::kReadMotorMode:
      case Command::kReadTorque:
      case Command::kReadLed:
      case Command::kReadLedErrorTriggers:
        break;
      default:
        return;
    }
    // All replies echo the request opcode, not an invented success byte.
    reply[kFrameOpcodeOffset] = cmd;
    reply.back() = HiwonderBus::calculateChecksum(reply.data(), reply.size() - 1);

    auto corrupt = corrupt_counts_.find(id);
    if (!broadcast && corrupt != corrupt_counts_.end() && corrupt->second > 0) {
      corrupt->second -= 1;
      reply.back() = static_cast<std::uint8_t>(reply.back() ^ 0xFF);
      if (corrupt->second == 0) corrupt_counts_.erase(corrupt);
    }

    rx_.insert(rx_.end(), reply.begin(), reply.end());
  }

  bool opened_ = false;
  HiwonderBus::BusConfig config_;
  std::map<std::uint8_t, SimulatedServo> servos_;
  std::vector<std::uint8_t> rx_;
  std::vector<std::uint8_t> tx_log_;
  std::map<std::uint8_t, int> corrupt_counts_;
  std::size_t write_calls_ = 0;
  std::size_t tx_bytes_written_ = 0;
  bool echo_tx_ = false;
};

}  // namespace testing
}  // namespace HiwonderBus

#endif  // HIWONDERBUS_TESTING_SIMULATOR_HPP
