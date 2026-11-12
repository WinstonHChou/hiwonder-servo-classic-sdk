// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// serial_bus.hpp
//   Native Linux POSIX serial transport (termios + poll(2)) with framed
//   transactions, echo rejection and single-syscall batch writes.
//
// Design notes
// ------------
// * Everything blocking goes through poll(2) with an absolute deadline taken
//   from CLOCK_MONOTONIC: no sleep(), no busy delay(), and a reply wakes the
//   caller the moment the 6..10 byte packet lands.
// * Multi-servo commands are concatenated into one buffer and pushed with a
//   single write(2), which is what makes 10+ DOF synchronized moves cheap.
// * `Transport` is an injection seam: the production impl is the POSIX one,
//   while tests and simulators can supply their own without re-compiling the
//   protocol stack.
//
#ifndef HIWONDERBUS_SERIAL_BUS_HPP
#define HIWONDERBUS_SERIAL_BUS_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "HiwonderBus/exceptions.hpp"
#include "HiwonderBus/protocol.hpp"

namespace HiwonderBus {

/// How hardware failures are reported.
enum class ErrorMode : std::uint8_t {
  /// Throw the matching exception (PyLX-16A behaviour). This is the default:
  /// silent failure on an actuator is a safety problem.
  kStrict = 0,
  /// Never throw for *hardware* faults. Return the `try*` variant's status and
  /// cache the last error, so one dead servo cannot halt a control loop.
  /// Argument/state errors still throw, and the `try*` API is always
  /// exception-free in both modes.
  kNonThrowing = 1,
};

/// Tunables for a bus handle.
struct BusConfig {
  /// Character device: /dev/ttyUSB0, /dev/ttyACM0, /dev/ttyTHS1, ...
  std::string device;

  /// Line speed. The classic bus is 115200 8N1.
  std::uint32_t baudrate = kDefaultBaudrate;

  /// poll(2) budget for one servo reply, milliseconds.
  int read_timeout_ms = 20;

  /// Budget for handing TX bytes to the kernel (not physical drain), milliseconds.
  int write_timeout_ms = 100;

  /// Ask the kernel for ASYNC_LOW_LATENCY (TIOCSSERIAL). Bypasses the 16 ms
  /// USB-serial receive timer on FTDI/CH340/CH341 BusLinker boards. Ignored
  /// when the driver rejects it (e.g. native UARTs, or non-root).
  bool enable_low_latency = true;

  /// tcflush(TCIFLUSH) before every query so stale bytes from a previous
  /// transaction, a reset servo or a hot-plugged cable cannot be mis-framed.
  bool flush_before_query = true;

  /// Drop a parsed reply that is byte-identical to the frame we just sent.
  /// Guards against USB-TTL adapters that echo TX back into RX.
  bool reject_tx_echo = true;

  /// Extra attempts for transiently-failing transactions. Keep at 0 for
  /// deterministic control loops.
  int max_retries = 0;

  /// Deprecated compatibility option; ignored. Classic replies echo the
  /// command opcode and have no firmware status byte. Correlation is mandatory.
  bool strict_status = false;

  /// TIOCEXCL: refuse to share the port with another process.
  bool exclusive = true;

  /// How hardware failures are reported for every transaction on this bus.
  ErrorMode error_mode = ErrorMode::kStrict;

  /// Size of the largest frame we will buffer (the u8 length field caps out
  /// at 255, so 255 + 3 + slack is plenty).
  std::size_t rx_buffer_size = 512;

  BusConfig() = default;
  explicit BusConfig(const std::string& dev) : device(dev) {}
};

/// Cumulative wire counters.
struct BusStats {
  std::uint64_t tx_bytes = 0;
  std::uint64_t rx_bytes = 0;
  std::uint64_t writes = 0;         ///< Transport write calls (usually one syscall)
  std::uint64_t transactions = 0;   ///< request/response round trips
  std::uint64_t timeouts = 0;
  std::uint64_t checksum_errors = 0;
  std::uint64_t io_errors = 0;
  std::uint64_t echo_drops = 0;

  void reset() noexcept { *this = BusStats{}; }
};

/// Byte pipe behind `SerialBus`.
///
/// The bus owns framing and policy; an implementation only has to move bytes.
/// `readSome()` must return as soon as anything is available rather than
/// waiting for the full request, which is what keeps latency at packet length.
class Transport {
 public:
  virtual ~Transport() = default;

  virtual bool open(std::string* error) = 0;
  virtual void close() noexcept = 0;
  virtual bool isOpen() const noexcept = 0;

  /// Write up to `count` bytes under one timeout budget. Returns count on
  /// success, a partial count on timeout, or -1 on an I/O error. Success means
  /// kernel acceptance, not physical transmission or actuator execution.
  virtual long writeBytes(const std::uint8_t* data, std::size_t count,
                          int timeout_ms, std::string* error) = 0;

  /// Non-blocking-ish read of *up to* `cap` bytes.
  /// Returns 1..cap on success, 0 when the deadline passed with nothing
  /// received, -1 on error. Must never block past `timeout_ms`.
  virtual long readSome(std::uint8_t* out, std::size_t cap, int timeout_ms,
                        std::string* error) = 0;

  virtual void flushInput() noexcept {}
  virtual void flushOutput() noexcept {}

  /// Re-apply a (possibly tweaked) `BusConfig` to the live handle.
  /// Return false when the backend cannot reconfigure in place.
  virtual bool applyConfig(const BusConfig& config, std::string* error) {
    (void)config;
    (void)error;
    return false;
  }

  /// Non-fatal advisory from the last `open()` -- e.g. "ASYNC_LOW_LATENCY was
  /// refused by this driver". Surfaced via `SerialBus::lastHint()`.
  virtual std::string lastHint() const { return std::string(); }

  /// Backend name for logs ("posix", "simulator", ...).
  virtual const char* name() const noexcept { return "unknown"; }
};

/// Open a Linux character device with cfmakeraw() at `baudrate`, 8N1.
///
/// Returns -1 on failure with an errno-derived message in `*error`.
std::unique_ptr<Transport> makePosixTransport(const BusConfig& config,
                                              std::string* error);

/// ---------------------------------------------------------------------------
/// SerialBus
/// ---------------------------------------------------------------------------
class SerialBus {
 public:
  using Clock = std::chrono::steady_clock;

  SerialBus();
  explicit SerialBus(BusConfig config);

  /// Test/simulator seam: run the same framing logic over a custom pipe.
  SerialBus(std::unique_ptr<Transport> transport, BusConfig config = BusConfig{});

  ~SerialBus();

  SerialBus(const SerialBus&) = delete;
  SerialBus& operator=(const SerialBus&) = delete;
  SerialBus(SerialBus&&) noexcept;
  SerialBus& operator=(SerialBus&&) noexcept;

  // --- Lifecycle ----------------------------------------------------------

  /// Open `device`. Throws ServoIOError on failure when the bus is in strict
  /// mode, and returns false in any mode (callers get to choose).
  bool open(const std::string& device);
  bool tryOpen(const std::string& device, std::string* error) noexcept;
  void close() noexcept;
  bool isOpen() const noexcept;

  std::string device() const;
  BusConfig config() const;

  /// Replace the whole `BusConfig`. Cheap: pair with `applyConfig()` to push
  /// line-speed / latency changes to a live descriptor.
  void setConfig(const BusConfig& config);
  const char* transportName() const noexcept;

  /// The active byte pipe. Exposed for backends and tests that need to reach
  /// their own transport -- e.g. the simulator's connect()/disconnect() hooks.
  /// Application code should not need it. Requires external synchronization
  /// against all bus operations; prefer withTransport() for live hooks.
  Transport* internalTransport() noexcept { return transport_.get(); }
  const Transport* internalTransport() const noexcept { return transport_.get(); }

  /// Execute a backend hook while holding the bus lock. The callback must not
  /// re-enter the bus. Used for synchronized simulator connect/disconnect.
  void withTransport(const std::function<void(Transport&)>& callback);

  /// Advisory note from the last open()/applyConfig() (e.g. the low-latency
  /// ioctl was refused). Empty when the port came up exactly as configured.
  std::string lastHint() const noexcept;

  /// Re-apply the current `BusConfig` to the live file descriptor.
  bool applyConfig() noexcept;

  // --- Policy -------------------------------------------------------------

  void setReadTimeoutMs(int ms) noexcept;
  int readTimeoutMs() const noexcept;
  void setWriteTimeoutMs(int ms) noexcept;
  int writeTimeoutMs() const noexcept;

  void setErrorMode(ErrorMode mode) noexcept;
  ErrorMode errorMode() const noexcept;

  /// Convenience: `false` == non-throwing real-time mode.
  void setPollHardware(bool poll_hardware) noexcept {
    setErrorMode(poll_hardware ? ErrorMode::kStrict : ErrorMode::kNonThrowing);
  }

  // --- Last error / counters ---------------------------------------------

  std::string lastError() const;
  ErrorCode lastErrorCode() const noexcept;
  void clearLastError() noexcept;
  BusStats stats() const noexcept;
  void resetStats() noexcept;

  // --- Raw byte movement (thread-safe) -----------------------------------

  /// One write(2) carrying every packet, checksummed individually.
  /// Returns bytes written (== total payload) or 0 on failure.
  std::size_t tryWriteFrames(const std::vector<Packet>& frames,
                             ErrorCode* error = nullptr) noexcept;
  std::size_t writeFrames(const std::vector<Packet>& frames);

  /// Single frame, one syscall.
  bool tryWrite(const Packet& frame, ErrorCode* error = nullptr) noexcept;
  void write(const Packet& frame);

  bool tryFlushInput(ErrorCode* error = nullptr) noexcept;
  void flushInput();
  bool tryFlushOutput(ErrorCode* error = nullptr) noexcept;
  void flushOutput();

  // --- Transactions ------------------------------------------------------

  /// One request/response round trip. Never throws; inspects `*error`.
  ///
  /// Only checksum-valid replies matching the request ID and echoed command
  /// are accepted. Unrelated frames are skipped within the original deadline.
  /// `expected_payload` (when non-zero) is the required data size; a matching
  /// reply of the wrong shape reports ErrorCode::kShortRead.
  bool tryTransact(const Packet& request, Response* response,
                   std::size_t expected_payload = 0, int timeout_ms = -1,
                   ErrorCode* error = nullptr) noexcept;

  /// `tryTransact` + throw the matching exception in strict mode.
  Response transact(const Packet& request, std::size_t expected_payload = 0,
                    int timeout_ms = -1);

  /// Fire-and-forget write with the same error handling as a transaction.
  bool tryCommand(const Packet& frame, ErrorCode* error = nullptr) noexcept;
  void command(const Packet& frame);

  // --- Batch / broadcast primitives --------------------------------------

  /// Queue synchronized targets for many servos at once.
  ///
  /// Builds one `SERVO_MOVE_TIME_WAIT_WRITE` (cmd 7) frame per servo and,
  /// when `broadcast_start` is true, appends the `0xFE` `SERVO_MOVE_START`
  /// (cmd 11) frame. All of it leaves in a single write(2).
  ///
  /// `ticks` must be pre-converted tick counts (see Servo::angleToTicks).
  /// Returns total bytes written, or 0 on failure.
  std::size_t queueMoves(const std::vector<std::uint8_t>& ids,
                         const std::vector<std::uint16_t>& ticks,
                         std::uint16_t time_ms, bool broadcast_start = true,
                         ErrorCode* error = nullptr) noexcept;

  std::size_t queueMoves(const std::vector<std::uint8_t>& ids,
                         const std::vector<std::uint16_t>& ticks,
                         std::uint16_t time_ms, bool broadcast_start,
                         int timeout_ms, ErrorCode* error) noexcept;

  /// Broadcast `SERVO_MOVE_START` to every servo on the bus.
  bool broadcastMoveStart(ErrorCode* error = nullptr) noexcept;
  void moveStart();

  /// Broadcast `SERVO_MOVE_STOP`.
  bool broadcastMoveStop(ErrorCode* error = nullptr) noexcept;
  void moveStop();

  /// Enable/disable torque on many servos in a single write(2).
  std::size_t setTorqueBatch(const std::vector<std::uint8_t>& ids, bool enable,
                             ErrorCode* error = nullptr) noexcept;

  /// Read the same opcode from many servos. Each reply is its own round trip
  /// (the bus is half duplex, so concurrent replies would collide), but the
  /// whole sweep still beats hand-rolled per-servo Python.
  bool tryReadMany(const std::vector<std::uint8_t>& ids, Command cmd,
                   std::size_t expected_payload, std::vector<Response>* out,
                   ErrorCode* error = nullptr) noexcept;

  std::vector<Response> readMany(const std::vector<std::uint8_t>& ids, Command cmd,
                                 std::size_t expected_payload);

  /// Walk an id range and collect every servo that answers.
  ///
  /// Uses a short read timeout so a 253-id sweep still finishes quickly;
  /// `first`/`last` bound the search (default: the whole unicast space).
  std::vector<std::uint8_t> scanIds(std::uint8_t first = kMinServoId,
                                    std::uint8_t last = kMaxServoId,
                                    Command probe = Command::kReadPhysicalAngle,
                                    int timeout_ms = 15);
  bool tryScanIds(std::uint8_t first, std::uint8_t last, Command probe,
                  int timeout_ms, std::vector<std::uint8_t>* out,
                  ErrorCode* error = nullptr) noexcept;

  /// Helper: concatenate frames into one buffer (exposed for tests/benchmarks).
  static std::vector<std::uint8_t> concatenate(const std::vector<Packet>& frames);

 private:
  friend class Servo;

  bool ensureOpen(ErrorCode* error) noexcept;

  /// Append bytes from the transport until a whole frame is parsed.
  /// Never locks; caller holds mutex_.
  bool readFrameLocked(Response* out, std::size_t expected_payload,
                       int timeout_ms, ErrorCode* error) noexcept;

  /// Consume already-buffered bytes into `out`. Returns false when more bytes
  /// are needed; on true, `*error` says whether the frame was usable.
  bool consumeFrameLocked(Response* out, std::size_t expected_payload,
                          ErrorCode* error) noexcept;

  /// True when a parsed reply is byte-identical to the frame we just sent.
  bool matchesTxEchoLocked(const Response& candidate) const noexcept;

  void discardBufferedLocked() noexcept;
  void rememberRecord(ErrorCode code, const std::string& message) noexcept;
  void recordOutcomeLocked(ErrorCode code, const std::string& message = {}) noexcept;
  void recordStatsLocked(ErrorCode code) noexcept;

  BusConfig config_;
  std::unique_ptr<Transport> transport_;
  std::vector<std::uint8_t> rx_buffer_;
  std::vector<std::uint8_t> last_tx_;  ///< for echo rejection
  mutable std::mutex mutex_;
  std::string last_error_;
  ErrorCode last_error_code_ = ErrorCode::kOk;
  BusStats stats_;
  bool owned_transport_ = true;
};

}  // namespace HiwonderBus

#endif  // HIWONDERBUS_SERIAL_BUS_HPP
