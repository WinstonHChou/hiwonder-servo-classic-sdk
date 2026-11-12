// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// src/serial_bus.cpp
//   Native Linux POSIX transport: termios raw 8N1, ASYNC_LOW_LATENCY, poll(2)
//   driven reads, single-syscall batch writes and robust frame resync.
//
#include "HiwonderBus/serial_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include "HiwonderBus/HiwonderBus.hpp"

namespace HiwonderBus {
namespace {

/// Bytes we ask the kernel for per poll() wakeup. One reply is 6..10 bytes,
/// so a single read(2) normally captures the whole packet.
constexpr std::size_t kReadChunk = 128;

/// Smallest usable inner `length` field: id + length + command.
constexpr std::size_t kMinInnerLength = 3;

using BusClock = std::chrono::steady_clock;

long msUntil(BusClock::time_point deadline) noexcept {
  using namespace std::chrono;
  const auto left = ceil<milliseconds>(deadline - BusClock::now()).count();
  return left < 0 ? 0 : static_cast<long>(left);
}

int clampToInt(long long v) noexcept {
  if (v < std::numeric_limits<int>::min()) return std::numeric_limits<int>::min();
  if (v > std::numeric_limits<int>::max()) return std::numeric_limits<int>::max();
  return static_cast<int>(v);
}

BusClock::time_point deadlineAfter(int timeout_ms) noexcept {
  return BusClock::now() + std::chrono::milliseconds(timeout_ms < 0 ? 0 : timeout_ms);
}

/// Translate a numeric baud rate into the matching termios constant.
/// Returns false (and leaves `*out` untouched) for rates this platform lacks.
bool toTermiosSpeed(std::uint32_t baudrate, speed_t* out) noexcept {
  switch (baudrate) {
    case 9600: *out = B9600; return true;
    case 19200: *out = B19200; return true;
    case 38400: *out = B38400; return true;
    case 57600: *out = B57600; return true;
    case 115200: *out = B115200; return true;
    case 230400: *out = B230400; return true;
    case 460800: *out = B460800; return true;
    case 921600: *out = B921600; return true;
#if defined(B1000000)
    case 1000000: *out = B1000000; return true;
#endif
#if defined(B1500000)
    case 1500000: *out = B1500000; return true;
#endif
#if defined(B2000000)
    case 2000000: *out = B2000000; return true;
#endif
#if defined(B2500000)
    case 2500000: *out = B2500000; return true;
#endif
#if defined(B3000000)
    case 3000000: *out = B3000000; return true;
#endif
#if defined(B3500000)
    case 3500000: *out = B3500000; return true;
#endif
#if defined(B4000000)
    case 4000000: *out = B4000000; return true;
#endif
    default: return false;
  }
}

const char* strerrorName(int err) noexcept {
  const char* s = std::strerror(err);
  return s ? s : "unknown errno";
}

/// Ask the kernel to skip the 16 ms receive timer USB-serial bridges apply by
/// default. Purely an optimisation: a refusal is reported as a hint, not fatal.
bool setLowLatencyFlag(int fd, bool enable, std::string* hint) {
#if defined(__linux__) && defined(ASYNC_LOW_LATENCY)
  struct serial_struct ss;
  std::memset(&ss, 0, sizeof(ss));
  if (::ioctl(fd, TIOCGSERIAL, &ss) != 0) {
    if (hint) {
      *hint = std::string("TIOCGSERIAL unsupported by this driver: ") +
              strerrorName(errno) + " (leaving the 16ms USB latency timer in place)";
    }
    return false;
  }
  const bool is_on = (ss.flags & ASYNC_LOW_LATENCY) != 0;
  if (is_on == enable) return true;

  if (enable) {
    ss.flags |= static_cast<int>(ASYNC_LOW_LATENCY);
  } else {
    ss.flags &= ~static_cast<int>(ASYNC_LOW_LATENCY);
  }
  if (::ioctl(fd, TIOCSSERIAL, &ss) != 0) {
    if (hint) {
      *hint = std::string("TIOCSSERIAL(ASYNC_LOW_LATENCY) refused: ") +
              strerrorName(errno) +
              " (some drivers/kernels require CAP_SYS_ADMIN)";
    }
    return false;
  }
  return true;
#else
  (void)fd;
  (void)enable;
  if (hint) *hint = "ASYNC_LOW_LATENCY is not available on this platform";
  return false;
#endif
}

/// poll(2) on a single fd until `deadline`, retrying on EINTR.
/// Returns >0 when ready, 0 when the deadline passed, -1 on error.
int pollUntil(int fd, short events, BusClock::time_point deadline, std::string* error) {
  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = static_cast<short>(events);
  for (;;) {
    const long left = msUntil(deadline);
    if (left <= 0) return 0;
    pfd.revents = 0;
    const int rc = ::poll(&pfd, 1, clampToInt(left));
    if (rc > 0) {
      if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        if (error) *error = "serial device signalled an error condition";
        return -1;
      }
      return rc;
    }
    if (rc == 0) return 0;  // kernel-side timeout
    if (errno == EINTR) continue;  // signals must not eat the budget
    if (error) *error = std::string("poll(2) failed: ") + strerrorName(errno);
    return -1;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// PosixTransport
// ---------------------------------------------------------------------------
class PosixTransport final : public Transport {
 public:
  explicit PosixTransport(BusConfig config) : config_(std::move(config)) {}

  ~PosixTransport() override { close(); }

  bool open(std::string* error) override {
    if (fd_ >= 0) return true;
    if (config_.device.empty()) {
      if (error) *error = "no serial device configured";
      return false;
    }

    int flags = O_RDWR | O_NOCTTY | O_CLOEXEC;
#if defined(O_NONBLOCK)
    // Never block in open() waiting for carrier detection.
    flags |= O_NONBLOCK;
#endif
    const int fd = ::open(config_.device.c_str(), flags);
    if (fd < 0) {
      const int err = errno;
      if (error) {
        switch (err) {
          case EBUSY:
          case EAGAIN:
            *error = config_.device + " is busy (another process owns the port)";
            break;
          case ENOENT:
            *error = config_.device + " does not exist";
            break;
          case ENOTTY:
            *error = config_.device + " is not a serial device";
            break;
          case EACCES:
          case EPERM:
            *error = "permission denied on " + config_.device +
                     " (add this user to the dialout/uucp group)";
            break;
          default:
            *error = "could not open " + config_.device + ": " + strerrorName(err);
            break;
        }
      }
      return false;
    }
    fd_ = fd;

    // Keep O_NONBLOCK: poll readiness never guarantees that a large write
    // will complete. Partial writes/EAGAIN are retried under one deadline.

    if (!applyTermios(error)) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    if (config_.exclusive && ::ioctl(fd_, TIOCEXCL) != 0) {
      // Another process holding the port would corrupt every transaction, but
      // refusing the lock is not fatal -- carry on and let the transaction
      // report what happens.
      hint_ = "TIOCEXCL refused: " + std::string(strerrorName(errno));
    }

    if (config_.enable_low_latency) {
      std::string latency_hint;
      if (!setLowLatencyFlag(fd_, true, &latency_hint)) hint_ = latency_hint;
    }

    ::tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close() noexcept override {
    if (fd_ >= 0) {
      ::tcflush(fd_, TCIOFLUSH);
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool isOpen() const noexcept override { return fd_ >= 0; }

  long writeBytes(const std::uint8_t* data, std::size_t count, int timeout_ms,
                  std::string* error) override {
    if (fd_ < 0) {
      if (error) *error = "port is not open";
      return -1;
    }
    const BusClock::time_point deadline = deadlineAfter(timeout_ms);
    std::size_t written = 0;
    while (written < count) {
      const int ready = pollUntil(fd_, POLLOUT, deadline, error);
      if (ready < 0) return -1;
      if (ready == 0) {
        if (error) {
          *error = "write timeout after " + std::to_string(written) + " of " +
                   std::to_string(count) + " bytes";
        }
        return static_cast<long>(written);
      }
      const ssize_t n = ::write(fd_, data + written, count - written);
      if (n > 0) {
        written += static_cast<std::size_t>(n);
        continue;
      }
      if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
        continue;  // let poll(2) re-arm
      }
      if (n == 0) {
        if (error) *error = "write(2) returned 0";
        return -1;
      }
      if (error) *error = std::string("write(2) failed: ") + strerrorName(errno);
      return -1;
    }
    return static_cast<long>(written);
  }

  long readSome(std::uint8_t* out, std::size_t cap, int timeout_ms,
                std::string* error) override {
    if (fd_ < 0) {
      if (error) *error = "port is not open";
      return -1;
    }
    if (cap == 0) return 0;

    const auto deadline = deadlineAfter(timeout_ms);
    for (;;) {
      const int ready = pollUntil(fd_, POLLIN, deadline, error);
      if (ready <= 0) return ready;
      const ssize_t n = ::read(fd_, out, cap);
      if (n > 0) return static_cast<long>(n);
      if (n == 0 || errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;  // readiness raced; retain the original deadline
      }
      if (error) *error = std::string("read(2) failed: ") + strerrorName(errno);
      return -1;
    }
  }

  void flushInput() noexcept override {
    if (fd_ >= 0) ::tcflush(fd_, TCIFLUSH);
  }

  void flushOutput() noexcept override {
    if (fd_ >= 0) ::tcflush(fd_, TCOFLUSH);
  }

  const char* name() const noexcept override { return "posix"; }

  std::string lastHint() const noexcept override { return hint_; }

  bool applyConfig(const BusConfig& config, std::string* error) override {
    if (fd_ < 0) return false;
    config_ = config;
    std::string latency_hint;
    const bool ok = applyTermios(error);
    if (config.enable_low_latency && !setLowLatencyFlag(fd_, true, &latency_hint)) {
      hint_ = latency_hint;
    }
    return ok;
  }

  /// The current file descriptor, exposed for diagnostics.
  int fd() const noexcept { return fd_; }

 private:
  bool applyTermios(std::string* error) {
    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (::tcgetattr(fd_, &tio) != 0) {
      if (error) {
        *error = std::string("tcgetattr(3) failed: ") + strerrorName(errno);
      }
      return false;
    }

    // Raw mode: no echo, no signal interpretation, no canonical buffering,
    // no CR/NL translation -- exactly what a binary bus protocol needs.
    ::cfmakeraw(&tio);

    tio.c_cflag = static_cast<tcflag_t>((tio.c_cflag | CLOCAL | CREAD) &
                                        ~static_cast<tcflag_t>(CSTOPB | PARENB));
#if defined(CRTSCTS)
    tio.c_cflag = static_cast<tcflag_t>(tio.c_cflag & ~static_cast<tcflag_t>(CRTSCTS));
#endif
    // 8 data bits, 1 stop bit, no parity.
    tio.c_cflag = static_cast<tcflag_t>(tio.c_cflag & ~static_cast<tcflag_t>(CSIZE)) | CS8;
    tio.c_iflag = static_cast<tcflag_t>(
        tio.c_iflag & ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR));
    tio.c_lflag = 0;
    tio.c_oflag = 0;

    // VMIN = VTIME = 0 so read(2) returns immediately with whatever is there;
    // the actual sleeping is done by poll(2) under our own deadline.
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    speed_t speed = B115200;
    if (!toTermiosSpeed(config_.baudrate, &speed)) {
      if (error) *error = "unsupported baud rate " + std::to_string(config_.baudrate);
      return false;
    }
    if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) {
      if (error) *error = "cfsetspeed(3) failed";
      return false;
    }
    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
      if (error) {
        *error = std::string("tcsetattr(3) failed: ") + strerrorName(errno);
      }
      return false;
    }
    return true;
  }

  BusConfig config_;
  int fd_ = -1;
  std::string hint_;
};

std::unique_ptr<Transport> makePosixTransport(const BusConfig& config,
                                              std::string* error) {
  if (config.device.empty()) {
    if (error) *error = "BusConfig::device is empty";
    return nullptr;
  }
  return std::unique_ptr<Transport>(new PosixTransport(config));
}

// ---------------------------------------------------------------------------
// SerialBus -- construction
// ---------------------------------------------------------------------------

SerialBus::SerialBus() = default;

SerialBus::SerialBus(BusConfig config) : config_(std::move(config)) {}

SerialBus::SerialBus(std::unique_ptr<Transport> transport, BusConfig config)
    : config_(std::move(config)),
      transport_(std::move(transport)),
      owned_transport_(false) {}

SerialBus::~SerialBus() { close(); }

SerialBus::SerialBus(SerialBus&& other) noexcept {
  std::lock_guard<std::mutex> self(mutex_);
  std::lock_guard<std::mutex> other_guard(other.mutex_);
  config_ = std::move(other.config_);
  transport_ = std::move(other.transport_);
  rx_buffer_ = std::move(other.rx_buffer_);
  last_tx_ = std::move(other.last_tx_);
  last_error_ = std::move(other.last_error_);
  last_error_code_ = other.last_error_code_;
  stats_ = other.stats_;
  owned_transport_ = other.owned_transport_;
  other.owned_transport_ = true;
}

SerialBus& SerialBus::operator=(SerialBus&& other) noexcept {
  if (this == &other) return *this;
  std::scoped_lock lock(mutex_, other.mutex_);
  config_ = std::move(other.config_);
  transport_ = std::move(other.transport_);
  rx_buffer_ = std::move(other.rx_buffer_);
  last_tx_ = std::move(other.last_tx_);
  last_error_ = std::move(other.last_error_);
  last_error_code_ = other.last_error_code_;
  stats_ = other.stats_;
  owned_transport_ = other.owned_transport_;
  other.owned_transport_ = true;
  return *this;
}

// --- Lifecycle -------------------------------------------------------------

namespace {
/// Decide whether `open()` may reuse `transport_` or must build a POSIX one.
bool needsPosixTransport(const std::unique_ptr<Transport>& transport,
                         bool owned) noexcept {
  return !transport || owned;
}
}  // namespace

bool SerialBus::open(const std::string& device) {
  ErrorCode code = ErrorCode::kOk;
  std::string message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.device = device;
    if (needsPosixTransport(transport_, owned_transport_)) {
      std::string create_error;
      transport_ = makePosixTransport(config_, &create_error);
      if (!transport_) {
        code = ErrorCode::kInvalidArgument;
        message = create_error;
      } else {
        owned_transport_ = true;
      }
    }
    if (transport_) {
      std::string open_error;
      if (!transport_->open(&open_error)) {
        code = ErrorCode::kIoError;
        message = open_error.empty() ? "could not open " + device : open_error;
      } else {
        rx_buffer_.clear();
        last_tx_.clear();
        stats_.reset();
        last_error_code_ = ErrorCode::kOk;
        last_error_.clear();
      }
    } else if (code == ErrorCode::kOk) {
      code = ErrorCode::kIoError;
      message = "no transport available for " + device;
    }
  }

  if (code != ErrorCode::kOk) {
    rememberRecord(code, message);
    if (errorMode() == ErrorMode::kStrict) {
      throw ServoIOError(ServoError::formatMessage(
                             "could not open serial device: " + message, kNoServoId),
                         kNoServoId);
    }
    return false;
  }
  return true;
}

bool SerialBus::tryOpen(const std::string& device, std::string* error) noexcept {
  // Same body as open() but never throws, for control loops and for Python.
  ErrorCode code = ErrorCode::kOk;
  std::string message;
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.device = device;
    if (needsPosixTransport(transport_, owned_transport_)) {
      std::string create_error;
      transport_ = makePosixTransport(config_, &create_error);
      if (!transport_) {
        code = ErrorCode::kInvalidArgument;
        message = create_error;
      } else {
        owned_transport_ = true;
      }
    }
    if (transport_) {
      std::string open_error;
      if (!transport_->open(&open_error)) {
        code = ErrorCode::kIoError;
        message = open_error.empty() ? "could not open " + device : open_error;
      } else {
        rx_buffer_.clear();
        last_tx_.clear();
        stats_.reset();
        last_error_code_ = ErrorCode::kOk;
        last_error_.clear();
      }
    }
  } catch (...) {
    code = ErrorCode::kIoError;
    message = "unexpected exception while opening the port";
  }

  if (code != ErrorCode::kOk) {
    rememberRecord(code, message);
    if (error) *error = message;
    return false;
  }
  if (error) error->clear();
  return true;
}

void SerialBus::close() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (transport_) transport_->close();
  rx_buffer_.clear();
  last_tx_.clear();
}

bool SerialBus::isOpen() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return transport_ && transport_->isOpen();
}

const char* SerialBus::transportName() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return transport_ ? transport_->name() : "none";
}

std::string SerialBus::lastHint() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    return transport_ ? transport_->lastHint() : std::string();
  } catch (...) {
    return std::string();
  }
}

bool SerialBus::applyConfig() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ensureOpen(nullptr)) return false;
  std::string error;
  if (!transport_->applyConfig(config_, &error)) {
    recordOutcomeLocked(ErrorCode::kIoError, error);
    return false;
  }
  recordOutcomeLocked(ErrorCode::kOk);
  return true;
}

// --- Policy ----------------------------------------------------------------

std::string SerialBus::device() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.device;
}

BusConfig SerialBus::config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void SerialBus::setConfig(const BusConfig& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  config_.read_timeout_ms = std::max(0, config_.read_timeout_ms);
  config_.write_timeout_ms = std::max(0, config_.write_timeout_ms);
  config_.max_retries = std::max(0, config_.max_retries);
}

void SerialBus::withTransport(const std::function<void(Transport&)>& callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!transport_) throw ServoLogicalError("no active transport", kNoServoId);
  callback(*transport_);
}

int SerialBus::readTimeoutMs() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.read_timeout_ms;
}

int SerialBus::writeTimeoutMs() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.write_timeout_ms;
}

void SerialBus::setErrorMode(ErrorMode mode) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  config_.error_mode = mode;
}

ErrorMode SerialBus::errorMode() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.error_mode;
}

std::string SerialBus::lastError() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

ErrorCode SerialBus::lastErrorCode() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_code_;
}

void SerialBus::setReadTimeoutMs(int ms) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  config_.read_timeout_ms = ms < 0 ? 0 : ms;
}

void SerialBus::setWriteTimeoutMs(int ms) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  config_.write_timeout_ms = ms < 0 ? 0 : ms;
}

void SerialBus::clearLastError() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  last_error_.clear();
  last_error_code_ = ErrorCode::kOk;
}

BusStats SerialBus::stats() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void SerialBus::resetStats() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.reset();
}

// --- Internals (all called with mutex_ held) -------------------------------

void SerialBus::rememberRecord(ErrorCode code, const std::string& message) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  recordOutcomeLocked(code, message);
}

void SerialBus::recordOutcomeLocked(ErrorCode code, const std::string& message) noexcept {
  last_error_code_ = code;
  last_error_ = code == ErrorCode::kOk ? std::string() :
                (message.empty() ? std::string(toString(code)) : message);
  recordStatsLocked(code);
}

void SerialBus::recordStatsLocked(ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::kTimeout:
    case ErrorCode::kShortRead:
      stats_.timeouts += 1;
      break;
    case ErrorCode::kBadChecksum:
    case ErrorCode::kBadFrame:
      stats_.checksum_errors += 1;
      break;
    case ErrorCode::kIoError:
    case ErrorCode::kPortBusy:
    case ErrorCode::kNotOpen:
      stats_.io_errors += 1;
      break;
    default:
      break;
  }
}

bool SerialBus::ensureOpen(ErrorCode* error) noexcept {
  if (transport_ && transport_->isOpen()) return true;
  if (error) *error = ErrorCode::kNotOpen;
  last_error_code_ = ErrorCode::kNotOpen;
  last_error_ = "port " + config_.device + " is not open";
  recordStatsLocked(ErrorCode::kNotOpen);
  return false;
}

bool SerialBus::matchesTxEchoLocked(const Response& candidate) const noexcept {
  if (!config_.reject_tx_echo) return false;
  // A well-formed echo frame is [55 55 id len cmd args... cksum].
  if (last_tx_.size() < kFrameHeaderSize + 3) return false;
  const std::uint8_t tx_length = last_tx_[3];
  if (tx_length < kMinInnerLength) return false;

  if (candidate.id != last_tx_[2]) return false;
  if (candidate.length != tx_length) return false;
  if (candidate.command != last_tx_[4]) return false;  // our own opcode

  const std::size_t arg_count = static_cast<std::size_t>(tx_length) - kMinInnerLength;
  if (candidate.data.size() != arg_count) return false;
  if (last_tx_.size() < kFrameHeaderSize + 3 + arg_count) return false;
  for (std::size_t i = 0; i < arg_count; ++i) {
    if (candidate.data[i] != last_tx_[kFrameHeaderSize + 3 + i]) return false;
  }
  return true;
}

bool SerialBus::consumeFrameLocked(Response* out, std::size_t expected_payload,
                                   ErrorCode* error) noexcept {
  for (;;) {
    if (rx_buffer_.empty()) {
      if (error) *error = ErrorCode::kOk;  // "need more bytes"
      return false;
    }

    std::size_t consumed = 0;
    Response parsed;
    const FrameResult fr =
        tryParseFrame(rx_buffer_.data(), rx_buffer_.size(), &consumed, &parsed);
    const std::ptrdiff_t how_many = static_cast<std::ptrdiff_t>(consumed);

    if (fr == FrameResult::kComplete) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + how_many);
      // A bridge echoing our own TX back is the only realistic way to see a
      // "reply" whose payload equals our request's args: drop it and keep
      // searching instead of mis-reading our own command as servo data.
      if (matchesTxEchoLocked(parsed)) {
        stats_.echo_drops += 1;
        parsed = Response{};
        continue;
      }
      // Discard unrelated (possibly delayed) frames before checking shape.
      if (last_tx_.size() >= 5 &&
          (parsed.id != last_tx_[2] || parsed.command != last_tx_[4])) {
        continue;
      }
      if (expected_payload != 0 && parsed.data.size() != expected_payload) {
        if (error) *error = ErrorCode::kShortRead;
        return true;
      }
      if (out) *out = std::move(parsed);
      if (error) *error = ErrorCode::kOk;
      return true;
    }

    if (fr == FrameResult::kBadChecksum) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + how_many);
      if (error) *error = ErrorCode::kBadChecksum;
      return true;
    }

    if (fr == FrameResult::kBadHeader) {
      if (consumed > 0) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + how_many);
        continue;  // resynced: re-run over whatever is left
      }
      if (error) *error = ErrorCode::kBadFrame;
      return true;
    }

    // FrameResult::kIncomplete -- possibly after skipping leading garbage.
    if (consumed > 0) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + how_many);
      continue;
    }
    if (error) *error = ErrorCode::kOk;
    return false;
  }
}

void SerialBus::discardBufferedLocked() noexcept { rx_buffer_.clear(); }

bool SerialBus::readFrameLocked(Response* out, std::size_t expected_payload,
                                int timeout_ms, ErrorCode* error) noexcept {
  if (timeout_ms < 0) timeout_ms = config_.read_timeout_ms;
  const BusClock::time_point deadline = deadlineAfter(timeout_ms);

  for (;;) {
    ErrorCode local = ErrorCode::kOk;
    if (consumeFrameLocked(out, expected_payload, &local)) {
      if (error) *error = local;
      return local == ErrorCode::kOk;
    }

    const long left = msUntil(deadline);
    if (left <= 0) {
      if (error) *error = ErrorCode::kTimeout;
      return false;
    }

    std::string terr;
    const std::size_t before = rx_buffer_.size();
    rx_buffer_.resize(before + kReadChunk);
    const long got = transport_->readSome(rx_buffer_.data() + before, kReadChunk,
                                          clampToInt(left), &terr);
    if (got < 0) {
      rx_buffer_.resize(before);
      if (error) *error = ErrorCode::kIoError;
      last_error_ = terr;
      return false;
    }
    if (got == 0) {
      rx_buffer_.resize(before);
      if (error) *error = ErrorCode::kTimeout;
      return false;
    }
    rx_buffer_.resize(before + static_cast<std::size_t>(got));
    stats_.rx_bytes += static_cast<std::uint64_t>(got);

    if (rx_buffer_.size() > config_.rx_buffer_size) {
      // Somebody is shouting at the bus. Stop buffering, hand the problem back.
      rx_buffer_.clear();
      if (error) *error = ErrorCode::kBadFrame;
      return false;
    }
  }
}

// --- Raw byte movement -----------------------------------------------------

std::vector<std::uint8_t> SerialBus::concatenate(const std::vector<Packet>& frames) {
  std::size_t total = 0;
  for (const Packet& f : frames) total += f.size();
  std::vector<std::uint8_t> buffer(total);
  std::size_t offset = 0;
  for (const Packet& f : frames) {
    if (!f.empty()) std::copy(f.begin(), f.end(), buffer.begin() + static_cast<std::ptrdiff_t>(offset));
    offset += f.size();
  }
  return buffer;
}

std::size_t SerialBus::tryWriteFrames(const std::vector<Packet>& frames,
                                      ErrorCode* error) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frames.empty() || std::any_of(frames.begin(), frames.end(),
                                   [](const Packet& frame) { return frame.empty(); })) {
    if (error) *error = ErrorCode::kInvalidArgument;
    recordOutcomeLocked(ErrorCode::kInvalidArgument, "no valid frames to write");
    return 0;
  }
  if (!ensureOpen(error)) return 0;

  std::vector<std::uint8_t> buffer = concatenate(frames);
  // Remember the first frame: that is what a bridge would echo.
  last_tx_.assign(frames.front().begin(), frames.front().end());

  std::string terr;
  const long written =
      transport_->writeBytes(buffer.data(), buffer.size(), config_.write_timeout_ms, &terr);
  stats_.writes += 1;
  if (written > 0) stats_.tx_bytes += static_cast<std::uint64_t>(written);
  if (written < 0 || static_cast<std::size_t>(written) != buffer.size()) {
    const ErrorCode code = written < 0 ? ErrorCode::kIoError : ErrorCode::kTimeout;
    if (error) *error = code;
    recordOutcomeLocked(code, terr.empty() ? "incomplete write" : terr);
    return 0;
  }
  recordOutcomeLocked(ErrorCode::kOk);
  if (error) *error = ErrorCode::kOk;
  return static_cast<std::size_t>(written);
}

std::size_t SerialBus::writeFrames(const std::vector<Packet>& frames) {
  ErrorCode code = ErrorCode::kOk;
  const std::size_t written = tryWriteFrames(frames, &code);
  if (code != ErrorCode::kOk &&
      (errorMode() == ErrorMode::kStrict || code == ErrorCode::kInvalidArgument)) {
    detail::throwError(code, lastError(), kNoServoId);
  }
  return written;
}

bool SerialBus::tryWrite(const Packet& frame, ErrorCode* error) noexcept {
  const std::vector<Packet> one{frame};
  return tryWriteFrames(one, error) == frame.size();
}

void SerialBus::write(const Packet& frame) {
  const std::vector<Packet> one{frame};
  writeFrames(one);
}

bool SerialBus::tryFlushInput(ErrorCode* error) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ensureOpen(error)) return false;
  transport_->flushInput();
  discardBufferedLocked();
  recordOutcomeLocked(ErrorCode::kOk);
  if (error) *error = ErrorCode::kOk;
  return true;
}

void SerialBus::flushInput() {
  ErrorCode code = ErrorCode::kOk;
  if (!tryFlushInput(&code) && errorMode() == ErrorMode::kStrict) {
    detail::throwError(code, lastError(), kNoServoId);
  }
}

bool SerialBus::tryFlushOutput(ErrorCode* error) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ensureOpen(error)) return false;
  transport_->flushOutput();
  recordOutcomeLocked(ErrorCode::kOk);
  if (error) *error = ErrorCode::kOk;
  return true;
}

void SerialBus::flushOutput() {
  ErrorCode code = ErrorCode::kOk;
  if (!tryFlushOutput(&code) && errorMode() == ErrorMode::kStrict) {
    detail::throwError(code, lastError(), kNoServoId);
  }
}

// --- Transactions ----------------------------------------------------------

bool SerialBus::tryTransact(const Packet& request, Response* response,
                            std::size_t expected_payload, int timeout_ms,
                            ErrorCode* error) noexcept {
  // Hold the lock over policy snapshots, flush, write and response parsing.
  std::lock_guard<std::mutex> lock(mutex_);
  if (response) *response = Response{};
  if (request.size() < 6 || request[2] > kMaxServoId) {
    if (error) *error = ErrorCode::kInvalidArgument;
    recordOutcomeLocked(ErrorCode::kInvalidArgument, "queries require a unicast frame");
    return false;
  }
  if (timeout_ms < 0) timeout_ms = config_.read_timeout_ms;
  const long long attempts = std::max(1LL, static_cast<long long>(config_.max_retries) + 1);

  ErrorCode code = ErrorCode::kOk;
  for (long long attempt = 0; attempt < attempts; ++attempt) {
    if (!ensureOpen(&code)) {
      if (error) *error = code;
      return false;
    }

    if (config_.flush_before_query) {
      transport_->flushInput();
      discardBufferedLocked();
    }

    last_tx_.assign(request.begin(), request.end());

    std::string terr;
    const long written = transport_->writeBytes(request.data(), request.size(),
                                                config_.write_timeout_ms, &terr);
    stats_.writes += 1;
    if (written > 0) stats_.tx_bytes += static_cast<std::uint64_t>(written);
    if (written < 0 || static_cast<std::size_t>(written) != request.size()) {
      code = written < 0 ? ErrorCode::kIoError : ErrorCode::kTimeout;
      recordOutcomeLocked(code, terr.empty() ? "incomplete request write" : terr);
      if (error) *error = code;
      return false;  // never retry a partially transmitted command
    }
    stats_.transactions += 1;

    Response parsed;
    code = ErrorCode::kOk;
    last_error_.clear();
    const bool ok = readFrameLocked(&parsed, expected_payload, timeout_ms, &code);
    if (ok) {
      recordOutcomeLocked(ErrorCode::kOk);
      if (error) *error = ErrorCode::kOk;
      if (response) *response = std::move(parsed);
      return true;
    }

    discardBufferedLocked();
    recordOutcomeLocked(code, last_error_);
    if (!isTransient(code) || attempt + 1 >= attempts) {
      if (error) *error = code;
      return false;
    }
  }
  if (error) *error = ErrorCode::kTimeout;
  return false;
}

Response SerialBus::transact(const Packet& request, std::size_t expected_payload,
                             int timeout_ms) {
  Response response;
  ErrorCode code = ErrorCode::kOk;
  if (tryTransact(request, &response, expected_payload, timeout_ms, &code)) {
    return response;
  }
  if (errorMode() == ErrorMode::kStrict || code == ErrorCode::kInvalidArgument) {
    const int servo_id = request.size() >= 3 ? request[2] : kNoServoId;
    detail::throwError(code, lastError(), servo_id);
  }
  return response;
}

bool SerialBus::tryCommand(const Packet& frame, ErrorCode* error) noexcept {
  return tryWrite(frame, error);
}

void SerialBus::command(const Packet& frame) { write(frame); }

// --- Batch / broadcast primitives -----------------------------------------

std::size_t SerialBus::queueMoves(const std::vector<std::uint8_t>& ids,
                                  const std::vector<std::uint16_t>& ticks,
                                  std::uint16_t time_ms, bool broadcast_start,
                                  ErrorCode* error) noexcept {
  return queueMoves(ids, ticks, time_ms, broadcast_start, -1, error);
}

std::size_t SerialBus::queueMoves(const std::vector<std::uint8_t>& ids,
                                  const std::vector<std::uint16_t>& ticks,
                                  std::uint16_t time_ms, bool broadcast_start,
                                  int timeout_ms, ErrorCode* error) noexcept {
  (void)timeout_ms;  // write budget stays config-driven for determinism
  if (ids.empty() || ids.size() != ticks.size()) {
    if (error) *error = ErrorCode::kInvalidArgument;
    rememberRecord(ErrorCode::kInvalidArgument, "IDs and ticks must be nonempty and equal length");
    return 0;
  }

  std::vector<Packet> frames;
  frames.reserve(ids.size() + 1);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    // Validate the whole batch before sending anything, never silently omit
    // a broadcast/reserved target from a partially valid batch.
    if (ids[i] > kMaxServoId || time_ms > kMaxMoveTimeMs) {
      if (error) *error = ErrorCode::kInvalidArgument;
      rememberRecord(ErrorCode::kInvalidArgument, "invalid unicast ID or move duration");
      return 0;
    }
    frames.push_back(Packet::makeMove(ids[i], ticks[i], time_ms, /*wait=*/true));
  }
  if (broadcast_start) frames.push_back(Packet::broadcastMoveStart());
  return tryWriteFrames(frames, error);
}

bool SerialBus::broadcastMoveStart(ErrorCode* error) noexcept {
  return tryWrite(Packet::broadcastMoveStart(), error);
}

void SerialBus::moveStart() {
  ErrorCode code = ErrorCode::kOk;
  if (!broadcastMoveStart(&code) && errorMode() == ErrorMode::kStrict) {
    detail::throwError(code, lastError(), kNoServoId);
  }
}

bool SerialBus::broadcastMoveStop(ErrorCode* error) noexcept {
  return tryWrite(Packet::broadcast(Command::kMoveStop), error);
}

void SerialBus::moveStop() {
  ErrorCode code = ErrorCode::kOk;
  if (!broadcastMoveStop(&code) && errorMode() == ErrorMode::kStrict) {
    detail::throwError(code, lastError(), kNoServoId);
  }
}

std::size_t SerialBus::setTorqueBatch(const std::vector<std::uint8_t>& ids, bool enable,
                                      ErrorCode* error) noexcept {
  if (ids.empty()) {
    if (error) *error = ErrorCode::kInvalidArgument;
    rememberRecord(ErrorCode::kInvalidArgument, "torque batch must not be empty");
    return 0;
  }
  const std::uint8_t value = enable ? 1 : 0;
  std::vector<Packet> frames;
  frames.reserve(ids.size());
  for (std::uint8_t id : ids) {
    if (id > kMaxServoId) {
      if (error) *error = ErrorCode::kInvalidArgument;
      rememberRecord(ErrorCode::kInvalidArgument, "torque batches require unicast IDs");
      return 0;
    }
    frames.push_back(Packet::make(id, Command::kWriteTorque, &value, 1));
  }
  return tryWriteFrames(frames, error);
}

bool SerialBus::tryReadMany(const std::vector<std::uint8_t>& ids, Command cmd,
                            std::size_t expected_payload, std::vector<Response>* out,
                            ErrorCode* error) noexcept {
  if (!out) {
    if (error) *error = ErrorCode::kInvalidArgument;
    rememberRecord(ErrorCode::kInvalidArgument, "readMany requires an output vector");
    return false;
  }
  out->clear();
  out->reserve(ids.size());
  for (std::uint8_t id : ids) {
    Response response;
    ErrorCode code = ErrorCode::kOk;
    if (!tryTransact(Packet::makeRead(id, cmd), &response, expected_payload, -1, &code) ||
        !response.ok()) {
      if (code == ErrorCode::kOk) code = ErrorCode::kBadChecksum;
      if (error) *error = code;
      return false;
    }
    out->push_back(std::move(response));
  }
  rememberRecord(ErrorCode::kOk, {});
  if (error) *error = ErrorCode::kOk;
  return true;
}

std::vector<Response> SerialBus::readMany(const std::vector<std::uint8_t>& ids,
                                          Command cmd, std::size_t expected_payload) {
  std::vector<Response> out;
  ErrorCode code = ErrorCode::kOk;
  if (!tryReadMany(ids, cmd, expected_payload, &out, &code) &&
      errorMode() == ErrorMode::kStrict) {
    detail::throwError(code, lastError(), kNoServoId);
  }
  return out;
}

bool SerialBus::tryScanIds(std::uint8_t first, std::uint8_t last, Command probe,
                           int timeout_ms, std::vector<std::uint8_t>* out,
                           ErrorCode* error) noexcept {
  if (!out) {
    if (error) *error = ErrorCode::kInvalidArgument;
    rememberRecord(ErrorCode::kInvalidArgument, "scan requires an output vector");
    return false;
  }
  out->clear();
  if (last > kMaxServoId) last = kMaxServoId;
  if (first > last) {
    rememberRecord(ErrorCode::kOk, {});
    if (error) *error = ErrorCode::kOk;
    return true;
  }

  for (std::uint8_t id = first;;) {
    Response response;
    ErrorCode code = ErrorCode::kOk;
    const Packet request = Packet::makeRead(id, probe);
    if (tryTransact(request, &response, 0, timeout_ms, &code) && response.ok() &&
        response.id == id) {
      out->push_back(id);
    } else if (code == ErrorCode::kNotOpen || code == ErrorCode::kIoError) {
      if (error) *error = code;
      return false;  // no point walking the range over a dead port
    }
    if (id == last) break;  // unsigned walk, terminates at 253
    ++id;
  }
  rememberRecord(ErrorCode::kOk, {});
  if (error) *error = ErrorCode::kOk;
  return true;
}

std::vector<std::uint8_t> SerialBus::scanIds(std::uint8_t first, std::uint8_t last,
                                             Command probe, int timeout_ms) {
  std::vector<std::uint8_t> ids;
  ErrorCode code = ErrorCode::kOk;
  if (!tryScanIds(first, last, probe, timeout_ms, &ids, &code) &&
      errorMode() == ErrorMode::kStrict) {
    detail::throwError(code, lastError(), kNoServoId);
  }
  return ids;
}

std::string versionString() {
  return std::string("HiwonderBus ") + HIWONDERBUS_VERSION_STRING +
         " (protocol 0x55 0x55, transport posix)";
}

}  // namespace HiwonderBus
