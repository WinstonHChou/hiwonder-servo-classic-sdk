// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// HiwonderBus.hpp -- umbrella header. Include this from application code.
//
//   #include <HiwonderBus/HiwonderBus.hpp>
//   using namespace HiwonderBus;
//
//   BusConfig cfg; cfg.device = "/dev/ttyUSB0";
//   SerialBus bus(cfg);
//   bus.open("/dev/ttyUSB0");
//   LX16A servo(bus, 1);
//   servo.initialize();
//   servo.move(90.0f, 500);
//
// Individual subsystems can still be included on their own:
//   <HiwonderBus/protocol.hpp>   -- framing, opcodes, tick geometry
//   <HiwonderBus/exceptions.hpp> -- the error hierarchy
//   <HiwonderBus/serial_bus.hpp> -- POSIX transport + batch primitives
//   <HiwonderBus/servo.hpp>      -- the degree-based servo object + aliases
//
#ifndef HIWONDERBUS_HIWONDERBUS_HPP
#define HIWONDERBUS_HIWONDERBUS_HPP

#include "HiwonderBus/exceptions.hpp"
#include "HiwonderBus/protocol.hpp"
#include "HiwonderBus/serial_bus.hpp"
#include "HiwonderBus/servo.hpp"

/// Library version, kept in step with CMake's `project(... VERSION ...)`.
#define HIWONDERBUS_VERSION_MAJOR 0
#define HIWONDERBUS_VERSION_MINOR 1
#define HIWONDERBUS_VERSION_PATCH 0
#define HIWONDERBUS_VERSION_STRING "0.1.0"

#define HIWONDERBUS_VERSION                                     \
  (HIWONDERBUS_VERSION_MAJOR * 10000 + HIWONDERBUS_VERSION_MINOR * 100 + \
   HIWONDERBUS_VERSION_PATCH)

namespace HiwonderBus {

/// "HiwonderBus 0.1.0 (posix)" -- useful in bug reports and log banners.
std::string versionString();

}  // namespace HiwonderBus

#endif  // HIWONDERBUS_HIWONDERBUS_HPP
