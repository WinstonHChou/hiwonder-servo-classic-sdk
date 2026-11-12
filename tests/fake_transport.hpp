// SPDX-License-Identifier: MIT
//
// HiwonderBus -- Classic Serial Bus Servo SDK (0x55 0x55 protocol)
//
// tests/fake_transport.hpp
//   Kept for the test suite's `hiwonder_test::FakeTransport` spelling. The
//   model itself now ships as a public, installable header so application
//   tests can use it too -- see include/HiwonderBus/testing/simulator.hpp.
//
#ifndef HIWONDERBUS_TEST_FAKE_TRANSPORT_HPP
#define HIWONDERBUS_TEST_FAKE_TRANSPORT_HPP

#include "HiwonderBus/testing/simulator.hpp"

namespace hiwonder_test {

using FakeTransport = HiwonderBus::testing::SimulatedTransport;
using SimulatedServo = HiwonderBus::testing::SimulatedServo;

}  // namespace hiwonder_test

#endif  // HIWONDERBUS_TEST_FAKE_TRANSPORT_HPP
