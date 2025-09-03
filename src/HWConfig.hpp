#pragma once
#include "HWConfigCommon.hpp"
namespace HW {
namespace Pin {
    using i2c_sda = decltype(makePinLocation(Kvasir::Io::port0, Kvasir::Io::pin18));
    using i2c_scl = decltype(makePinLocation(Kvasir::Io::port0, Kvasir::Io::pin19));

    using chip_en = decltype(makePinLocation(Kvasir::Io::port0, Kvasir::Io::pin16));
    using pwm     = decltype(makePinLocation(Kvasir::Io::port0, Kvasir::Io::pin17));
    using led     = decltype(makePinLocation(Kvasir::Io::port0, Kvasir::Io::pin25));
}   // namespace Pin

struct I2CConfig {
    static constexpr auto clockSpeed = ClockSpeed;

    static constexpr auto instance       = 1;
    static constexpr auto sdaPinLocation = Pin::i2c_sda{};
    static constexpr auto sclPinLocation = Pin::i2c_scl{};
    static constexpr auto baudRate       = 100'000;
    static constexpr auto isrPriority    = 3;
};

struct Fault_CleanUpAction {
    void operator()() {}
};

struct PinConfig {
    static constexpr auto powerClockEnable = list(
      clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::io_bank0),
      clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::pads_bank0));

    static constexpr auto initStepPinConfig = list(
      makeOutput(HW::Pin::chip_en{}),
      makeOutput(HW::Pin::led{}));
};

struct PwmConfig {
    static constexpr auto clockSpeed = ClockSpeed;

    static constexpr auto top       = 4096;
    static constexpr auto frequency = 10000;
    static constexpr auto invert    = true;
};

}   // namespace HW
