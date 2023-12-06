#pragma once
#include "HWConfig.hpp"
///
#include "ALed7709A.hpp"

#include "cmake_git_version/version.hpp"
#include "kvasir/Devices/ADS8675.hpp"
#include "kvasir/Util/FaultHandler.hpp"
#include "kvasir/Util/StackProtector.hpp"

using Clock          = HW::SystickClock;
using StackProtector = Kvasir::StackProtector<>;
using FaultHandler   = Kvasir::Fault::Handler<HW::Fault_CleanUpAction>;

struct DMAConfig {
    static constexpr auto numberOfChannels     = 3;
    static constexpr auto callbackFunctionSize = 64;
};
using DMA = Kvasir::DMA::DmaBase<DMAConfig>;

using Pwm = Kvasir::PWM::PWM<HW::Pin::pwm, HW::PwmConfig>;
using I2C = Kvasir::I2C::I2CBehavior<HW::I2CConfig, Clock, 32>;
using Aled = Kvasir::ALED7709A<I2C, Clock>;

struct Crc {
    using type = std::uint16_t;
    static type calc(std::span<std::byte const> data) {
        return Kvasir::CRC::calcCrc<Kvasir::CRC::CRC_Type::crc16, DMA, DMA::Channel::ch2>(data);
    }
};

using Startup = Kvasir::Startup::Startup<
  HW::ClockSettings,
  Clock,
  HW::ComBackend,
  FaultHandler,
  StackProtector,
  HW::PinConfig,
  DMA,
  Pwm,
//  I2C,
  Crc>;
