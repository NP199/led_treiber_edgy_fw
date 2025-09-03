#include "ApplicationConfig.hpp"
//
#include "aglio/packager.hpp"
#include "aglio/remote_fmt.hpp"
#include "cmake_git_version/version.hpp"
#include "kvasir/Util/StaticVector.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "remote_fmt/remote_fmt.hpp"

using packager = aglio::Packager<aglio::CrcConfig<Crc>>;
/*
        std::uint32_t const RTop = PwmR::getTop();
        std::uint32_t const RDuty = static_cast<std::uint16_t>(((RTop + 1) * R) / Max);
        PwmR::setDuty(RDuty);
  */

int main() {
    //Kvasir::Register::makeOutput(HW::Pin::i2c_sda{}); // ging nicht musste in HWConfig gesetzt werden
    //Kvasir::Register::makeOutput(HW::Pin::i2c_scl{}); // ging nicht musste in HWConfig gesetzt werden
    UC_LOG_D("{}", CMakeGitVersion::FullVersion);
//    Aled ledChip{0x28};

    auto next   = Clock::now();
    //auto off = Clock::now();
    std::uint16_t Duty = (Pwm::getTop() * (100*255)) / (255*255);
    UC_LOG_D("Duty: {} getTop:{}", Duty, Pwm::getTop());
    Pwm::setDuty(Duty);
    apply(set(HW::Pin::chip_en{}));
    while(true) {
        auto const now = Clock::now();

        if(now > next) {
            apply(toggle(HW::Pin::led{}));
            //apply(toggle(HW::Pin::i2c_scl{}));
            //apply(toggle(HW::Pin::i2c_sda{}));
            next = Clock::now();
            UC_LOG_D("ping");
            next += 1s;
        }
        //ledChip.handler();
    }
}

KVASIR_START(Startup)
