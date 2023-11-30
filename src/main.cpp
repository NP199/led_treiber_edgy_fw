#include "ApplicationConfig.hpp"
//
#include "aglio/packager.hpp"
#include "aglio/remote_fmt.hpp"
#include "cmake_git_version/version.hpp"
#include "kvasir/Util/StaticVector.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "remote_fmt/remote_fmt.hpp"
#include "ALed7709A.hpp"

using packager = aglio::Packager<aglio::CrcConfig<Crc>>;
/*
        std::uint32_t const RTop = PwmR::getTop();
        std::uint32_t const RDuty = static_cast<std::uint16_t>(((RTop + 1) * R) / Max);
        PwmR::setDuty(RDuty);
  */

int main() {
    UC_LOG_D("{}", CMakeGitVersion::FullVersion);
    auto next = Clock::now();
    //auto off = Clock::now();
    std::uint16_t Duty = (Pwm::getTop() * (100*255)) / (255*255);
    UC_LOG_D("Duty: {} getTop:{}", Duty, Pwm::getTop());
    Pwm::setDuty(Duty);
    apply(set(HW::Pin::chip_en{}));
    while(true) {
        auto const now = Clock::now();

        if(now > next) {
            next = Clock::now();
            UC_LOG_D("ping");
            next += 1s;
            apply(set(HW::Pin::led{}));
        }
    }
}

KVASIR_START(Startup)
