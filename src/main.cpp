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
    UC_LOG_D("{}", CMakeGitVersion::FullVersion);
    auto next = Clock::now();
    while(true) {
        auto const now = Clock::now();
        if(now > next) {
            UC_LOG_D("foo");
            next += 1s;
        }
    }
}

KVASIR_START(Startup)
