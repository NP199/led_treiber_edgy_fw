/*
            * Device Indentification Register  readonly (7:0)
            * Add: 00h
            * POR A = 13h
            * silicon version identification (bit7:4)
            * 3h: cut x.3 (hardwired)
            * revision identification (bit3:0)
            * 1h: cut 1.x (hardwired)
            */
/*
            *   Reading from Register
            *   start Condition followed by 7 bit-device address
            *   bit 8 : w(0) bit
            *   Acknowledge Pulse pulse received
            *   send address
            *   Acknowledge pulse received
            *   Master generates Restart Condition
            *   sending 7 bit device address
            *   bit 8 : (1)r
            *   Acknowledge pulse
            *   Start sending Data one databit per clock pulse
            *   Master generates Stop condition to terminate Communication
            */
//send_receive(tp const& currentTime,
//std::uint8_t address, device Adress
//C const& c, c = container with Register Number
//std::uint8_t size)
#pragma once

#include "kvasir/Devices/SharedBusDevice.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace Kvasir {
namespace aled7709 {
    enum class Access : std::uint8_t { RO, WO, RW, RSV };

    // Register-Map
    enum class Reg : std::uint8_t {
        DEVID    = 0x00,   // RO
        DEVEN    = 0x01,
        PWM1H    = 0x02,
        PWM1L    = 0x03,
        GAIN1    = 0x04,
        PWM2H    = 0x05,
        PWM2L    = 0x06,
        GAIN2    = 0x07,
        PWM3H    = 0x08,
        PWM3L    = 0x09,
        GAIN3    = 0x0A,
        PWM4H    = 0x0B,
        PWM4L    = 0x0C,
        GAIN4    = 0x0D,
        PWMH_STS = 0x0E,   // RO
        PWML_STS = 0x0F,   // RO
        GAIN_STS = 0x10,   // RO
        CHCFG    = 0x11,
        OUTCFG   = 0x12,
        BOOSTCFG = 0x13,
        DIMCFG   = 0x14,
        FMCFG    = 0x15,
        FMASK    = 0x16,
        DEVSTA   = 0x17,   // RO
        CHSTA    = 0x18,   // RO
        INITSTA  = 0x19    // RO
    };

    // ---- 2) Register-Traits (RO/WO/RW/RSV) ----
    template<Reg R>
    struct reg_traits {
        static constexpr Access access = Access::RW;
    };

    template<>
    struct reg_traits<Reg::DEVID> {
        static constexpr Access access = Access::RO;
    };
    template<>
    struct reg_traits<Reg::PWMH_STS> {
        static constexpr Access access = Access::RO;
    };
    template<>
    struct reg_traits<Reg::PWML_STS> {
        static constexpr Access access = Access::RO;
    };
    template<>
    struct reg_traits<Reg::GAIN_STS> {
        static constexpr Access access = Access::RO;
    };
    template<>
    struct reg_traits<Reg::DEVSTA> {
        static constexpr Access access = Access::RO;
    };
    template<>
    struct reg_traits<Reg::CHSTA> {
        static constexpr Access access = Access::RO;
    };
    template<>
    struct reg_traits<Reg::INITSTA> {
        static constexpr Access access = Access::RO;
    };

    // Bitmasken
    namespace Bits {
        struct Deven {
            static constexpr std::uint8_t DEN  = 1u << 0;   // enable
            static constexpr std::uint8_t CLRF = 1u << 7;   // write-only action: clear faults
        };
        struct Chcfg {
            static constexpr std::uint8_t CEN1 = 1u << 0, CEN2 = 1u << 1, CEN3 = 1u << 2,
                                          CEN4   = 1u << 3;
            static constexpr std::uint8_t CH1CH2 = 1u << 4, CH2CH3 = 1u << 5, CH3CH4 = 1u << 6;
        };
        struct Dimcfg {
            static constexpr std::uint8_t REG_PWMI  = 1u << 5;
            static constexpr std::uint8_t PWMI_DRCT = 1u << 4;
            static constexpr std::uint8_t LECC      = 1u << 3;   // 0 linear, 1 exponential
            static constexpr std::uint8_t UMDM      = 1u << 2;   // 1 mixed
            static constexpr std::uint8_t GLDM      = 1u << 1;   // 0 global, 1 local
            static constexpr std::uint8_t FDIM_MASK = 0x07;      // [2:0]
        };
    }   // namespace Bits

    // Bit-Traits
    template<Reg R, std::uint8_t MASK>
    struct bit_traits {
        static constexpr bool write_only      = false;
        static constexpr bool write_one_clear = false;   // W1C
        static constexpr bool write_one_set   = false;   // W1S
        static constexpr bool reserved_bits   = false;
    };

    template<>
    struct bit_traits<Reg::DEVEN, Bits::Deven::CLRF> {
        static constexpr bool write_only      = true;
        static constexpr bool write_one_clear = false;
        static constexpr bool write_one_set   = false;
        static constexpr bool reserved_bits   = false;
    };

    // Concepts aus Traits
    template<Reg R>
    concept RegReadable
      = (reg_traits<R>::access == Access::RO) || (reg_traits<R>::access == Access::RW);
    template<Reg R>
    concept RegWritable
      = (reg_traits<R>::access == Access::WO) || (reg_traits<R>::access == Access::RW);
    template<Reg R>
    concept RegReserved = (reg_traits<R>::access == Access::RSV);

    template<Reg R, std::uint8_t MASK>
    concept BitWriteOnly = bit_traits<R, MASK>::write_only;
    template<Reg R, std::uint8_t MASK>
    concept BitW1C = bit_traits<R, MASK>::write_one_clear;
    template<Reg R, std::uint8_t MASK>
    concept BitW1S = bit_traits<R, MASK>::write_one_set;
    template<Reg R, std::uint8_t MASK>
    concept BitReserved = bit_traits<R, MASK>::reserved_bits;
    template<Reg R, std::uint8_t MASK>
    concept BitNormal
      = !BitWriteOnly<R, MASK> && !BitW1C<R, MASK> && !BitW1S<R, MASK> && !BitReserved<R, MASK>;

}   // namespace aled7709

template<typename I2C, typename Clock>
struct ALED7709A : SharedBusDevice<I2C> {
    using tp   = typename Clock::time_point;
    using Base = SharedBusDevice<I2C>;
    using Base::acquire;
    using Base::incementErrorCount;   // Fehlendes 'r' pull-request nachreichen
    using Base::isOwner;
    using Base::release;
    using Base::resetErrorCount;
    using Base::resetHandler;
    using OS = typename I2C::OperationState;

    static constexpr std::uint8_t i2c_addr_a = 0x28;   //Device Addresse ALED7709A 0x28h

    using Reg = aled7709::Reg;
    //using Bits = aled7709::Bits;

    static constexpr std::size_t readout_packet_size{1};

    enum class State { reset, init, idle, write_wait, get_msg, read_wait };

    State                                   st_{State::reset};
    tp                                      waitTime_{tp::min()};
    std::array<std::optional<std::byte>, 1> msgraw_;
    std::uint8_t const                      i2caddress;

    static constexpr auto startup_time{std::chrono::milliseconds(250)};
    static constexpr auto fail_retry_time{std::chrono::milliseconds(250)};
    static constexpr auto sensor_read_time{std::chrono::milliseconds(100)};

    constexpr explicit ALED7709A(std::uint8_t devAdress = i2c_addr_a) : i2caddress{devAdress} {}

    std::array<std::optional<std::byte>, 1> returnMsg() const { return msgraw_; }

    template<Reg R>
    bool write8bit(std::uint8_t bitValue) {
        if(!acquire()) return false;
        auto const               currentTime = Clock::now();
        std::array<std::byte, 2> buf{
          static_cast<std::byte>(R),
          static_cast<std::byte>(bitValue)};
        I2C::send(currentTime, i2caddress, buf);
        while(true) {
            switch(I2C::operationState(Clock::now())) {
            case OS::ongoing: continue;
            case OS::succeeded: release(); return true;
            case OS::failed:
                release();
                incementErrorCount();
                return false;
            }
        }
    }

    template<Reg R>
    bool read8bit(std::uint8_t& receivedValue) {
        if(!acquire()) return false;
        auto const                        currentTime = Clock::now();
        const std::array<std::byte, 1> selectedReg{static_cast<std::byte>(R)};
        //I2C::send_receive(currentTime, i2caddress, Command::getId, 1);
        I2C::send_receive(currentTime, i2caddress, selectedReg, readout_packet_size);
        while(true) {
            switch(I2C::operationState(Clock::now())) {
            case OS::ongoing: continue;
            case OS::succeeded:
                {
                    std::array<std::byte, 1> rx{};
                    I2C::getReceivedBytes(rx);
                    receivedValue = static_cast<std::uint8_t>(rx[0]);
                    release();
                    return true;
                }
            case OS::failed:
                release();
                incementErrorCount();
                return false;
            }
        }
    }

    template<Reg R>
        requires aled7709::RegReadable<R>
    bool readReg(std::uint8_t& v) {
        return read8bit<R>(v);
    }

    template<Reg R>
        requires aled7709::RegWritable<R>
    bool writeReg(std::uint8_t v) {
        return write8bit<R>(v);
    }

    // Blockade für Reserved (falls definiert):
    template<Reg R>
        requires aled7709::RegReserved<R>
    bool readReg(std::uint8_t&) {
        static_assert(!aled7709::RegReserved<R>, "READ of RESERVED register");
        return false;
    }
    template<Reg R>
        requires aled7709::RegReserved<R>
    bool writeReg(std::uint8_t) {
        static_assert(!aled7709::RegReserved<R>, "WRITE of RESERVED register");
        return false;
    }

    template<Reg R, std::uint8_t MASK>
        requires(aled7709::RegWritable<R> && aled7709::BitNormal<R, MASK>)
    bool setBits() {
        std::uint8_t v{};
        if(!readReg<R>(v)) return false;
        v = static_cast<std::uint8_t>(v | MASK);
        return writeReg<R>(v);
    }

    // clearBits: W1C → direkter Write; normal/W1S → RMW; write-only → verbieten
    template<Reg R, std::uint8_t MASK>
        requires(aled7709::RegWritable<R> && aled7709::BitW1C<R, MASK>)
    bool clearBits() {
        return writeReg<R>(MASK);
    }

    template<Reg R, std::uint8_t MASK>
        requires(aled7709::RegWritable<R> && aled7709::BitWriteOnly<R, MASK>)
    bool clearBits() {
        static_assert(!aled7709::BitWriteOnly<R, MASK>, "Cannot clear a write-only action bit");
        return false;
    }

    template<Reg R, std::uint8_t MASK>
        requires(
          aled7709::RegWritable<R> && (aled7709::BitNormal<R, MASK> || aled7709::BitW1S<R, MASK>))
    bool clearBits() {
        std::uint8_t v{};
        if(!readReg<R>(v)) return false;
        v = static_cast<std::uint8_t>(v & ~MASK);
        return writeReg<R>(v);
    }

    bool clearFaults() { return setBits<Reg::DEVEN, aled7709::Bits::Deven::CLRF>(); }

    bool enable(bool on) {
        if(on) {
            return setBits<Reg::DEVEN, aled7709::Bits::Deven::DEN>();
        } else {
            return clearBits<Reg::DEVEN, aled7709::Bits::Deven::DEN>();
        }
    }

    /* Burst Funktionen + 16bit write muesste ergaenzt werden.
    // Beispiel: Kanal-Enable Maske (CHCFG.CENx) — RMW über set/clearBits nutzen
    bool setChannelEnableMask(std::uint8_t maskCEN_0to3) {
        // lösche erst alle 4 CEN, setze dann gewünschte:
        if(!clearBits<
             Reg::CHCFG,
             (std::uint8_t)(Bits::Chcfg::CEN1 | Bits::Chcfg::CEN2 | Bits::Chcfg::CEN3
                            | Bits::Chcfg::CEN4)>())
            return false;
        // mask mapping: bit0->CEN1, bit1->CEN2, ...
        std::uint8_t m = 0;
        if(maskCEN_0to3 & 0x01) m |= Bits::Chcfg::CEN1;
        if(maskCEN_0to3 & 0x02) m |= Bits::Chcfg::CEN2;
        if(maskCEN_0to3 & 0x04) m |= Bits::Chcfg::CEN3;
        if(maskCEN_0to3 & 0x08) m |= Bits::Chcfg::CEN4;
        if(m) return setBits<Reg::CHCFG, m>();
        return true;
    }

    // 16-bit PWM je Kanal
    bool setPwm1(std::uint16_t v) { return writeReg16<Reg::PWM1H, Reg::PWM1L>(v); }
    bool setPwm2(std::uint16_t v) { return writeReg16<Reg::PWM2H, Reg::PWM2L>(v); }
    bool setPwm3(std::uint16_t v) { return writeReg16<Reg::PWM3H, Reg::PWM3L>(v); }
    bool setPwm4(std::uint16_t v) { return writeReg16<Reg::PWM4H, Reg::PWM4L>(v); }
    */

    bool valid() const {
        return std::all_of(msgraw_.begin(), msgraw_.end(), [](auto const& msg) {
            return msg.has_value();
        });
    }
    void clear() {
        for(auto& msg : msgraw_) {
            msg.reset();
        }
    }

    void handler() {
        auto const currentTime = Clock::now();
        if(resetHandler()) {
            clear();
            st_ = State::reset;
        }

        switch(st_) {
        case State::reset:
            {
                st_       = State::init;
                waitTime_ = currentTime + startup_time;
            }
            break;
        case State::init:
            {
                if(currentTime > waitTime_) {
                    st_ = State::idle;
                }
            }
            break;
        case State::idle:
            {
                if(currentTime > waitTime_ && acquire()) {
                    release();
                    (void)enable(true);
                    st_ = State::get_msg;
                    //I2C::send_receive(currentTime, i2caddress, Command::getId, 1);
                    //I2C::send_receive(currentTime, i2caddress, Setting::Deven::DEN, 1);
                }
            }
            break;
        case State::write_wait:
            {
                assert(isOwner());
                switch(I2C::operationState(currentTime)) {
                case OS::ongoing:
                    {
                    }
                    break;
                case OS::succeeded:
                    {
                        st_       = State::get_msg;
                        waitTime_ = currentTime + sensor_read_time;
                        release();
                    }
                    break;
                case OS::failed:
                    {
                        st_       = State::idle;
                        waitTime_ = currentTime + fail_retry_time;
                        release();
                        incementErrorCount();
                    }
                    break;
                }
            }
            break;
        case State::get_msg:
            {
                if(currentTime > waitTime_ && acquire()) {
                    st_ = State::read_wait;
                    I2C::receive(currentTime, i2caddress, readout_packet_size);
                }
            }
            break;
        case State::read_wait:
            {
                assert(isOwner());
                switch(I2C::operationState(currentTime)) {
                case OS::ongoing: break;
                case OS::succeeded:
                    {
                        UC_LOG_D("succeeded");
                        st_       = State::idle;
                        waitTime_ = currentTime;
                        release();
                        std::array<std::byte, readout_packet_size> buffer{};
                        I2C::getReceivedBytes(buffer);
                        UC_LOG_D("DEVID: 0x{:02X}", buffer[0]);
                    }
                    break;
                case OS::failed:
                    {
                        UC_LOG_D("failed");
                        st_       = State::idle;
                        waitTime_ = currentTime + fail_retry_time;
                        release();
                        incementErrorCount();
                    }
                    break;
                }
            }
            break;
        }
    }
};
}   // namespace Kvasir
