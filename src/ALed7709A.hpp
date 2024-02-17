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

    //Device Addresse ALED7709A 0x28h
    struct Command {
        static constexpr std::array getId{std::byte{0x00}};
        /*
        static constexpr std::array<std::byte, 1> getID() {
            std::array<std::byte, 1> ret;
            ret[0] = 0x00_b;
       }
             */
    };

    static constexpr std::size_t readout_packet_size{1};

    enum class State { reset, init, idle, write_wait, get_msg, read_wait };

    State                                 st_;
    tp                                    waitTime_;
    std::array<std::optional<std::byte>, 1> msgraw_;
    std::uint8_t const                    i2caddress;

    static constexpr auto startup_time{std::chrono::milliseconds(250)};
    static constexpr auto fail_retry_time{std::chrono::milliseconds(250)};
    static constexpr auto sensor_read_time{std::chrono::milliseconds(100)};

    constexpr explicit ALED7709A(std::uint8_t devAdress)
      : st_{State::reset}
      , waitTime_{tp::min()}
      , i2caddress{devAdress} {}

    std::array<std::optional<std::byte>, 1> returnMsg() const { return msgraw_; }

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
                    st_ = State::read_wait;
                    I2C::send_receive(currentTime, i2caddress, Command::getId, 1);
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
                case OS::ongoing:
                    {
                    }
                    break;
                case OS::succeeded:
                    {
                        UC_LOG_D("succeeded");
                        st_       = State::idle;
                        waitTime_ = currentTime;
                        release();
                        std::array<std::byte, readout_packet_size> buffer{};
                        I2C::getReceivedBytes(buffer);
                        UC_LOG_D("foo: {}", buffer);
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
