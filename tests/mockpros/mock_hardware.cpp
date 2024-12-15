
#include "mock_hardware.hpp"

namespace prosMock
{
    /**
     * Simulate devices.
     */
    static std::array<MockDevice, 21> devices = {};
    void install_mock_devices(const uint8_t port, const pros::DeviceType type, void *impl)
    {
        devices[port].type = type;
        devices[port].impl = impl;
    }

    MockDevice &get_mock_device(const uint8_t port)
    {
        return devices[port];
    }

    /*
     * Simulate time.
     */
    static uint64_t time_us = 0;
    uint64_t get_tims_us(void)
    {
        return time_us;
    }

    void advance_time_us(uint64_t delta_us)
    {
        time_us += delta_us;
    }

    void advance_time_ms(uint32_t delta_ms)
    {
        time_us += static_cast<uint64_t>(delta_ms) * 1000;
    }
}