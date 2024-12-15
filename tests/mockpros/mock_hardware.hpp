/**
 * Mock the hardware.
 */

#pragma once

#include "pros/device.hpp"

namespace prosMock
{
    struct MockDevice
    {
        pros::DeviceType type = pros::DeviceType::none;
        void *impl = nullptr;
    };

    void install_mock_devices(const uint8_t port, const pros::DeviceType type, void *impl);
    MockDevice &get_mock_device(const uint8_t port);

    uint64_t get_tims_us();
    void advance_time_us(uint64_t delta_us);
    void advance_time_ms(uint32_t delta_ms);
}
