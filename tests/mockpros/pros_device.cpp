#include "pros/device.hpp"
#include "mock_hardware.hpp"

namespace pros
{
    inline namespace v5
    {
        Device::Device(const std::uint8_t port) : Device(port, DeviceType::none)
        {
        }

        using namespace prosMock;
        DeviceType Device::get_plugged_type(std::uint8_t port)
        {
            return get_mock_device(port).type;
        }
    };
}