#include "pros/distance.hpp"
#include "mock_hardware.hpp"

namespace pros
{
    inline namespace v5
    {
        struct MockDistanceSensor
        {
            /** Distance in mm. */
            int32_t distance_mm = 0;
            /** velocity in m/s */
            double object_velocity_mps = 0;
            int32_t object_size = 0;
            /** Confidence from 0 to 63, with 63 the highest confidence. */
            int32_t confidence = 63;
        };

        using namespace prosMock;

        Distance::Distance(const std::uint8_t port) : Device(port, DeviceType::distance)
        {
            install_mock_devices(port, DeviceType::distance, this);
        }

        std::int32_t Distance::get()
        {
            MockDistanceSensor *distance_sensors = static_cast<MockDistanceSensor *>(get_mock_device(_port).impl);
            return distance_sensors->distance_mm;
        }

        bool Device::is_installed()
        {
            return get_plugged_type(_port) == DeviceType::distance;
        }

        std::int32_t Distance::get_distance()
        {
            return this->get();
        }

        std::int32_t Distance::get_confidence()
        {
            MockDistanceSensor *distance_sensors = static_cast<MockDistanceSensor *>(get_mock_device(_port).impl);
            return distance_sensors->confidence;
        }

        std::int32_t Distance::get_object_size()
        {
            MockDistanceSensor *distance_sensors = static_cast<MockDistanceSensor *>(get_mock_device(_port).impl);
            return distance_sensors->object_size;
        }

        double Distance::get_object_velocity()
        {
            MockDistanceSensor *distance_sensors = static_cast<MockDistanceSensor *>(get_mock_device(_port).impl);
            return distance_sensors->object_velocity_mps;
        }
    };
}