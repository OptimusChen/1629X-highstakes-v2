#include "pros/rtos.h"
#include "mock_hardware.hpp"

namespace pros
{
#ifdef __cplusplus
    namespace c
    {
#endif
        using namespace prosMock;

        uint64_t micros(void)
        {
            return get_tims_us();
        }

        uint32_t millis(void)
        {
            return get_tims_us() / 1000;
        }
#ifdef __cplusplus
    }
#endif
}