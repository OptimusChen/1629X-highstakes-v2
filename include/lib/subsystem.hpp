#pragma once

namespace lib {
    class Subsystem {
    public:
        bool initialized = false;
        virtual void initialize() = 0;
        virtual void update() = 0;
        virtual void stop() = 0;
        virtual ~Subsystem() = default;
    };
}