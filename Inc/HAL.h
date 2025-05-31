#pragma once

#include "AP_HAL_Namespace.h"

namespace AP_HAL {

class HAL {
public:
    HAL() {
        //AP_HAL::init();
    }

    struct Callbacks {
        virtual void setup() = 0;
        virtual void loop() = 0;
        virtual ~Callbacks() = default;
    };

    struct FunCallbacks : public Callbacks {
        FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void))
            : _setup(setup_fun), _loop(loop_fun) {}

        void setup() override { _setup(); }
        void loop() override { _loop(); }

    private:
        void (*_setup)(void);
        void (*_loop)(void);
    };

    virtual void run(int argc, char* const argv[], Callbacks* callbacks) const = 0;

    static constexpr uint8_t num_serial = 10;
};

} // namespace AP_HAL