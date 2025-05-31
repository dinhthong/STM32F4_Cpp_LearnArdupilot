#pragma once

#include "string.h"
#include "HAL.h"
//#include "utility/functor.h"

namespace AP_HAL {
		class HAL;  // forward declaration of HAL
    // Must be implemented by the concrete HALs and return the same reference.
    const HAL& get_HAL();
    //HAL& get_HAL_mutable();
}
