/*

 */
#include "AP_HAL.h"
#include "HAL.h"
#include "HAL_ChibiOS_Class.h"
//#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
//#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS_Private.h>
// #include "shared_dma.h"
// #include "sdcard.h"
// #include "hwdef/common/usbcfg.h"
// #include "hwdef/common/stm32_util.h"
// #include "hwdef/common/watchdog.h"
// #include <AP_BoardConfig/AP_BoardConfig.h>
// #include <AP_InternalError/AP_InternalError.h>

// #include <AP_Vehicle/AP_Vehicle_Type.h>
// #include <AP_HAL/SIMState.h>

static AP_HAL::HAL::Callbacks* g_callbacks;

HAL_ChibiOS::HAL_ChibiOS() :
    AP_HAL::HAL()
{}
	
static void main_loop()
{
    g_callbacks->setup();


    while (true) {
        g_callbacks->loop();

    }
}

void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
#if defined(HAL_EARLY_WATCHDOG_INIT) && !defined(DISABLE_WATCHDOG)
    stm32_watchdog_init();
    stm32_watchdog_pat();
#endif

    g_callbacks = callbacks;

    //Takeover main
    main_loop();
}

static HAL_ChibiOS hal_chibios;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_chibios;
}

