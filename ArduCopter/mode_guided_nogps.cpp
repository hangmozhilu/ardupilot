#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED

/*
 * Init and run calls for guided_nogps flight mode 初始化并运行引导nogps飞行模式的调用
 */

// initialise guided_nogps controller 初始化引导无gps控制器
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // start in angle control mode 开始角度控制模式
    ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller 引导运行-运行引导控制器
// should be called at 100hz or more 应该以100hz或更多的频率调用
void ModeGuidedNoGPS::run()
{
    // run angle controller
    ModeGuided::angle_control_run();
}

#endif
