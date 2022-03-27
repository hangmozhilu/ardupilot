#include "Copter.h"

// guided_init - initialise guided controller 引导模式初始化 - 初始化引导控制器
bool Copter::ModeDrawStar::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        // initialise yaw
        auto_yaw.set_mode_to_default(false);

        path_num = 0;
        generate_path();

        // start in position control mode
        pos_control_start();
        return true;
    }else{
        return false;
    }
}

void Copter::ModeDrawStar::generate_path()
{
    //飞机机体坐标系，机头前方X+,机体右舷Y+,机体下方Z+
    //第0航点为圆心
    float radius_cm = g2.star_radius_cm;//默认10m米
    wp_nav->get_wp_stopping_point(path[0]); //取得停止点
    path[1] = path[0] + Vector3f(1.0f,0,0)*radius_cm;

    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];

}

// initialise guided mode's position controller 初始化引导模式的位置控制器
void Copter::ModeDrawStar::pos_control_start()
{ 
    // initialise waypoint and spline controller 初始化航点和样条曲线控制器
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used 不需要检查返回状态，因为没有使用地形数据 
    wp_nav->set_wp_destination(path[0], false); //把当前的目标航点设置为停止点

    // initialise yaw 初始化航向
    auto_yaw.set_mode_to_default(false);
}

// guided_run - runs the guided controller
// should be called at 100hz or more 应该以100hz或更多的频率调用
void Copter::ModeDrawStar::run()
{
    if (path_num < 6) {
        if (wp_nav->reached_wp_destination()) {
            path_num++;
            wp_nav->set_wp_destination(path[path_num], false);
        }
    } else if (path_num == 6 && wp_nav->reached_wp_destination()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,
                        "Draw star finished,now go into Land Mode");
        copter.set_mode(LAND, MODE_REASON_MISSION_END);

        // if (copter.set_mode(LAND, MODE_REASON_MISSION_END)) {  //错误，切换模式以后，飞机进入其他模式了，不会再执行后面的代码
        //     gcs().send_text(MAV_SEVERITY_CRITICAL, "Land Mode change successful!");
        // } else {
        //     gcs().send_text(MAV_SEVERITY_CRITICAL, "Land Mode change false!");
        // }
    }
    pos_control_run();  //每400Hz调用一次
 }

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::ModeDrawStar::pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately 如果不是自动解锁或电机未设置油门为零，则立即退出
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac(); //油门为零
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0; //target_yaw_rate 目标航向转动速率
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate //获取飞手期望的航向转动速率
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller 调用姿态控制器
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot 横滚和俯仰由航点控制器控制，偏航速率由驾驶员控制
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item 从航点控制器获得横滚和俯仰，从mavlink命令或任务清单获得偏航速率
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading() 横滚、俯仰由航点控制器控制，偏航航向由GCS或自动航向控制
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}


