/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/Location.h>

class AP_Beacon_Backend;

#define AP_BEACON_MAX_BEACONS 4
#define AP_BEACON_TIMEOUT_MS 300
#define AP_BEACON_MINIMUM_FENCE_BEACONS 3

class AP_Beacon
{
public:
    friend class AP_Beacon_Backend;

    AP_Beacon(AP_SerialManager &_serial_manager);

    // get singleton instance 获取单个实例
    static AP_Beacon *get_singleton() { return _singleton; }

    // external position backend types (used by _TYPE parameter)    外部位置后端类型(由_TYPE参数使用)  
    enum AP_BeaconType {        // 信标类型
        AP_BeaconType_None   = 0,
        AP_BeaconType_Pozyx  = 1,
        AP_BeaconType_Marvelmind = 2,
        AP_BeaconType_Nooploop  = 3,
        AP_BeaconType_SITL   = 10
    };

    // The AP_BeaconState structure is filled in by the backend driver    AP_BeaconState结构由后端驱动程序填充  
    struct BeaconState {        // 信标状态
        uint16_t id;            // unique id of beacon
        bool     healthy;       // true if beacon is healthy
        float    distance;      // distance from vehicle to beacon (in meters)
        uint32_t distance_update_ms;    // system time of last update from this beacon
        Vector3f position;      // location of beacon as an offset from origin in NED in meters
    };

    // initialise any available position estimators 初始化
    void init(void);

    // return true if beacon feature is enabled 如果信标功能被启用，则返回true。增加了打印消息输出
    bool enabled(void) const;

    // return true if sensor is basically healthy (we are receiving data) 如果传感器基本正常则返回true(我们正在接收数据)，增加了打印消息输出
    bool healthy(void) const;

    // update state of all beacons 更新所有信标的状态
    void update(void);

    // return origin of position estimate system in lat/lon 以经纬度返回位置估计系统的原点
    bool get_origin(Location &origin_loc) const;

    // return vehicle position in NED from position estimate system's origin in meters 位置估计系统原点返回车辆在NED中的位置，单位是米
    bool get_vehicle_position_ned(Vector3f& pos, float& accuracy_estimate) const;

    // return the number of beacons 返回信标的数量
    uint8_t count() const;

    // methods to return beacon specific information 返回信标特定信息的方法

    // return all beacon data 返回所有信标数据
    bool get_beacon_data(uint8_t beacon_instance, struct BeaconState& state) const;

    // return individual beacon's id 返回单个信标的id
    uint8_t beacon_id(uint8_t beacon_instance) const;

    // return beacon health 返回信标是否良好
    bool beacon_healthy(uint8_t beacon_instance) const;

    // return distance to beacon in meters 返回到信标的距离以米为单位
    float beacon_distance(uint8_t beacon_instance) const;

    // return NED position of beacon in meters relative to the beacon systems origin 返回信标相对于信标系统原点的NED位置(单位为米) 
    Vector3f beacon_position(uint8_t beacon_instance) const;

    // return last update time from beacon in milliseconds 返回上次更新时间从信标毫秒 
    uint32_t beacon_last_update_ms(uint8_t beacon_instance) const;

    // update fence boundary array 更新围栏边界阵列
    void update_boundary_points();

    // return fence boundary array 返回栅栏边界阵列
    const Vector2f* get_boundary_points(uint16_t& num_points) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_Beacon *_singleton;

    // check if device is ready 检查设备是否准备就绪
    bool device_ready(void) const;

    // find next boundary point from an array of boundary points given the current index into that array
    // returns true if a next point can be found
    //   current_index should be an index into the boundary_pts array
    //   start_angle is an angle (in radians), the search will sweep clockwise from this angle
    //   the index of the next point is returned in the next_index argument
    //   the angle to the next point is returned in the next_angle argument
    // 从给定当前索引的边界点数组中查找下一个边界点
    // 如果可以找到下一个点，则返回true
    // current_index应该是boundary y_pts数组的索引
    // start_angle是一个角度(以弧度为单位)，搜索将从这个角度顺时针扫描
    // 下一个点的索引在next_index参数中返回
    // 到下一个点的角度在next_angle参数中返回
    static bool get_next_boundary_point(const Vector2f* boundary, uint8_t num_points, uint8_t current_index, float start_angle, uint8_t& next_index, float& next_angle);

    // parameters 参数
    AP_Int8 _type;
    AP_Float origin_lat;
    AP_Float origin_lon;
    AP_Float origin_alt;
    AP_Int16 orient_yaw;

    // external references 外部引用
    AP_Beacon_Backend *_driver;
    AP_SerialManager &serial_manager;

    // last known position 最后已知位置
    Vector3f veh_pos_ned;    //NED坐标系飞机位置
    float veh_pos_accuracy;  //位置精度
    uint32_t veh_pos_update_ms;

    // individual beacon data 单个信标数据
    uint8_t num_beacons = 0;
    BeaconState beacon_state[AP_BEACON_MAX_BEACONS];

    // fence boundary 围栏边界
    Vector2f boundary[AP_BEACON_MAX_BEACONS+1]; // array of boundary points (used for fence) 边界点阵列(用于围栏)
    uint8_t boundary_num_points;                // number of points in boundary 边界点的数量
    uint8_t boundary_num_beacons;               // total number of beacon points consumed while building boundary 围栏边界消耗的信标点总数
};

namespace AP {
    AP_Beacon *beacon();
};
