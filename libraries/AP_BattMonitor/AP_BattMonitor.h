#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_BattMonitor_Params.h"

// maximum number of battery monitors 电池监视器的最大数量
#define AP_BATT_MONITOR_MAX_INSTANCES       9

// first monitor is always the primary monitor 第一个监视器始终是主监视器
#define AP_BATT_PRIMARY_INSTANCE            0

#define AP_BATT_SERIAL_NUMBER_DEFAULT       -1

#define AP_BATT_MONITOR_TIMEOUT             5000

#define AP_BATT_MONITOR_RES_EST_TC_1        0.5f
#define AP_BATT_MONITOR_RES_EST_TC_2        0.1f

#if !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#define AP_BATT_MONITOR_CELLS_MAX           14
#else
#define AP_BATT_MONITOR_CELLS_MAX           12
#endif

#ifndef HAL_BATTMON_SMBUS_ENABLE
#define HAL_BATTMON_SMBUS_ENABLE 1
#endif

#ifndef HAL_BATTMON_FUEL_ENABLE
#define HAL_BATTMON_FUEL_ENABLE 1
#endif

// declare backend class
class AP_BattMonitor_Backend;
class AP_BattMonitor_Analog;
class AP_BattMonitor_SMBus;
class AP_BattMonitor_SMBus_Solo;
class AP_BattMonitor_SMBus_Generic;
class AP_BattMonitor_SMBus_Maxell;
class AP_BattMonitor_SMBus_Rotoye;
class AP_BattMonitor_UAVCAN;
class AP_BattMonitor_Generator;
class AP_BattMonitor_MPPT_PacketDigital;

class AP_BattMonitor
{
    friend class AP_BattMonitor_Backend;
    friend class AP_BattMonitor_Analog;
    friend class AP_BattMonitor_SMBus;
    friend class AP_BattMonitor_SMBus_Solo;
    friend class AP_BattMonitor_SMBus_Generic;
    friend class AP_BattMonitor_SMBus_Maxell;
    friend class AP_BattMonitor_SMBus_Rotoye;
    friend class AP_BattMonitor_UAVCAN;
    friend class AP_BattMonitor_Sum;
    friend class AP_BattMonitor_FuelFlow;
    friend class AP_BattMonitor_FuelLevel_PWM;
    friend class AP_BattMonitor_Generator;
    friend class AP_BattMonitor_MPPT_PacketDigital;

public:

    // battery failsafes must be defined in levels of severity so that vehicles wont fall backwards
    enum class Failsafe : uint8_t {
        None = 0,
        Low,
        Critical
    };

    // Battery monitor driver types 电池监视器驱动程序类型
    enum class Type {
        NONE                       = 0,
        ANALOG_VOLTAGE_ONLY        = 3,
        ANALOG_VOLTAGE_AND_CURRENT = 4,
        SOLO                       = 5,
        BEBOP                      = 6,
        SMBus_Generic              = 7,
        UAVCAN_BatteryInfo         = 8,
        BLHeliESC                  = 9,
        Sum                        = 10,
        FuelFlow                   = 11,
        FuelLevel_PWM              = 12,
        SUI3                       = 13,
        SUI6                       = 14,
        NeoDesign                  = 15,
        MAXELL                     = 16,
        GENERATOR_ELEC             = 17,
        GENERATOR_FUEL             = 18,
        Rotoye                     = 19,
        MPPT_PacketDigital         = 20,
    };

    FUNCTOR_TYPEDEF(battery_failsafe_handler_fn_t, void, const char *, const int8_t);

    AP_BattMonitor(uint32_t log_battery_bit, battery_failsafe_handler_fn_t battery_failsafe_handler_fn, const int8_t *failsafe_priorities);

    /* Do not allow copies */
    AP_BattMonitor(const AP_BattMonitor &other) = delete;
    AP_BattMonitor &operator=(const AP_BattMonitor&) = delete;

    static AP_BattMonitor *get_singleton() {
        return _singleton;
    }

    // cell voltages in millivolts 电池电压（毫伏）
    struct cells {
        uint16_t cells[AP_BATT_MONITOR_CELLS_MAX];
    };

    // The BattMonitor_State structure is filled in by the backend driver   BattMonitor_State 结构由后端驱动程序填充
    struct BattMonitor_State {
        cells       cell_voltages;             // battery cell voltages in millivolts, 10 cells matches the MAVLink spec 电池电压单位为毫伏，10片电池符合MAVLink规范
        float       voltage;                   // voltage in volts 电压单位：伏特
        float       current_amps;              // current in amperes 以安培为单位的电流
        float       consumed_mah;              // total current draw in milliamp hours since start-up 启动后的总电流消耗（毫安时）
        float       consumed_wh;               // total energy consumed in Wh since start-up 自启动以来消耗的总能量（单位：Wh）
        uint32_t    last_time_micros;          // time when voltage and current was last read in microseconds 上次读取电压和电流的时间（微秒）
        uint32_t    low_voltage_start_ms;      // time when voltage dropped below the minimum in milliseconds 电压降至最低值以下的时间（毫秒）
        uint32_t    critical_voltage_start_ms; // critical voltage failsafe start timer in milliseconds 临界电压故障保护启动计时器（毫秒）
        float       temperature;               // battery temperature in degrees Celsius 电池温度（摄氏度）
        uint32_t    temperature_time;          // timestamp of the last received temperature message 上次接收到的温度消息的时间戳
        float       voltage_resting_estimate;  // voltage with sag removed based on current and resistance estimate in Volt 根据电流和电阻估计值（以伏特为单位）消除拉低的电压
        float       resistance;                // resistance, in Ohms, calculated by comparing resting voltage vs in flight voltage 通过比较静止电压和飞行电压计算得出的电阻，单位为欧姆
        Failsafe    failsafe;                  // stage failsafe the battery is in 电池处于阶段故障保护状态
        bool        healthy;                   // battery monitor is communicating correctly 电池监视器通讯正常
        bool        is_powering_off;           // true when power button commands power off 当电源按钮命令关闭电源时为true
        bool        powerOffNotified;          // only send powering off notification once 只发送一次断电通知
    };

    // Return the number of battery monitor instances 返回电池监视器实例数
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available battery monitors 检测并初始化任何可用的电池监测器
    void init();

    /// Read the battery voltage and current for all batteries.  Should be called at 10hz 读取所有电池的电池电压和电流。应在10hz时调用
    void read();

    // healthy - returns true if monitor is functioning 健康状况-如果监视器正常工作，则返回true
    bool healthy(uint8_t instance) const;
    bool healthy() const { return healthy(AP_BATT_PRIMARY_INSTANCE); }

    /// voltage - returns battery voltage in volts 电压-返回以伏特为单位的蓄电池电压
    float voltage(uint8_t instance) const;
    float voltage() const { return voltage(AP_BATT_PRIMARY_INSTANCE); }

    /// get voltage with sag removed (based on battery current draw and resistance) 消除压降后获得电压（基于蓄电池电流消耗和电阻）
    /// this will always be greater than or equal to the raw voltage 这将始终大于或等于原始电压
    float voltage_resting_estimate(uint8_t instance) const;
    float voltage_resting_estimate() const { return voltage_resting_estimate(AP_BATT_PRIMARY_INSTANCE); }

    /// current_amps - returns the instantaneous current draw in amperes 返回以安培为单位的瞬时电流消耗
    bool current_amps(float &current, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /// consumed_mah - returns total current drawn since start-up in milliampere.hours 消耗的毫安时-返回自启动以来消耗的总电量，单位为毫安时
    bool consumed_mah(float &mah, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /// consumed_wh - returns total energy drawn since start-up in watt.hours 返回自启动以来消耗的总能量，单位为瓦时
    bool consumed_wh(float&wh, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100) 返回剩余电池容量的百分比（0~100）
    virtual uint8_t capacity_remaining_pct(uint8_t instance) const;
    uint8_t capacity_remaining_pct() const { return capacity_remaining_pct(AP_BATT_PRIMARY_INSTANCE); }

    /// pack_capacity_mah - returns the capacity of the battery pack in mAh when the pack is full 当电池组已满时，返回电池组的容量（单位：mah）
    int32_t pack_capacity_mah(uint8_t instance) const;
    int32_t pack_capacity_mah() const { return pack_capacity_mah(AP_BATT_PRIMARY_INSTANCE); }
 
    /// returns true if a battery failsafe has ever been triggered 如果曾经触发过电池故障保护，则返回true
    bool has_failsafed(void) const { return _has_triggered_failsafe; };

    /// returns the highest failsafe action that has been triggered 返回已触发的最高故障保护操作
    int8_t get_highest_failsafe_priority(void) const { return _highest_failsafe_priority; };

    /// get_type - returns battery monitor type 获取类型-返回电池监视器类型
    enum Type get_type() const { return get_type(AP_BATT_PRIMARY_INSTANCE); }
    enum Type get_type(uint8_t instance) const {
        return (Type)_params[instance]._type.get();
    }

    /// get_serial_number - returns battery serial number 获取序列号-返回电池序列号
    int32_t get_serial_number() const { return get_serial_number(AP_BATT_PRIMARY_INSTANCE); }
    int32_t get_serial_number(uint8_t instance) const {
        return _params[instance]._serial_number;
    }

    /// true when (voltage * current) > watt_max 当（电压*电流）>最大功率时为真
    bool overpower_detected() const;
    bool overpower_detected(uint8_t instance) const;

    // cell voltages in millivolts 电池电压（毫伏）
    bool has_cell_voltages() const { return has_cell_voltages(AP_BATT_PRIMARY_INSTANCE); }
    bool has_cell_voltages(const uint8_t instance) const;
    const cells &get_cell_voltages() const { return get_cell_voltages(AP_BATT_PRIMARY_INSTANCE); }
    const cells &get_cell_voltages(const uint8_t instance) const;

    // temperature 温度
    bool get_temperature(float &temperature) const { return get_temperature(temperature, AP_BATT_PRIMARY_INSTANCE); }
    bool get_temperature(float &temperature, const uint8_t instance) const;

    // cycle count 电池充放数
    bool get_cycle_count(uint8_t instance, uint16_t &cycles) const;

    // get battery resistance estimate in ohms 获取以欧姆为单位的电池电阻估计值
    float get_resistance() const { return get_resistance(AP_BATT_PRIMARY_INSTANCE); }
    float get_resistance(uint8_t instance) const { return state[instance].resistance; }

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message 如果解锁检查失败，则返回false，在这种情况下，缓冲区将填充失败消息
    bool arming_checks(size_t buflen, char *buffer) const;

    // sends powering off mavlink broadcasts and sets notify flag 发送关闭mavlink广播并设置notify标志
    void checkPoweringOff(void);

    // reset battery remaining percentage 重置电池剩余百分比
    bool reset_remaining_mask(uint16_t battery_mask, float percentage);
    bool reset_remaining(uint8_t instance, float percentage) { return reset_remaining_mask(1U<<instance, percentage);}

    // Returns mavlink charge state 返回链路充电状态
    MAV_BATTERY_CHARGE_STATE get_mavlink_charge_state(const uint8_t instance) const;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// parameters
    AP_BattMonitor_Params _params[AP_BATT_MONITOR_MAX_INSTANCES];

private:
    static AP_BattMonitor *_singleton;

    BattMonitor_State state[AP_BATT_MONITOR_MAX_INSTANCES];
    AP_BattMonitor_Backend *drivers[AP_BATT_MONITOR_MAX_INSTANCES];
    uint32_t    _log_battery_bit;
    uint8_t     _num_instances;                                     /// number of monitors

    void convert_params(void);

    /// returns the failsafe state of the battery
    Failsafe check_failsafe(const uint8_t instance);
    void check_failsafes(void); // checks all batteries failsafes

    battery_failsafe_handler_fn_t _battery_failsafe_handler_fn;
    const int8_t *_failsafe_priorities; // array of failsafe priorities, sorted highest to lowest priority, -1 indicates no more entries
                                        // 故障保护优先级数组，从最高优先级到最低优先级排序，-1表示没有更多条目
    int8_t      _highest_failsafe_priority; // highest selected failsafe action level (used to restrict what actions we move into)
                                            // 选择的最高故障保护操作级别（用于限制我们采取的操作）
    bool        _has_triggered_failsafe;  // true after a battery failsafe has been triggered for the first time
                                          // 首次触发电池故障保护后为真

};

namespace AP {
    AP_BattMonitor &battery();
};
