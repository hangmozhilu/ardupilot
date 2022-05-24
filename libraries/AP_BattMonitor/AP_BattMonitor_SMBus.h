#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#define AP_BATTMONITOR_SMBUS_BUS_INTERNAL           0
#define AP_BATTMONITOR_SMBUS_BUS_EXTERNAL           1
#define AP_BATTMONITOR_SMBUS_I2C_ADDR               0x0B
#define AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds
                                                            // 如果5秒钟内没有成功读数，认为传感器不健康
class AP_BattMonitor_SMBus : public AP_BattMonitor_Backend
{
public:

    // Smart Battery Data Specification Revision 1.1 智能电池数据规范第1.1版
    enum BATTMONITOR_SMBUS {
        BATTMONITOR_SMBUS_TEMP = 0x08,                 // Temperature 温度
        BATTMONITOR_SMBUS_VOLTAGE = 0x09,              // Voltage 电压
        BATTMONITOR_SMBUS_CURRENT = 0x0A,              // Current 电流
        BATTMONITOR_SMBUS_REMAINING_CAPACITY = 0x0F,   // Remaining Capacity 剩余容量
        BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY = 0x10, // Full Charge Capacity 全充电容量
        BATTMONITOR_SMBUS_CYCLE_COUNT = 0x17,          // Cycle Count 充放次数
        BATTMONITOR_SMBUS_SPECIFICATION_INFO = 0x1A,   // Specification Info 规格信息
        BATTMONITOR_SMBUS_SERIAL = 0x1C,               // Serial Number 序列号
        BATTMONITOR_SMBUS_MANUFACTURE_NAME = 0x20,     // Manufacture Name 序列号
        BATTMONITOR_SMBUS_MANUFACTURE_DATA = 0x23,     // Manufacture Data 制造数据
    };

    /// Constructor
    AP_BattMonitor_SMBus(AP_BattMonitor &mon,
                    AP_BattMonitor::BattMonitor_State &mon_state,
                    AP_BattMonitor_Params &params,
                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // virtual destructor to reduce compiler warnings
    virtual ~AP_BattMonitor_SMBus() {}

    bool has_cell_voltages() const override { return _has_cell_voltages; }

    bool has_temperature() const override { return _has_temperature; }

    // all smart batteries are expected to provide current 所有智能电池都将提供电流
    bool has_current() const override { return true; }

    // don't allow reset of remaining capacity for SMBus  不允许重置SMBus的剩余容量
    bool reset_remaining(float percentage) override { return false; }

    // return true if cycle count can be provided and fills in cycles argument 如果可以提供充放数并填写cycles参数，则返回true
    bool get_cycle_count(uint16_t &cycles) const override;

    virtual void init(void) override;

protected:

    void read(void) override;

    // reads the pack full charge capacity
    // returns true if the read was successful, or if we already knew the pack capacity
    bool read_full_charge_capacity(void);

    // reads the remaining capacity
    // returns true if the read was successful, which is only considered to be the
    // we know the full charge capacity
    bool read_remaining_capacity(void);

    // return a scaler that should be multiplied by the battery's reported capacity numbers to arrive at the actual capacity in mAh
    virtual uint16_t get_capacity_scaler() const { return 1; }

    // reads the temperature word from the battery
    // returns true if the read was successful
    virtual bool read_temp(void);

    // reads the serial number if it's not already known
    // returns true if the read was successful, or the number was already known
    bool read_serial_number(void);

    // reads the battery's cycle count
    void read_cycle_count();

     // read word from register
     // returns true if read was successful, false if failed
    bool read_word(uint8_t reg, uint16_t& data) const;

    // get_PEC - calculate PEC for a read or write from the battery
    // buff is the data that was read or will be written
    uint8_t get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool _pec_supported; // true if PEC is supported

    int32_t _serial_number = -1;    // battery serial number 电池序列号
    uint16_t _full_charge_capacity; // full charge capacity, used to stash the value before setting the parameter 全充电容量，用于在设置参数前储存数值
    bool _has_cell_voltages;        // smbus backends flag this as true once they have received a valid cell voltage report smbus后端在收到有效的单元电压报告后将其标记为true
    uint16_t _cycle_count = 0;      // number of cycles the battery has experienced. An amount of discharge approximately equal to the value of DesignCapacity. 电池经历的循环次数。大约等于设计容量值的排放量
    bool _has_cycle_count;          // true if cycle count has been retrieved from the battery 如果已从电池中检索到循环计数，则为true
    bool _has_temperature;          // 是否有电池电压

    virtual void timer(void) = 0;   // timer function to read from the battery

    AP_HAL::Device::PeriodicHandle timer_handle;
};
