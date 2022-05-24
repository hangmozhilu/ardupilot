#include "AP_BattMonitor_SMBus.h"

#define AP_BATTMONITOR_SMBUS_PEC_POLYNOME 0x07 // Polynome for CRC generation

AP_BattMonitor_SMBus::AP_BattMonitor_SMBus(AP_BattMonitor &mon,
                                           AP_BattMonitor::BattMonitor_State &mon_state,
                                           AP_BattMonitor_Params &params,
                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
        : AP_BattMonitor_Backend(mon, mon_state, params),
        _dev(std::move(dev))
{
    _params._serial_number = AP_BATT_SERIAL_NUMBER_DEFAULT;
    _params._pack_capacity = 0;
}

void AP_BattMonitor_SMBus::init(void)
{
    if (_dev) {
        timer_handle = _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus::timer, void));
    }
}

// return true if cycle count can be provided and fills in cycles argument 如果可以提供循环计数并填写cycles参数，则返回true
bool AP_BattMonitor_SMBus::get_cycle_count(uint16_t &cycles) const
{
    if (!_has_cycle_count) {
        return false;
    }
    cycles = _cycle_count;
    return true;
}

/// read the battery_voltage and current, should be called at 10hz 读取电池的电压和电流，应以10hz的频率调用
void AP_BattMonitor_SMBus::read(void)
{
    // nothing to be done here for actually interacting with the battery 为了与电池进行真正的互动，这里什么都不用做
    // however we can use this to set any parameters that need to be set 然而，我们可以使用它来设置任何需要设置的参数

    if (_serial_number != _params._serial_number) {
        _params._serial_number.set_and_notify(_serial_number);
    }

    if (_full_charge_capacity != _params._pack_capacity) {
        _params._pack_capacity.set_and_notify(_full_charge_capacity);
    }
}

// reads the pack full charge capacity 如果已从电池中检索到循环计数，则为true
// returns true if the read was successful, or if we already knew the pack capacity 如果读取成功，或者我们已经知道包容量，则返回true
bool AP_BattMonitor_SMBus::read_full_charge_capacity(void)
{
    uint16_t data;

    if (_full_charge_capacity != 0) {
        return true;
    } else if (read_word(BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY, data)) {
        _full_charge_capacity = data * get_capacity_scaler();
        return true;
    }
    return false;
}

// reads the remaining capacity 读取剩余容量
// returns true if the read was successful, which is only considered to be the 如果读取成功，则返回true，这仅被视为
// we know the full charge capacity 我们知道全充电容量
bool AP_BattMonitor_SMBus::read_remaining_capacity(void)
{
    int32_t capacity = _params._pack_capacity;

    if (capacity > 0) {
        uint16_t data;
        if (read_word(BATTMONITOR_SMBUS_REMAINING_CAPACITY, data)) {
            _state.consumed_mah = MAX(0, capacity - (data * get_capacity_scaler()));
            return true;
        }
    }

    return false;
}

// reads the temperature word from the battery
// returns true if the read was successful
bool AP_BattMonitor_SMBus::read_temp(void)
{
    uint16_t data;
    if (read_word(BATTMONITOR_SMBUS_TEMP, data)) {
        _has_temperature = 1;//_has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;

        _state.temperature_time = AP_HAL::millis();
        _state.temperature = 33.3f;//_state.temperature = ((float)(data - 2731)) * 0.1f;
        return true;
    }
    
    _has_temperature = false;

    return false;
}

// reads the serial number if it's not already known
// returns true if the read was successful or the number was already known
bool AP_BattMonitor_SMBus::read_serial_number(void)
{
    uint16_t data;

    // don't recheck the serial number if we already have it
    if (_serial_number != -1) {
        return true;
    } else if (read_word(BATTMONITOR_SMBUS_SERIAL, data)) {
        _serial_number = data;
        return true;
    }

    return false;
}

// reads the battery's cycle count
void AP_BattMonitor_SMBus::read_cycle_count()
{
    // only read cycle count once
    if (_has_cycle_count) {
        return;
    }
    _has_cycle_count = read_word(BATTMONITOR_SMBUS_CYCLE_COUNT, _cycle_count);
}

// read word from register
// returns true if read was successful, false if failed
bool AP_BattMonitor_SMBus::read_word(uint8_t reg, uint16_t& data) const
{
    // buffer to hold results (1 extra byte returned holding PEC) 缓冲区保留结果（保留PEC时返回1个额外字节）
    const uint8_t read_size = 2 + (_pec_supported ? 1 : 0);
    uint8_t buff[read_size];    // buffer to hold results 保存结果的缓冲区

    // read the appropriate register from the device 从设备中读取相应的寄存器
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return false;
    }

    // check PEC 检查PEC
    if (_pec_supported) {
        const uint8_t pec = get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, 2);
        if (pec != buff[2]) {
            return false;
        }
    }

    // convert buffer to word 将缓冲区转换为字
    data = (uint16_t)buff[1]<<8 | (uint16_t)buff[0];

    // return success
    return true;
}

/// get_PEC - calculate packet error correction code of buffer 计算缓冲区的数据包纠错码
uint8_t AP_BattMonitor_SMBus::get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const
{
    // exit immediately if no data 如果没有数据，请立即退出
    if (len == 0) {
        return 0;
    }

    // prepare temp buffer for calculating crc 为计算crc准备临时缓冲区
    uint8_t tmp_buff[len+3];
    tmp_buff[0] = i2c_addr << 1;
    tmp_buff[1] = cmd;
    tmp_buff[2] = tmp_buff[0] | (uint8_t)reading;
    memcpy(&tmp_buff[3],buff,len);

    // initialise crc to zero 将crc初始化为零
    uint8_t crc = 0;
    uint8_t shift_reg = 0;
    bool do_invert;

    // for each byte in the stream 对于流中的每个字节
    for (uint8_t i=0; i<sizeof(tmp_buff); i++) {
        // load next data byte into the shift register 将下一个数据字节加载到移位寄存器中
        shift_reg = tmp_buff[i];
        // for each bit in the current byte 对于当前字节中的每一位
        for (uint8_t j=0; j<8; j++) {
            do_invert = (crc ^ shift_reg) & 0x80;
            crc <<= 1;
            shift_reg <<= 1;
            if(do_invert) {
                crc ^= AP_BATTMONITOR_SMBUS_PEC_POLYNOME;
            }
        }
    }

    // return result
    return crc;
}

