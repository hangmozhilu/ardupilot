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

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Beacon_Nooploop.h"
#include <ctype.h>
#include <stdio.h>

#define NOOPLOOP_INVALID_VAL -8388000 // indicates data unavailable or invalid 表示数据不可用或无效
#define NOOPLOOP_SF0_SZ 128 // setting_frame0 packet size 设置frame0的报文大小

#define NOOPLOOP_HEADER                     0x55    // message header 报文帧头
#define NOOPLOOP_FUNCTION_MARK_NODE_FRAME2  4
#define NOOPLOOP_NODE_FRAME2_FRAMELEN_MAX   4096    // frames should be less than 4k bytes
#define NOOPLOOP_NODE_FRAME2_SYSTIME        6       // start of 4 bytes holding system time in ms 以毫秒为单位保存系统时间的4字节的起始
#define NOOPLOOP_NODE_FRAME2_PRECISION_X    10      // start of 1 byte holding precision in m*100 in x axis  x轴精度位
#define NOOPLOOP_NODE_FRAME2_PRECISION_Y    11      // start of 1 byte holding precision in m*100 in y axis  y轴精度位
#define NOOPLOOP_NODE_FRAME2_PRECISION_Z    12      // start of 1 byte holding precision in m*100 in y axis  Z轴精度位
#define NOOPLOOP_NODE_FRAME2_POSX           13      // start of 3 bytes holding x position in m*1000
#define NOOPLOOP_NODE_FRAME2_POSY           16      // start of 3 bytes holding y position in m*1000
#define NOOPLOOP_NODE_FRAME2_POSZ           19      // start of 3 bytes holding z position in m*1000
#define NOOPLOOP_NODE_FRAME2_VALID_NODES    118
#define NOOPLOOP_NODE_FRAME2_NODE_BLOCK     119

#define NOOPLOOP_HEADER2                    0x54    // message header 报文帧头
#define NOOPLOOP_FUNCTION_MARK_SETTING_FRAME0  0
#define NOOPLOOP_SETTING_FRAME0_A0          37


extern const AP_HAL::HAL& hal;

AP_Beacon_Nooploop::AP_Beacon_Nooploop(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);
    if (_uart != nullptr) {
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Beacon, 0));
        //gcs().send_text(MAV_SEVERITY_INFO, "!!!!! _uart != nullptr !!!!!");//如果串口有数据，打印_uart != nullptr
    }
    else
    {
       // gcs().send_text(MAV_SEVERITY_INFO, "!!!!! _uart = nullptr !!!!!");//如果串口没有数据，打印_uart = nullptr
    }
    
}

// return true if sensor is basically healthy (we are receiving data) 如果传感器基本正常则返回true(我们正在接收数据)
bool AP_Beacon_Nooploop::healthy()
{
    //gcs().send_text(MAV_SEVERITY_INFO, "!!!!! AP_Beacon_Nooploop::healthy !!!!!");
    // healthy if we have parsed a message within the past 300ms 如果我们在过去300毫秒内解析了一条消息，则是正常的
    return ((AP_HAL::millis() - _last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor 更新传感器状态
void AP_Beacon_Nooploop::update(void)
{
    // return immediately if not serial port 如果没有找到串口，则立即返回
    if (_uart == nullptr) {
        return;
        //gcs().send_text(MAV_SEVERITY_INFO, "!!!AP_Beacon_Nooploop::update _uart == nullptr!!!"); //如果串口没有数据，打印 _uart == nullptr
    }
    //gcs().send_text(MAV_SEVERITY_INFO, "!!!AP_Beacon_Nooploop::update _uart != nullptr!!!"); //如果串口没有数据，打印 _uart != nullptr

    // check uart for any incoming messages 检查uart是否有任何传入的消息
    uint32_t nbytes = MIN(_uart->available(), 1024U);
    while (nbytes-- > 0) {
        int16_t b = _uart->read();
        if (b >= 0 ) {
            MsgType type = parse_byte((uint8_t)b);
            if (type == MsgType::NODE_FRAME2) {
                if (_anchor_pos_avail) {   //如果获得了固定基站的位置，就执行相关函数
                //if (1) {
                    //gcs().send_text(MAV_SEVERITY_INFO, "!!AP_Beacon_Nooploop::update parse_node_frame2()!!");
                    parse_node_frame2();   
                } else if (AP_HAL::millis() - _last_request_setting_ms > 2000) {
                    _last_request_setting_ms = AP_HAL::millis();
                    //gcs().send_text(MAV_SEVERITY_INFO, "!!AP_Beacon_Nooploop::update request_setting()!!");
                    request_setting();     //超时没有收到固定基站位置数据，设置一些参数
                }
            } else if (type == MsgType::SETTING_FRAME0) {
                parse_setting_frame0();
            } 
        }
    }
}

void AP_Beacon_Nooploop::request_setting()
{
    //send setting_frame0 to tag, tag will fill anchor position and ack 发送设置frame0到标签，标签将填充锚位和ack
    _uart->write((uint8_t)0x54);
    _uart->write((uint8_t)0);
    _uart->write((uint8_t)1);
    for (uint8_t i = 0; i < 124; i++) {
        _uart->write((uint8_t)0); //manual states filled with any char, but in fact only 0 works 手动状态填充任何char，但实际上只有0起作用
    }
    _uart->write((uint8_t)0x55);
}

// process one byte received on serial port 处理串口接收到的一个字节
// message is stored in _msgbuf 消息存储在msgbuf中
AP_Beacon_Nooploop::MsgType AP_Beacon_Nooploop::parse_byte(uint8_t b)
{
    // process byte depending upon current state 进程字节取决于当前状态
    switch (_state) {

    case ParseState::HEADER:
        if (b == NOOPLOOP_HEADER) {
            _msgbuf[0] = b;
            _msg_len = 1;
            _crc_expected = b;
            _state = ParseState::H55_FUNCTION_MARK;
        } else if (b == NOOPLOOP_HEADER2) {
            _msgbuf[0] = b;
            _msg_len = 1;
            _crc_expected = b;
            _state = ParseState::H54_FUNCTION_MARK;
        }
        break;

    case ParseState::H55_FUNCTION_MARK:
        if (b == NOOPLOOP_FUNCTION_MARK_NODE_FRAME2) {
            _msgbuf[1] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::LEN_L;
        } else {
            _state = ParseState::HEADER;
        }
        break;

    case ParseState::H54_FUNCTION_MARK:
        if (b == NOOPLOOP_FUNCTION_MARK_SETTING_FRAME0) {
            _msgbuf[1] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::SF0_PAYLOAD;
        } else {
            _state = ParseState::HEADER;
        }
        break;

    case ParseState::LEN_L:
        _msgbuf[2] = b;
        _msg_len++;
        _crc_expected += b;
        _state = ParseState::LEN_H;
        break;

    case ParseState::LEN_H:
        // extract and sanity check frame length
        _frame_len = UINT16_VALUE(b, _msgbuf[2]);
        if (_frame_len > NOOPLOOP_NODE_FRAME2_FRAMELEN_MAX) {
            _state = ParseState::HEADER;
        } else {
            _msgbuf[3] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::NF2_PAYLOAD;
        }
        break;

    case ParseState::NF2_PAYLOAD:
        // add byte to buffer if there is room
        if (_msg_len < NOOPLOOP_MSG_BUF_MAX) {
            _msgbuf[_msg_len] = b;
        }
        _msg_len++;
        if (_msg_len >= _frame_len) {
            _state = ParseState::HEADER;
            // check crc
            if (b == _crc_expected) {
                return MsgType::NODE_FRAME2;
            }
        } else {
            _crc_expected += b;
        }
        break;

    case ParseState::SF0_PAYLOAD:
        // add byte to buffer if there is room
        if (_msg_len < NOOPLOOP_MSG_BUF_MAX) {
            _msgbuf[_msg_len] = b;
        }
        _msg_len++;
        if (_msg_len >= NOOPLOOP_SF0_SZ) {
            _state = ParseState::HEADER;
            // check crc
            if (b == _crc_expected) {
                return MsgType::SETTING_FRAME0;
            }
        } else {
            _crc_expected += b;
        }
        break;
    }

    return MsgType::INVALID;
}

void AP_Beacon_Nooploop::parse_node_frame2()
{
    // a message has been received 已收到消息
    _last_update_ms = AP_HAL::millis();

    // estimated precision for x,y position in meters x,y位置的估计精度，单位为米
    const float precision_x = _msgbuf[NOOPLOOP_NODE_FRAME2_PRECISION_X] * 0.01;
    const float precision_y = _msgbuf[NOOPLOOP_NODE_FRAME2_PRECISION_Y] * 0.01;
    //EKF's estimate goes very bad if the error value sent into the EKF is unrealistically low. ensure it's never less than a reasonable value
    //如果发送到EKF的误差值低得不现实，EKF的估计就会变得非常糟糕。确保它永远不会低于一个合理的值
    const float pos_err = MAX(0.1f, sqrtf(sq(precision_x)+sq(precision_y)));

    // x,y,z position in m*1000 in ENU frame ENU坐标系中m*1000的x,y,z位置。特别的，对于 int24 类型，我们需要先转换成 int32 类型，为了保持符号不变，采用左移后 除以256方式 或者 >> 8
    const int32_t pos_x = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX] << 8) >> 8;
    const int32_t pos_y = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY] << 8) >> 8;
    const int32_t pos_z = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ] << 8) >> 8;

    // gcs().send_text(MAV_SEVERITY_INFO, "!!!!! Nooploop X: %ld !!!!!", pos_x);  // debug,如果信标功能正常就输出坐标
    // gcs().send_text(MAV_SEVERITY_INFO, "!!!!! Nooploop Y: %ld !!!!!", pos_y);  // debug,如果信标功能正常就输出坐标
    // gcs().send_text(MAV_SEVERITY_INFO, "!!!!! Nooploop Z: %ld !!!!!", pos_z);  // debug,如果信标功能正常就输出坐标

    // !!!!! UWB的坐标系为ENU坐标系，飞机飞行的坐标系是NED坐标，那就需要将UWB的位置数据X和Y数值颠倒，Z数值取负值，这样就把UWB的数据转换到NED的坐标系下
    // position scaled to meters and changed to NED 位置缩放到米，并更改为NED
    const Vector3f pos_m {pos_y * 0.001f, pos_x * 0.001f, -pos_z * 0.001f};

    // gcs().send_text(MAV_SEVERITY_INFO, "!!!!! vehicle X: %lf !!!!!", pos_m[0]);  // debug,如果信标功能正常就输出坐标
    // gcs().send_text(MAV_SEVERITY_INFO, "!!!!! vehicle Y: %lf !!!!!", pos_m[1]);  // debug,如果信标功能正常就输出坐标
    // gcs().send_text(MAV_SEVERITY_INFO, "!!!!! vehicle Z: %lf !!!!!", pos_m[2]);  // debug,如果信标功能正常就输出坐标

    set_vehicle_position(pos_m, pos_err); //设置载具的位置

    const uint8_t valid_nodes = _msgbuf[NOOPLOOP_NODE_FRAME2_VALID_NODES];
    for (uint8_t i = 0; i < valid_nodes; i++) {
        uint16_t offset = NOOPLOOP_NODE_FRAME2_NODE_BLOCK + i * 13;
        uint8_t id = _msgbuf[offset+1]; //nooploop id starts from 0, increments clockwise, 0 -> 1 define Y axis. noloop id从0开始，顺时针递增，0 -> 1定义Y轴。
        const int32_t dist = ((int32_t)_msgbuf[offset+2+2] << 24 | (int32_t)_msgbuf[offset+2+1] << 16 | (int32_t)_msgbuf[offset+2] << 8) >> 8;
        set_beacon_distance(id, dist * 0.001f);  //计算固定基站的距离
    }
}

void AP_Beacon_Nooploop::parse_setting_frame0()
{
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t offset = NOOPLOOP_SETTING_FRAME0_A0 + i * 9;

        // x,y,z position in m*1000 in ENU frame
        const int32_t pos_x = ((int32_t)_msgbuf[offset+2] << 24 | (int32_t)_msgbuf[offset+1] << 16 | (int32_t)_msgbuf[offset] << 8) >> 8;
        if (pos_x == NOOPLOOP_INVALID_VAL) { //anchor position not available
            return;
        }
        offset+=3;
        const int32_t pos_y = ((int32_t)_msgbuf[offset+2] << 24 | (int32_t)_msgbuf[offset+1] << 16 | (int32_t)_msgbuf[offset] << 8) >> 8;
        if (pos_y == NOOPLOOP_INVALID_VAL) {
            return;
        }
        offset+=3;
        const int32_t pos_z = ((int32_t)_msgbuf[offset+2] << 24 | (int32_t)_msgbuf[offset+1] << 16 | (int32_t)_msgbuf[offset] << 8) >> 8;
        if (pos_z == NOOPLOOP_INVALID_VAL) {
            return;
        }

        // position scaled to meters and changed to NED 位置缩放到米，并更改为NED坐标系
        const Vector3f pos_m {pos_y * 0.001f, pos_x * 0.001f, -pos_z * 0.001f};
        
        set_beacon_position(i, pos_m);
    }
    _anchor_pos_avail = true;
}
