// Copyright (c) 2020 Monika Roznere, RLab @ Dartmouth College
// Copyright (c) 2020 Blue Robotics

// C++ implementation of the Blue Robotics 'Ping' binary message protocol

#ifndef PING_NODELET_PING_MESSAGE_PING360_H
#define PING_NODELET_PING_MESSAGE_PING360_H

#pragma once

#include "ping_nodelet/ping-message.h"

namespace ping_nodelet
{

// TODO(bluerobotics) should maybe be an enum
namespace Ping360Id
{
    static const uint16_t DEVICE_DATA = 2300;
    static const uint16_t MOTOR_OFF = 2903;
    static const uint16_t RESET = 2600;
    static const uint16_t TRANSDUCER = 2601;
    static const uint16_t AUTO_TRANSMIT = 2602;
    static const uint16_t DEVICE_ID = 2000;
}

class ping360_device_data : public ping_message
{
public:
    explicit ping360_device_data(const ping_message& msg) : ping_message { msg } {}
    ping360_device_data(const uint8_t* buf, const uint16_t length) : ping_message { buf, length } {}
    ping360_device_data(uint16_t data_length
) : ping_message { static_cast<uint16_t>(24 + data_length) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = static_cast<uint16_t>(14 + data_length);  // payload size
        (uint16_t&)msgData[4] = 2300;  // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t mode() const { return (uint8_t&)msgData[headerLength + 0]; }
    void set_mode(const uint8_t mode) { (uint8_t&)msgData[headerLength + 0] = mode; }
    uint8_t gain_setting() const { return (uint8_t&)msgData[headerLength + 1]; }
    void set_gain_setting(const uint8_t gain_setting) { (uint8_t&)msgData[headerLength + 1] = gain_setting; }
    uint16_t angle() const { return (uint16_t&)msgData[headerLength + 2]; }
    void set_angle(const uint16_t angle) { (uint16_t&)msgData[headerLength + 2] = angle; }
    uint16_t transmit_duration() const { return (uint16_t&)msgData[headerLength + 4]; }
    void set_transmit_duration(const uint16_t transmit_duration)
    {
        (uint16_t&)msgData[headerLength + 4] = transmit_duration;
    }
    uint16_t sample_period() const { return (uint16_t&)msgData[headerLength + 6]; }
    void set_sample_period(const uint16_t sample_period) { (uint16_t&)msgData[headerLength + 6] = sample_period; }
    uint16_t transmit_frequency() const { return (uint16_t&)msgData[headerLength + 8]; }
    void set_transmit_frequency(const uint16_t transmit_frequency)
    {
        (uint16_t&)msgData[headerLength + 8] = transmit_frequency;
    }
    uint16_t number_of_samples() const { return (uint16_t&)msgData[headerLength + 10]; }
    void set_number_of_samples(const uint16_t number_of_samples)
    {
        (uint16_t&)msgData[headerLength + 10] = number_of_samples;
    }
    uint16_t data_length() const { return (uint16_t&)msgData[headerLength + 12]; }
    // TODO(bluerobotics) do this in constructor (const)
    void set_data_length(const uint16_t data_length) { (uint16_t&)msgData[headerLength + 12] = data_length;}
    uint8_t* data() const { return reinterpret_cast<uint8_t*>(msgData+headerLength+14); }
    void set_data_at(const uint16_t i, const uint8_t data) { (uint8_t&)msgData[headerLength + 14 + i] = data; }
};

class ping360_motor_off : public ping_message
{
public:
    explicit ping360_motor_off(const ping_message& msg) : ping_message { msg } {}
    ping360_motor_off(const uint8_t* buf, const uint16_t length) : ping_message { buf, length } {}
    ping360_motor_off() : ping_message { static_cast<uint16_t>(10) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = static_cast<uint16_t>(0);  // payload size
        (uint16_t&)msgData[4] = 2903;  // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }
};

class ping360_reset : public ping_message
{
public:
    explicit ping360_reset(const ping_message& msg) : ping_message { msg } {}
    ping360_reset(const uint8_t* buf, const uint16_t length) : ping_message { buf, length } {}
    ping360_reset() : ping_message { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = static_cast<uint16_t>(2);  // payload size
        (uint16_t&)msgData[4] = 2600;  // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t bootloader() const { return (uint8_t&)msgData[headerLength + 0]; }
    void set_bootloader(const uint8_t bootloader) { (uint8_t&)msgData[headerLength + 0] = bootloader; }
    uint8_t reserved() const { return (uint8_t&)msgData[headerLength + 1]; }
    void set_reserved(const uint8_t reserved) { (uint8_t&)msgData[headerLength + 1] = reserved; }
};

class ping360_transducer : public ping_message
{
public:
    explicit ping360_transducer(const ping_message& msg) : ping_message { msg } {}
    ping360_transducer(const uint8_t* buf, const uint16_t length) : ping_message { buf, length } {}
    ping360_transducer() : ping_message { static_cast<uint16_t>(24) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = static_cast<uint16_t>(14);  // payload size
        (uint16_t&)msgData[4] = 2601;  // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t mode() const { return (uint8_t&)msgData[headerLength + 0]; }
    void set_mode(const uint8_t mode) { (uint8_t&)msgData[headerLength + 0] = mode; }
    uint8_t gain_setting() const { return (uint8_t&)msgData[headerLength + 1]; }
    void set_gain_setting(const uint8_t gain_setting) { (uint8_t&)msgData[headerLength + 1] = gain_setting; }
    uint16_t angle() const { return (uint16_t&)msgData[headerLength + 2]; }
    void set_angle(const uint16_t angle) { (uint16_t&)msgData[headerLength + 2] = angle; }
    uint16_t transmit_duration() const { return (uint16_t&)msgData[headerLength + 4]; }
    void set_transmit_duration(const uint16_t transmit_duration)
    {
        (uint16_t&)msgData[headerLength + 4] = transmit_duration;
    }
    uint16_t sample_period() const { return (uint16_t&)msgData[headerLength + 6]; }
    void set_sample_period(const uint16_t sample_period) { (uint16_t&)msgData[headerLength + 6] = sample_period; }
    uint16_t transmit_frequency() const { return (uint16_t&)msgData[headerLength + 8]; }
    void set_transmit_frequency(const uint16_t transmit_frequency)
    {
        (uint16_t&)msgData[headerLength + 8] = transmit_frequency;
    }
    uint16_t number_of_samples() const { return (uint16_t&)msgData[headerLength + 10]; }
    void set_number_of_samples(const uint16_t number_of_samples)
    {
        (uint16_t&)msgData[headerLength + 10] = number_of_samples;
    }
    uint8_t transmit() const { return (uint8_t&)msgData[headerLength + 12]; }
    void set_transmit(const uint8_t transmit) { (uint8_t&)msgData[headerLength + 12] = transmit; }
    uint8_t reserved() const { return (uint8_t&)msgData[headerLength + 13]; }
    void set_reserved(const uint8_t reserved) { (uint8_t&)msgData[headerLength + 13] = reserved; }
};

class ping360_auto_transmit : public ping_message
{
public:
    explicit ping360_auto_transmit(const ping_message& msg) : ping_message { msg } {}
    ping360_auto_transmit(const uint8_t* buf, const uint16_t length) : ping_message { buf, length } {}
    ping360_auto_transmit() : ping_message { static_cast<uint16_t>(16) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = static_cast<uint16_t>(6);  // payload size
        (uint16_t&)msgData[4] = 2602;  // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint16_t start_angle() const { return (uint16_t&)msgData[headerLength + 0]; }
    void set_start_angle(const uint16_t start_angle) { (uint16_t&)msgData[headerLength + 0] = start_angle; }
    uint16_t stop_angle() const { return (uint16_t&)msgData[headerLength + 2]; }
    void set_stop_angle(const uint16_t stop_angle) { (uint16_t&)msgData[headerLength + 2] = stop_angle; }
    uint8_t num_steps() const { return (uint8_t&)msgData[headerLength + 4]; }
    void set_num_steps(const uint8_t num_steps) { (uint8_t&)msgData[headerLength + 4] = num_steps; }
    uint8_t delay() const { return (uint8_t&)msgData[headerLength + 5]; }
    void set_delay(const uint8_t delay) { (uint8_t&)msgData[headerLength + 5] = delay; }
};

class ping360_device_id : public ping_message
{
public:
    explicit ping360_device_id(const ping_message& msg) : ping_message { msg } {}
    ping360_device_id(const uint8_t* buf, const uint16_t length) : ping_message { buf, length } {}
    ping360_device_id() : ping_message { static_cast<uint16_t>(12) }
    {
        msgData[0] = 'B';
        msgData[1] = 'R';
        (uint16_t&)msgData[2] = static_cast<uint16_t>(2);  // payload size
        (uint16_t&)msgData[4] = 2000;  // ID
        msgData[6] = 0;
        msgData[7] = 0;
    }

    uint8_t id() const { return (uint8_t&)msgData[headerLength + 0]; }
    void set_id(const uint8_t id) { (uint8_t&)msgData[headerLength + 0] = id; }
    uint8_t reserved() const { return (uint8_t&)msgData[headerLength + 1]; }
    void set_reserved(const uint8_t reserved) { (uint8_t&)msgData[headerLength + 1] = reserved; }
};

}  // namespace ping_nodelet

#endif  // PING_NODELET_PING_MESSAGE_PING360_H