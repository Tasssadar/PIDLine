#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

enum opcodes
{
    SMSG_SET_PID_VALS = 0x01,
    SMSG_START_STOP   = 0x02,
};

class rs232;

class Packet
{
public:
    Packet();
    Packet(uint8_t cmd);

    void setCmd(uint8_t cmd)
    {
        m_cmd = cmd;
    }

    void clear();
    bool add(uint8_t ch);
    bool isValid()
    {
        return (m_len && 3+m_len == m_recv);
    }
    void send();

    uint8_t getCmd() const { return m_cmd; }

    void write8(const uint8_t& v);
    void write16(const int16_t& v);
    void write32(const uint32_t& v);

    uint8_t read8();
    uint16_t read16();
    uint32_t read32();

private:
    uint8_t m_len;
    uint8_t m_cmd;
    uint8_t m_data[20];

    uint8_t m_recv;
    uint8_t m_ritr;
};

#endif