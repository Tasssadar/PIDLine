#include "packet.h"

Packet::Packet()
{
    clear();
}

Packet::Packet(uint8_t cmd)
{
    clear();
    m_cmd = cmd;
}

void Packet::clear()
{
    m_cmd = 0;
    m_len = 0;
    m_ritr = 0;
    m_recv = 0;
}

bool Packet::add(uint8_t ch)
{
    switch(m_recv)
    {
        case 0:
            if(ch != 0xFF)
                return false;
            break;
        case 1:
            if(ch != 0x00)
                return false;
            break;
        case 2:
            m_len = ch;
            break;
        case 3:
            m_cmd = ch;
            break;
        default:
        {
            if(m_recv-3 >= m_len)
                return false;

            m_data[m_recv - 4] = ch;
            break;
        }
    }
    ++m_recv;
    return true;
}

void Packet::write8(const uint8_t& v)
{
    m_data[m_len++] = v;
}

void Packet::write16(const uint16_t& v)
{
    m_data[m_len++] = (v >> 8);
    m_data[m_len++] = (v & 0xFF);
}

void Packet::write32(const uint32_t& v)
{
    m_data[m_len++] = (v >> 24);
    m_data[m_len++] = (v >> 16);
    m_data[m_len++] = (v >> 8);
    m_data[m_len++] = (v & 0xFF);
}

uint8_t Packet::read8()
{
    return m_data[m_ritr++];
}

uint16_t Packet::read16()
{
    uint16_t res = (m_data[m_ritr++] << 8);
    res |= m_data[m_ritr++];
    return res;
}

uint32_t Packet::read32()
{
    uint32_t res = ((uint32_t)m_data[m_ritr++] << 24);
    res |= ((uint32_t)m_data[m_ritr++] << 16);
    res |= (m_data[m_ritr++] << 8);
    res |= m_data[m_ritr++];
    return res;
}
