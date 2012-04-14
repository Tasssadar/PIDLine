#include "3piLibPack.h"
#include "packet.h"

void updateDisplay()
{
    display.printNumToXY(getBatteryVoltage(), 4, 0);
}

void dumpData(const int16_t& P, const int16_t& D, const int32_t& I, const uint16_t& last_P,
              const int16_t& pwr_diff, const int16_t& l, const int16_t& r)
{
    Packet pkt(0);
    pkt.write16(P);         // 0
    pkt.write16(D);         // 2
    pkt.write32(I);         // 4
    pkt.write16(last_P);    // 8
    pkt.write16(pwr_diff);  // 10
    pkt.write16(r);         // 12
    pkt.write16(l);         // 14

    pkt.send();
}

void handlePkt(Packet& pkt);

volatile int16_t p_val = 20;
volatile int16_t i_val = 10000;
volatile uint8_t d_val1 = 3;
volatile uint8_t d_val2 = 2;
volatile int16_t max_spd = 60;

bool stopped = true;
bool calibrated = false;


void stopStart()
{
    if(stopped)
    {
        delay(500);
        if(!calibrated)
            cal_round();
        calibrated = true;
        stopped = false;
    }
    else
    {
        stopped = true;
        setMotorPower(0, 0);
    }
}

void run()
{
    setSoftAccel(false);
    updateDisplay();


    char ch;
    uint16_t last_P = 0;
    int32_t I = 0;
    int16_t r = 0;
    int16_t l = 0;

    Packet pkt;

    while(true)
    {
        while(rs232.peek(ch))
        {
            pkt.add(ch);
            if(pkt.isValid())
            {
                handlePkt(pkt);
                pkt.clear();
            }
        }

        if(stopped)
        {
            if(!isPressed(BUTTON_B))
                continue;

            waitForRelease(BUTTON_B);
            stopStart();
        }
        else
        {
            uint16_t pos = getLinePos();
            int16_t P = ((int16_t)pos) - 2048;
            int16_t D = P - last_P;
            I += P;

            int16_t pwr_diff = P/p_val + I/i_val + D*d_val1/d_val2;

            if(pwr_diff > max_spd)
                pwr_diff = max_spd;
            else if(pwr_diff < -max_spd)
                pwr_diff = -max_spd;

            if(pwr_diff < 0)
            {
                l = max_spd+pwr_diff;
                r = max_spd;
            }
            else
            {
                l = max_spd;
                r = max_spd-pwr_diff;
            }

            dumpData(P, D, I, last_P, pwr_diff, l, r);
            setMotorPower(l, r);

            last_P = P;
        }
    }
#if 0
        // Get the position of the line.  Note that we *must* provide
        // the "sensors" argument to read_line() here, even though we
        // are not interested in the individual sensor readings.
        unsigned int position = read_line(sensors,IR_EMITTERS_ON);

        // The "proportional" term should be 0 when we are on the line.
        int proportional = ((int)position) - 2000;

        // Compute the derivative (change) and integral (sum) of the
        // position.
        int derivative = proportional - last_proportional;
        integral += proportional;

        // Remember the last position.
        last_proportional = proportional;

        // Compute the difference between the two motor power settings,
        // m1 - m2.  If this is a positive number the robot will turn
        // to the right.  If it is a negative number, the robot will
        // turn to the left, and the magnitude of the number determines
        // the sharpness of the turn.
        int power_difference = proportional/20 + integral/10000 + derivative*3/2;

        // Compute the actual motor settings.  We never set either motor
        // to a negative value.
        const int max = 60;
        if(power_difference > max)
            power_difference = max;
        if(power_difference < -max)
            power_difference = -max;

        if(power_difference < 0)
            set_motors(max+power_difference, max);
        else
            set_motors(max, max-power_difference);
#endif
}

void handlePkt(Packet& pkt)
{
    switch(pkt.getCmd())
    {
        case SMSG_START_STOP:
        {
            stopStart();
            break;
        }
        case SMSG_SET_PID_VALS:
        {
            p_val = pkt.read16();
            i_val = pkt.read16();
            d_val1 = pkt.read8();
            d_val2 = pkt.read8();
            max_spd = pkt.read16();
            break;
        }
    }
}

void Packet::send()
{
    rs232.sendCharacter(0xFF);
    rs232.sendCharacter(0x00);
    rs232.sendCharacter(m_len+1);
    rs232.sendCharacter(m_cmd);

    for(uint8_t i = 0; i < m_len; ++i)
        rs232.sendCharacter(m_data[i]);
}
