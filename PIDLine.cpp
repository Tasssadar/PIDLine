#include "3piLibPack.h"
#include "packet.h"

#if 0
int16_t p_val = 20;
int16_t i_val = 10000;
int16_t d_val1 = 3;
int16_t d_val2 = 2;
int16_t max_spd = 60;
#endif

#if 0
int16_t p_val = 3;
int16_t i_val = 20000;
int16_t d_val1 = 50;
int16_t d_val2 = 1;
int16_t max_spd = 255;
#endif

#if 1
uint8_t p_val1 = 2;
uint8_t p_val2 = 5;
int16_t i_val = 7000;
int16_t d_val1 = 60;
int16_t d_val2 = 1;
int16_t max_spd = 255;
#endif

bool stopped = true;
bool calibrated = false;
int16_t last_P = 0;
int32_t I = 0;
bool on_line = false;
uint16_t on_line_counter = 0;

#define POWER_SAFE_SLOW 90
#define POWER_SAFE_FAST 255

static const uint32_t POWER_SAFE_FAST_T = 20000;
static const uint32_t POWER_SAFE_SLOW_T = uint32_t(100000);

enum followMode
{
    MODE_FAST = 0,
    MODE_NORMAL,
    MODE_SLOW,
    MODE_POWER_SAFE,

    MODE_COUNT
};

uint8_t mode = 0;

void handlePkt(Packet& pkt);
void dumpData(int16_t P, int16_t D, int32_t I, uint16_t last_P,
              int16_t pwr_diff);
void stopStart();
void follow();
void inceraseMode();
void setModeVals();
void executeMode();
void showModeDisplay();
void loadEEPROM();
void calibrate();

void run()
{
    loadEEPROM();
    display.printNumToXY(getBatteryVoltage(), 4, 0);
    setModeVals();

    char ch;
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

        if(isPressed(BUTTON_A))
        {
            waitForRelease(BUTTON_A);
            inceraseMode();
        }
        if(isPressed(BUTTON_B))
        {
            waitForRelease(BUTTON_B);
            stopStart();
            continue;
        }

        if(stopped)
        {
             if(isPressed(BUTTON_C))
            {
                waitForRelease(BUTTON_C);
                delay(300);
                calibrate();
            }
        }
        else
        {
            executeMode();
            follow();
        }
    }
}


void follow()
{
    static int16_t off_P = 0;

    int16_t pos = getLinePos(&on_line);

    if(!on_line)
    {
        if(on_line_counter == 0)
            off_P = last_P;

        if(++on_line_counter >= 400)
        {
            rs232.dumpNumber(off_P);
            rs232.dumpNumber(pos);
            rs232.dumpNumber(I);
            rs232.sendCharacter('\n');
            setMotorPower(0, 0);
            stopStart();
            return;
        }
    }
    else
        on_line_counter = 0;

    int16_t P = ((int16_t)pos) - 2048;
    int16_t D = P - last_P;
    I += P;

    int16_t pwr_diff = P*p_val1/p_val2 + I/i_val + D*d_val1/d_val2;

    if(pwr_diff > max_spd)
        pwr_diff = max_spd;
    else if(pwr_diff < -max_spd)
        pwr_diff = -max_spd;

    if(pwr_diff < 0)
        setMotorPower(max_spd+pwr_diff, max_spd);
    else
        setMotorPower(max_spd, max_spd-pwr_diff);

    last_P = P;
}

void stopStart()
{
    if(stopped)
    {
        delay(100);
        if(!calibrated)
            calibrate();

        stopped = false;
        last_P = 0;
        I = 0;
        on_line_counter = 0;
        resetTicks();
    }
    else
    {
        stopped = true;
        setMotorPower(0, 0);
    }
    setSoftAccel(stopped);
}

void calibrate()
{
    cal_round();
    calibrated = true;
    store_eeprom(1, uint8_t(42));
    store_sensor_cal(2);
}

void inceraseMode()
{
    if(++mode >= MODE_COUNT)
        mode = 0;

    setModeVals();
}

void setModeVals()
{
    switch(mode)
    {
        case MODE_FAST:
            max_spd = 255;
            break;
        case MODE_NORMAL:
            max_spd = 140;
            break;
        case MODE_SLOW:
            max_spd = 80;
            break;
        case MODE_POWER_SAFE:
            max_spd = POWER_SAFE_FAST;
            resetTicks();
            break;
        default:
            mode = MODE_FAST;
            setModeVals();
            break;
    }
    store_eeprom(0, mode);
    showModeDisplay();
}

void showModeDisplay()
{
    //display.printToXY("M:", 0, 1);

    static const char * modes[MODE_COUNT] = {
        "Fast    ",
        "Normal  ",
        "Slow    ",
        "Pwr save"
    };

    display.printToXY(modes[mode], 0, 1);
}

void executeMode()
{
    if(mode != MODE_POWER_SAFE)
        return;

    const uint32_t ticks_cur = getTicksCount();
    if(max_spd == POWER_SAFE_FAST)
    {
        if(ticks_cur >= POWER_SAFE_FAST_T)
        {
            resetTicks();
            max_spd = POWER_SAFE_SLOW;
        }
    }
    else
    {
        if(ticks_cur >= POWER_SAFE_SLOW_T)
        {
            resetTicks();
            max_spd = POWER_SAFE_FAST;
        }
    }
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
            p_val1 = pkt.read8();
            p_val2 = pkt.read8();
            i_val = pkt.read16();
            d_val1 = pkt.read16();
            d_val2 = pkt.read16();
            max_spd = pkt.read16();
            break;
        }
        case SMSG_SET_MOTORS:
        {
            if(!stopped)
                break;
            setRightMotor(pkt.read16());
            setLeftMotor(pkt.read16());
            break;
        }
    }
}

void dumpData(int16_t P, int16_t D, int32_t I, uint16_t last_P,
              int16_t pwr_diff)
{
    Packet pkt(0);
    pkt.write16(P);         // 0
    pkt.write16(D);         // 2
    pkt.write32(I);         // 4
    pkt.write16(last_P);    // 8
    pkt.write16(pwr_diff);  // 10

    pkt.send();
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

void loadEEPROM()
{
    mode = load_eeprom<uint8_t>(0);
    if(mode >= MODE_COUNT)
        mode = MODE_FAST;

    uint8_t cal = load_eeprom<uint8_t>(1);
    if(cal == 42)
    {
        load_sensor_cal(2);
        calibrated = true;
    }
}
