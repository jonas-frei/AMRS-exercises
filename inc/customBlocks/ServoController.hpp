#ifndef SERVOCONTROLLER_HPP_
#define SERVOCONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/core/Fault.hpp>
#include <rc/servo.h>
#include <array>

using namespace eeros::control;

template <uint8_t NrOfServos = 2>
class ServoController : public Block
{
public:
    ServoController(std::array<int, NrOfServos> channels,
                    std::array<double, NrOfServos> initPos,
                    double inc)
        : ch(channels), initPos(initPos), pos(initPos), targetPos(initPos), inc(inc)
    {
        if (rc_servo_init())
            throw eeros::Fault("Unable to initialise servo controller");
        rc_servo_power_rail_en(1);
    }
    ~ServoController()
    {
        rc_servo_power_rail_en(0);
        rc_servo_cleanup();
    }

    virtual void run()
    {
        for (int i = 0; i < NrOfServos; i++)
        {
            if (pos[i] < targetPos[i])
            {
                pos[i] += inc;
                if (pos[i] > targetPos[i])
                {
                    pos[i] = targetPos[i];
                    targetReached[i] = true;
                }
            }
            else if (pos[i] > targetPos[i])
            {
                pos[i] -= inc;
                if (pos[i] < targetPos[i])
                {
                    pos[i] = targetPos[i];
                    targetReached[i] = true;
                }
            }
            rc_servo_send_pulse_normalized(ch[i], pos[i]);
        }
    }

    void setTargetPos(int channel, double target)
    {
        targetPos[channel - 1] = target;
        targetReached[channel - 1] = false;
    }

    bool getStatus(int channel)
    {
        return targetReached[channel - 1];
    }

protected:
    std::array<int, NrOfServos> ch;
    std::array<double, NrOfServos> initPos, pos, targetPos;
    double inc;
    std::array<bool, NrOfServos> targetReached;
};

#endif //SERVOCONTROLLER_HPP_
