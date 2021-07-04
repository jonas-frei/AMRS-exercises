#ifndef MOVESERVO_HPP_
#define MOVESERVO_HPP_

#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Sequence.hpp>

class MoveServo : public eeros::sequencer::Step
{
public:
    MoveServo(std::string name, eeros::sequencer::Sequence *caller,
              ControlSystem &cs)
        : eeros::sequencer::Step(name, caller),
          cs(cs)
    {
        log.info() << "Step created: " << name;
    }

    int operator()(int channel, double targetPos)
    {
        this->channel = channel;
        this->targetPos = targetPos;
        return start();
    }

    int action()
    {
        cs.servoController.setTargetPos(channel, targetPos);
        log.info() << "Servo number " << channel << " starting to move to position " << targetPos;
        return 0;
    }

    bool checkExitCondition()
    {
        if (cs.servoController.getStatus(channel))
        {
            log.info() << "Servo number " << channel << "reached the position " << targetPos;
            return true;
        }
        return false;
    }

private:
    ControlSystem &cs;
    int channel;
    double targetPos;
};

#endif // MOVESERVO_HPP_
